/**
 * Communication with the console / PC over USB HID.
 * Includes the HID report descriptor, and the GcReport struct.
 */
use core::{default::Default, future::Future};

use defmt::{debug, info, trace, warn};
use embassy_futures::join::join;
use embassy_rp::{
    peripherals::{PIN_25, PIN_29, PWM_SLICE4, PWM_SLICE6, USB},
    pwm::Pwm,
    usb::Driver as EmbassyDriver,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::{
    class::hid::{HidReader, HidReaderWriter, HidWriter, RequestHandler, State},
    driver::Driver,
    msos::{self, windows_version},
    Builder, Handler, UsbDevice,
};
use libm::powf;

use crate::{
    config::{ControllerMode, InputConsistencyMode},
    hid::{
        gcc::{GcReportBuilder, GcState, GccRequestHandler, GCC_REPORT_DESCRIPTOR},
        procon::{ProconReportBuilder, ProconRequestHandler, PROCON_REPORT_DESCRIPTOR},
    },
    input::CHANNEL_GCC_STATE,
};

pub static SIGNAL_RUMBLE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// We could turn the config change signal into a PubSubChannel instead, but that
/// would just transmit unnecessary amounts of data.
pub static SIGNAL_CHANGE_RUMBLE_STRENGTH: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Only dispatched ONCE after powerup, to determine how to advertise itself via USB.
pub static MUTEX_INPUT_CONSISTENCY_MODE: Mutex<
    CriticalSectionRawMutex,
    Option<InputConsistencyMode>,
> = Mutex::new(None);

/// Only dispatched ONCE after powerup, to determine how to advertise itself via USB.
pub static MUTEX_CONTROLLER_MODE: Mutex<CriticalSectionRawMutex, Option<ControllerMode>> =
    Mutex::new(None);

/// Vendor-defined property data
const DEVICE_INTERFACE_GUID: &str = "{ecceff35-146c-4ff3-acd9-8f992d09acdd}";

pub trait HidReportBuilder<const LEN: usize> {
    async fn get_hid_report(&mut self, state: &GcState) -> [u8; LEN];
}

struct UsbConfig {
    vid: u16,
    pid: u16,
    report_descriptor: &'static [u8],
}

impl From<ControllerMode> for UsbConfig {
    fn from(value: ControllerMode) -> Self {
        match value {
            ControllerMode::GcAdapter => Self {
                vid: 0x057e,
                pid: 0x0337,
                report_descriptor: GCC_REPORT_DESCRIPTOR,
            },
            ControllerMode::Procon => Self {
                vid: 0x57e,
                pid: 0x2009,
                report_descriptor: PROCON_REPORT_DESCRIPTOR,
            },
        }
    }
}

struct MyDeviceHandler {
    configured: bool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler { configured: false }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured = true;

        // reason: in production, info! compiles to nothing
        #[allow(clippy::if_same_then_else)]
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured = false;
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured = false;
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured = configured;
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}

fn mk_hid_reader_writer<'d, D: Driver<'d>, const R: usize, const W: usize>(
    input_consistency_mode: InputConsistencyMode,
    report_descriptor: &'d [u8],
    mut builder: Builder<'d, D>,
    state: &'d mut State<'d>,
) -> (UsbDevice<'d, D>, HidReader<'d, D, R>, HidWriter<'d, D, W>) {
    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor,
        request_handler: None,
        poll_ms: match input_consistency_mode {
            InputConsistencyMode::Original => 8,
            InputConsistencyMode::ConsistencyHack
            | InputConsistencyMode::SuperHack
            | InputConsistencyMode::PC => 1,
        },
        max_packet_size_in: W as u16,
        max_packet_size_out: R as u16,
    };

    let hid: HidReaderWriter<'d, D, R, W> =
        HidReaderWriter::<'_, D, R, W>::new(&mut builder, state, hid_config);

    let usb = builder.build();

    let (reader, writer) = hid.split();

    (usb, reader, writer)
}

fn mk_usb_transfer_futures<'d, D, H, Rq, const R: usize, const W: usize>(
    input_consistency_mode: InputConsistencyMode,
    usb_config: &UsbConfig,
    request_handler: &'d mut Rq,
    builder: Builder<'d, D>,
    state: &'d mut State<'d>,
    mut hid_report_builder: H,
) -> (
    impl Future<Output = ()> + 'd,
    impl Future<Output = ()> + 'd,
    impl Future<Output = ()> + 'd,
)
where
    D: Driver<'d> + 'd,
    H: HidReportBuilder<W> + 'd,
    Rq: RequestHandler,
{
    let (mut usb, reader, mut writer) = mk_hid_reader_writer::<_, R, W>(
        input_consistency_mode,
        usb_config.report_descriptor,
        builder,
        state,
    );

    let usb_fut = async move {
        loop {
            usb.run_until_suspend().await;
            debug!("Suspended");
            usb.wait_resume().await;
            debug!("RESUMED!");
        }
    };

    let in_fut = async move {
        let mut gcc_subscriber = CHANNEL_GCC_STATE.subscriber().unwrap();

        let mut last_report_time = Instant::now();
        let mut rate_limit_end_time = Instant::now();

        loop {
            // This is what we like to call a "hack".
            // It forces reports to be sent at least every 8.33ms instead of every 8ms.
            // 8.33ms is a multiple of the game's frame interval (16.66ms), so if we
            // send a report every 8.33ms, it should (in theory) ensure (close to)
            // 100% input accuracy.
            //
            // From the console's perspective, we are basically a laggy adapter, taking
            // a minimum of 333 extra us to send a report every time it's polled, but it
            // works to our advantage.
            match input_consistency_mode {
                InputConsistencyMode::SuperHack | InputConsistencyMode::ConsistencyHack => {
                    // "Ticker at home", so we can use this for both consistency and SuperHack mode
                    Timer::at(rate_limit_end_time).await;
                }
                InputConsistencyMode::Original | InputConsistencyMode::PC => {}
            }

            writer.ready().await;

            let state = gcc_subscriber.next_message_pure().await;

            let report = hid_report_builder.get_hid_report(&state).await;

            trace!("Writing report: {:08b}", report);

            match writer.write(&report).await {
                Ok(()) => {
                    let currtime = Instant::now();
                    let polltime = currtime.duration_since(last_report_time);
                    let micros = polltime.as_micros();
                    debug!("Report written in {}us", micros);
                    if input_consistency_mode != InputConsistencyMode::Original
                        && input_consistency_mode != InputConsistencyMode::PC
                    {
                        while rate_limit_end_time < currtime {
                            rate_limit_end_time += Duration::from_micros(8333);
                        }
                    }
                    last_report_time = currtime;
                }
                Err(e) => warn!("Failed to send report: {:?}", e),
            }
        }
    };

    let out_fut = async move {
        trace!("Readery loop");
        reader.run(true, request_handler).await;
    };

    let usb_fut_wrapped = async {
        usb_fut.await;
        debug!("USB FUT DED");
    };

    (usb_fut_wrapped, in_fut, out_fut)
}

#[embassy_executor::task]
pub async fn usb_transfer_task(raw_serial: [u8; 8], driver: EmbassyDriver<'static, USB>) {
    let input_consistency_mode = {
        while MUTEX_INPUT_CONSISTENCY_MODE.lock().await.is_none() {
            Timer::after(Duration::from_millis(100)).await;
        }
        MUTEX_INPUT_CONSISTENCY_MODE.lock().await.unwrap()
    };

    let controller_mode = {
        while MUTEX_CONTROLLER_MODE.lock().await.is_none() {
            Timer::after(Duration::from_millis(100)).await;
        }

        MUTEX_CONTROLLER_MODE.lock().await.unwrap()
    };

    let config = UsbConfig::from(controller_mode);

    let mut serial_buffer = [0u8; 64];

    let serial = format_no_std::show(
        &mut serial_buffer,
        format_args!(
            "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
            raw_serial[0],
            raw_serial[1],
            raw_serial[2],
            raw_serial[3],
            raw_serial[4],
            raw_serial[5],
            raw_serial[6],
            raw_serial[7]
        ),
    )
    .unwrap();

    info!("Detected flash with unique serial number {}", serial);

    trace!("Start of config");
    let mut usb_config = embassy_usb::Config::new(config.vid, config.pid);
    usb_config.manufacturer = Some("Naxdy");
    usb_config.product = Some(match input_consistency_mode {
        InputConsistencyMode::Original => "NaxGCC (OG Mode)",
        InputConsistencyMode::ConsistencyHack => "NaxGCC (Consistency Mode)",
        InputConsistencyMode::SuperHack => "NaxGCC (SuperHack Mode)",
        InputConsistencyMode::PC => "NaxGCC (PC Mode)",
    });
    usb_config.serial_number = Some(serial);
    usb_config.max_power = 200;
    usb_config.max_packet_size_0 = 64;
    usb_config.device_class = 0;
    usb_config.device_protocol = 0;
    usb_config.self_powered = false;
    usb_config.device_sub_class = 0;
    usb_config.supports_remote_wakeup = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.msos_descriptor(windows_version::WIN8_1, 2);

    let msos_writer = builder.msos_writer();
    msos_writer.device_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    msos_writer.device_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUID",
        msos::PropertyData::Sz(DEVICE_INTERFACE_GUID),
    ));

    builder.handler(&mut device_handler);

    match controller_mode {
        ControllerMode::GcAdapter => {
            let mut request_handler = GccRequestHandler;

            let (usb_fut_wrapped, in_fut, out_fut) = mk_usb_transfer_futures::<_, _, _, 5, 37>(
                input_consistency_mode,
                &config,
                &mut request_handler,
                builder,
                &mut state,
                GcReportBuilder::default(),
            );

            join(usb_fut_wrapped, join(in_fut, out_fut)).await;
        }
        ControllerMode::Procon => {
            let mut request_handler = ProconRequestHandler;

            let (usb_fut_wrapped, in_fut, out_fut) = mk_usb_transfer_futures::<_, _, _, 64, 64>(
                input_consistency_mode,
                &config,
                &mut request_handler,
                builder,
                &mut state,
                ProconReportBuilder::default(),
            );

            join(usb_fut_wrapped, join(in_fut, out_fut)).await;
        }
    };
}

fn calc_rumble_power(strength: u8) -> u16 {
    if strength > 0 {
        powf(2.0, 7.0 + ((strength as f32 - 3.0) / 8.0)) as u16
    } else {
        0
    }
}

#[embassy_executor::task]
pub async fn rumble_task(
    pin_rumble: PIN_25,
    pin_brake: PIN_29,
    pwm_ch_rumble: PWM_SLICE4,
    pwm_ch_brake: PWM_SLICE6,
) {
    let mut rumble_config: embassy_rp::pwm::Config = Default::default();
    rumble_config.top = 255;
    rumble_config.enable = true;
    rumble_config.compare_b = 0;

    let mut brake_config = rumble_config.clone();
    brake_config.compare_b = 255;

    let mut pwm_rumble = Pwm::new_output_b(pwm_ch_rumble, pin_rumble, rumble_config.clone());
    let mut pwm_brake = Pwm::new_output_b(pwm_ch_brake, pin_brake, brake_config.clone());

    let mut rumble_power = {
        let strength = SIGNAL_CHANGE_RUMBLE_STRENGTH.wait().await;
        calc_rumble_power(strength)
    };

    loop {
        let new_rumble_status = SIGNAL_RUMBLE.wait().await;

        debug!("Received rumble signal: {}", new_rumble_status);

        if let Some(new_strength) = SIGNAL_CHANGE_RUMBLE_STRENGTH.try_take() {
            rumble_power = calc_rumble_power(new_strength);
        }

        if new_rumble_status {
            rumble_config.compare_b = rumble_power;
            brake_config.compare_b = 0;

            pwm_rumble.set_config(&rumble_config);
            pwm_brake.set_config(&brake_config);
        } else {
            rumble_config.compare_b = 0;
            brake_config.compare_b = 255;

            pwm_rumble.set_config(&rumble_config);
            pwm_brake.set_config(&brake_config);
        }
    }
}
