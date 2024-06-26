/**
 * Communication with the console / PC over USB HID.
 * Includes the HID report descriptor, and the GcReport struct.
 */
use core::default::Default;

use defmt::{debug, info, trace, warn, Format};
use embassy_futures::join::join;
use embassy_rp::{
    peripherals::{PIN_25, PIN_29, PWM_CH4, PWM_CH6, USB},
    pwm::Pwm,
    usb::Driver,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::{
    class::hid::{HidReaderWriter, ReportId, RequestHandler, State},
    control::OutResponse,
    msos::{self, windows_version},
    Builder, Handler,
};
use libm::powf;
use packed_struct::{derive::PackedStruct, PackedStruct};

use crate::{config::InputConsistencyMode, input::CHANNEL_GCC_STATE};

static SIGNAL_RUMBLE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// We could turn the config change signal into a PubSubChannel instead, but that
/// would just transmit unnecessary amounts of data.
pub static SIGNAL_CHANGE_RUMBLE_STRENGTH: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Only dispatched ONCE after powerup, to determine how to advertise itself via USB.
pub static MUTEX_INPUT_CONSISTENCY_MODE: Mutex<
    CriticalSectionRawMutex,
    Option<InputConsistencyMode>,
> = Mutex::new(None);

/// Vendor-defined property data
const DEVICE_INTERFACE_GUID: &str = "{ecceff35-146c-4ff3-acd9-8f992d09acdd}";

#[rustfmt::skip]
pub const GCC_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0xA1, 0x03,        //   Collection (Report)
    0x85, 0x11,        //     Report ID (17)
    0x19, 0x00,        //     Usage Minimum (Undefined)
    0x2A, 0xFF, 0x00,  //     Usage Maximum (0xFF)
    0x15, 0x00,        //     Logical Minimum (0)
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x05,        //     Report Count (5)
    0x91, 0x00,        //     Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              //   End Collection
    0xA1, 0x03,        //   Collection (Report)
    0x85, 0x21,        //     Report ID (33)
    0x05, 0x00,        //     Usage Page (Undefined)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0xFF,        //     Logical Maximum (-1)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x08,        //     Usage Maximum (0x08)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x33,        //     Usage (Rx)
    0x09, 0x34,        //     Usage (Ry)
    0x09, 0x35,        //     Usage (Rz)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x06,        //     Report Count (6)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xA1, 0x03,        //   Collection (Report)
    0x85, 0x13,        //     Report ID (19)
    0x19, 0x00,        //     Usage Minimum (Undefined)
    0x2A, 0xFF, 0x00,  //     Usage Maximum (0xFF)
    0x15, 0x00,        //     Logical Minimum (0)
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x91, 0x00,        //     Output (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              //   End Collection
    0xC0,              // End Collection
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct Buttons1 {
    #[packed_field(bits = "0")]
    pub button_a: bool,
    #[packed_field(bits = "1")]
    pub button_b: bool,
    #[packed_field(bits = "2")]
    pub button_x: bool,
    #[packed_field(bits = "3")]
    pub button_y: bool,
    #[packed_field(bits = "4")]
    pub dpad_left: bool,
    #[packed_field(bits = "5")]
    pub dpad_right: bool,
    #[packed_field(bits = "6")]
    pub dpad_down: bool,
    #[packed_field(bits = "7")]
    pub dpad_up: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct Buttons2 {
    #[packed_field(bits = "0")]
    pub button_start: bool,
    #[packed_field(bits = "1")]
    pub button_z: bool,
    #[packed_field(bits = "2")]
    pub button_r: bool,
    #[packed_field(bits = "3")]
    pub button_l: bool,
    #[packed_field(bits = "4..=7")]
    pub blank1: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PackedStruct, Format)]
#[packed_struct(bit_numbering = "msb0", size_bytes = "8")]
pub struct GcReport {
    #[packed_field(bits = "0..=7")]
    pub buttons_1: Buttons1,
    #[packed_field(bits = "8..=15")]
    pub buttons_2: Buttons2,
    #[packed_field(bits = "16..=23")]
    pub stick_x: u8,
    #[packed_field(bits = "24..=31")]
    pub stick_y: u8,
    #[packed_field(bits = "32..=39")]
    pub cstick_x: u8,
    #[packed_field(bits = "40..=47")]
    pub cstick_y: u8,
    #[packed_field(bits = "48..=55")]
    pub trigger_l: u8,
    #[packed_field(bits = "56..=63")]
    pub trigger_r: u8,
}

impl Default for GcReport {
    fn default() -> Self {
        Self {
            buttons_1: Buttons1::default(),
            buttons_2: Buttons2::default(),
            stick_x: 127,
            stick_y: 127,
            cstick_x: 127,
            cstick_y: 127,
            trigger_l: 0,
            trigger_r: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(C, align(8))]
pub struct RawConsoleReport {
    pub packet: [u8; 64],
}

impl Default for RawConsoleReport {
    fn default() -> Self {
        Self { packet: [0u8; 64] }
    }
}

struct GccRequestHandler {}

impl RequestHandler for GccRequestHandler {
    fn get_report(&self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

fn get_gcinput_hid_report(input_state: &GcReport) -> [u8; 37] {
    static mut GC_FIRST: bool = false;

    let mut buffer = [0u8; 37];

    buffer[0] = 0x21;
    buffer[1] |= 0x14;

    let data = input_state.pack().expect("Failed to pack GC input data");

    if unsafe { !GC_FIRST } {
        buffer[1] |= 0x04;
        buffer[10] |= 0x04;
        buffer[19] |= 0x04;
        buffer[28] |= 0x04;
        unsafe { GC_FIRST = true };
    } else {
        // controller in "port 1"
        buffer[2..=9].copy_from_slice(&data[0..=7]);
    }

    buffer
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

#[embassy_executor::task]
pub async fn usb_transfer_task(raw_serial: [u8; 8], driver: Driver<'static, USB>) {
    let input_consistency_mode = {
        while MUTEX_INPUT_CONSISTENCY_MODE.lock().await.is_none() {
            Timer::after(Duration::from_millis(100)).await;
        }
        MUTEX_INPUT_CONSISTENCY_MODE.lock().await.unwrap()
    };

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
    let mut usb_config = embassy_usb::Config::new(0x057e, 0x0337);
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

    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let request_handler = GccRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut device_descriptor,
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

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: GCC_REPORT_DESCRIPTOR,
        request_handler: Some(&request_handler),
        poll_ms: match input_consistency_mode {
            InputConsistencyMode::Original => 8,
            InputConsistencyMode::ConsistencyHack
            | InputConsistencyMode::SuperHack
            | InputConsistencyMode::PC => 1,
        },
        max_packet_size_in: 37,
        max_packet_size_out: 5,
    };
    let hid = HidReaderWriter::<_, 5, 37>::new(&mut builder, &mut state, hid_config);

    let mut usb = builder.build();

    let usb_fut = async {
        loop {
            usb.run_until_suspend().await;
            debug!("Suspended");
            usb.wait_resume().await;
            debug!("RESUMED!");
        }
    };

    let (mut reader, mut writer) = hid.split();

    let in_fut = async {
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

            match writer
                .write(&{
                    let state = gcc_subscriber.next_message_pure().await;
                    let report = get_gcinput_hid_report(&state);
                    trace!("Report Written: {:08b}", report);
                    report
                })
                .await
            {
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

    let out_fut = async {
        loop {
            trace!("Readery loop");
            let mut buf = [0u8; 5];
            match reader.read(&mut buf).await {
                Ok(_e) => {
                    debug!("READ SOMETHIN: {:08b}", buf);
                    SIGNAL_RUMBLE.signal((buf[1] & 0x01) != 0);
                }
                Err(e) => {
                    warn!("Failed to read: {:?}", e);
                }
            }
        }
    };

    let usb_fut_wrapped = async {
        usb_fut.await;
        debug!("USB FUT DED");
    };

    join(usb_fut_wrapped, join(in_fut, out_fut)).await;
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
    pwm_ch_rumble: PWM_CH4,
    pwm_ch_brake: PWM_CH6,
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
