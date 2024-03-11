use core::default::Default;

use defmt::{error, info, unwrap, Debug2Format};
use embedded_hal::timer::CountDown as _;
use fugit::ExtU32;
use packed_struct::{derive::PackedStruct, PackedStruct};
use rp2040_hal::timer::CountDown;
use usb_device::{
    bus::{UsbBus, UsbBusAllocator},
    device::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_human_interface_device::{
    descriptor::InterfaceProtocol,
    device::DeviceClass,
    interface::{
        InBytes64, Interface, InterfaceBuilder, InterfaceConfig, OutBytes64, ReportSingle,
        UsbAllocatable,
    },
    usb_class::UsbHidClassBuilder,
    UsbHidError,
};

use crate::input::GCC_STATE;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct)]
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
pub struct GcConfig<'a> {
    interface: InterfaceConfig<'a, InBytes64, OutBytes64, ReportSingle>,
}

impl<'a> GcConfig<'a> {
    #[must_use]
    pub fn new(interface: InterfaceConfig<'a, InBytes64, OutBytes64, ReportSingle>) -> Self {
        Self { interface }
    }
}

impl<'a> Default for GcConfig<'a> {
    #[must_use]
    fn default() -> Self {
        let i = unwrap!(
            unwrap!(unwrap!(InterfaceBuilder::new(GCC_REPORT_DESCRIPTOR))
                .boot_device(InterfaceProtocol::None)
                .description("NaxGCC")
                .in_endpoint(1.millis()))
            .with_out_endpoint(1.millis())
        );

        Self::new(i.build())
    }
}

impl<'a, B: UsbBus + 'a> UsbAllocatable<'a, B> for GcConfig<'a> {
    type Allocated = GcController<'a, B>;

    fn allocate(self, usb_alloc: &'a UsbBusAllocator<B>) -> Self::Allocated {
        Self::Allocated {
            interface: Interface::new(usb_alloc, self.interface),
        }
    }
}

pub struct GcController<'a, B: UsbBus> {
    interface: Interface<'a, B, InBytes64, OutBytes64, ReportSingle>,
}

impl<'a, B: UsbBus> GcController<'a, B> {
    pub fn write_report(&mut self, report: &GcReport) -> Result<(), UsbHidError> {
        let report = get_gcinput_hid_report(report);

        self.interface
            .write_report(&report)
            .map(|_| ())
            .map_err(|e| UsbHidError::from(e))
    }

    pub fn read_report(&mut self) -> Result<RawConsoleReport, UsbHidError> {
        let mut report = RawConsoleReport::default();
        match self.interface.read_report(&mut report.packet) {
            Err(e) => Err(UsbHidError::from(e)),
            Ok(_) => Ok(report),
        }
    }
}

impl<'a, B: UsbBus> DeviceClass<'a> for GcController<'a, B> {
    type I = Interface<'a, B, InBytes64, OutBytes64, ReportSingle>;

    fn interface(&mut self) -> &mut Self::I {
        &mut self.interface
    }

    fn reset(&mut self) {}

    fn tick(&mut self) -> Result<(), UsbHidError> {
        Ok(())
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

pub fn usb_transfer_loop<'a, T: UsbBus>(
    usb_bus: UsbBusAllocator<T>,
    mut poll_timer: CountDown<'a>,
) -> ! {
    info!("Got to this point");
    let mut gcc = UsbHidClassBuilder::new()
        .add_device(GcConfig::default())
        .build(&usb_bus);

    info!("Got the gc");

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x057e, 0x0337))
        .manufacturer("Naxdy")
        .product("NaxGCC")
        .serial_number("fleeb") // TODO: Get this from the flash unique id
        .device_class(0)
        .device_protocol(0)
        .device_sub_class(0)
        .self_powered(false)
        .max_power(500)
        .max_packet_size_0(64)
        .build();

    poll_timer.start(1.millis());

    info!("Got here");

    loop {
        if poll_timer.wait().is_ok() {
            match gcc.device().write_report(&(unsafe { GCC_STATE })) {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    error!("Error: {:?}", Debug2Format(&e));
                    panic!();
                }
            }
        }
        if usb_dev.poll(&mut [&mut gcc]) {
            match gcc.device().read_report() {
                Err(UsbHidError::WouldBlock) => {}
                Err(e) => {
                    error!("Failed to read report: {:?}", Debug2Format(&e));
                }
                Ok(report) => {
                    info!("Received report: {:08x}", report.packet);
                    // rumble packet
                    if report.packet[0] == 0x11 {
                        info!("Received rumble info: Controller1 ({:08x}) Controller2 ({:08x}) Controller3 ({:08x}) Controller4 ({:08x})", report.packet[1], report.packet[2], report.packet[3], report.packet[4]);
                    }
                }
            }
        }
    }
}
