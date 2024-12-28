///
/// The majority of the logic in this file is derived from HOJA-LIB-RP2040
/// https://github.com/HandHeldLegend/HOJA-LIB-RP2040
///
/// The original author gave their consent for this to be included in the NaxGCC firmware,
/// and for this file to be distributed under the GPLv3, see https://git.naxdy.org/NaxdyOrg/NaxGCC-FW/pulls/26
///
use core::ops::{Deref, DerefMut};

use defmt::{info, trace, Format};
use embassy_rp::clocks::RoscRng;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use embassy_usb::{
    class::hid::{ReportId, RequestHandler},
    control::OutResponse,
};
use packed_struct::{derive::PackedStruct, PackedStruct};
use rand::RngCore;

use crate::{input::ControllerState, usb_comms::HidReportBuilder};

const SW_INFO_SET_MAC: u8 = 0x01;

const SW_CMD_SET_INPUT_MODE: u8 = 0x03;
const SW_CMD_GET_DEVINFO: u8 = 0x02;
const SW_CMD_SET_SHIPMODE: u8 = 0x08;
const SW_CMD_GET_SPI: u8 = 0x10;
const SW_CMD_SET_PAIRING: u8 = 0x01;
const SW_CMD_GET_TRIGGERET: u8 = 0x04;

const ACK_GET_DEVINFO: u8 = 0x82;
const ACK_GET_SPI: u8 = 0x90;
const ACK_SET_PAIRING: u8 = 0x81;
const ACK_GET_TRIGERET: u8 = 0x83;

const ACK_GENERIC: u8 = 0x80;

const RM_SEND_STATE: u8 = 0x30;

const PRO_CONTROLLER_STRING: [u8; 24] = [
    0x00, 0x25, 0x08, 0x50, 0x72, 0x6F, 0x20, 0x43, 0x6F, 0x6E, 0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65,
    0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68,
];

#[derive(Debug, Format, Clone, Copy)]
struct ProconRequestInfo {
    report_id: ProconRequestId,
    response_id: ProconResponseId,
    command_id: u8,
    raw_data: [u8; 64],
}

#[derive(Debug, Format, Clone, Copy)]
enum ProconRequestId {
    GetInfo = 0x80,
    Command = 0x01,
    Rumble = 0x10,
}

#[derive(Debug, Format, Clone, Copy)]
enum ProconResponseId {
    GetInfo = 0x81,
    GetState = 0x30,
    Command = 0x21,
}

static SIGNAL_PROCON_REQUEST: Signal<CriticalSectionRawMutex, ProconRequestInfo> = Signal::new();

#[rustfmt::skip]
pub const PROCON_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x15, 0x00, // Logical Minimum (0)

    0x09, 0x04, // Usage (Joystick)
    0xA1, 0x01, // Collection (Application)

    0x85, 0x30, //   Report ID (48)
    0x05, 0x01, //   Usage Page (Generic Desktop Ctrls)
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (0x01)
    0x29, 0x0A, //   Usage Maximum (0x0A)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x0A, //   Report Count (10)
    0x55, 0x00, //   Unit Exponent (0)
    0x65, 0x00, //   Unit (None)
    0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x0B, //   Usage Minimum (0x0B)
    0x29, 0x0E, //   Usage Maximum (0x0E)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x04, //   Report Count (4)
    0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x02, //   Report Count (2)
    0x81, 0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x0B, 0x01, 0x00, 0x01, 0x00, //   Usage (0x010001)
    0xA1, 0x00,                   //   Collection (Physical)
    0x0B, 0x30, 0x00, 0x01, 0x00, //     Usage (0x010030)
    0x0B, 0x31, 0x00, 0x01, 0x00, //     Usage (0x010031)
    0x0B, 0x32, 0x00, 0x01, 0x00, //     Usage (0x010032)
    0x0B, 0x35, 0x00, 0x01, 0x00, //     Usage (0x010035)
    0x15, 0x00,                   //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x75, 0x10,                   //     Report Size (16)
    0x95, 0x04,                   //     Report Count (4)
    0x81, 0x02,                   //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                         //   End Collection

    0x0B, 0x39, 0x00, 0x01, 0x00, //   Usage (0x010039)
    0x15, 0x00,                   //   Logical Minimum (0)
    0x25, 0x07,                   //   Logical Maximum (7)
    0x35, 0x00,                   //   Physical Minimum (0)
    0x46, 0x3B, 0x01,             //   Physical Maximum (315)
    0x65, 0x14,                   //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,                   //   Report Size (4)
    0x95, 0x01,                   //   Report Count (1)
    0x81, 0x02,                   //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,                   //   Usage Page (Button)
    0x19, 0x0F,                   //   Usage Minimum (0x0F)
    0x29, 0x12,                   //   Usage Maximum (0x12)
    0x15, 0x00,                   //   Logical Minimum (0)
    0x25, 0x01,                   //   Logical Maximum (1)
    0x75, 0x01,                   //   Report Size (1)
    0x95, 0x04,                   //   Report Count (4)
    0x81, 0x02,                   //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x08,                   //   Report Size (8)
    0x95, 0x34,                   //   Report Count (52)
    0x81, 0x03,                   //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x85, 0x21,       //   Report ID (33)
    0x09, 0x01,       //   Usage (0x01)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x3F,       //   Report Count (63)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x81, //   Report ID (-127)
    0x09, 0x02, //   Usage (0x02)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x3F, //   Report Count (63)
    0x81, 0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x01, //   Report ID (1)
    0x09, 0x03, //   Usage (0x03)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x3F, //   Report Count (63)
    0x91, 0x83, //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x10, //   Report ID (16)
    0x09, 0x04, //   Usage (0x04)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x3F, //   Report Count (63)
    0x91, 0x83, //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x80, //   Report ID (-128)
    0x09, 0x05, //   Usage (0x05)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x3F, //   Report Count (63)
    0x91, 0x83, //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0x85, 0x82, //   Report ID (-126)
    0x09, 0x06, //   Usage (0x06)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x3F, //   Report Count (63)
    0x91, 0x83, //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Volatile)

    0xC0, // End Collection

    // 203 bytes
];

#[derive(Clone, Copy, Debug, Format)]
struct ProconByteReport([u8; 64]);

impl Deref for ProconByteReport {
    type Target = [u8; 64];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ProconByteReport {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl ProconByteReport {
    fn set_report_id(&mut self, id: u8) {
        self[0] = id;
    }

    fn set_battery_status(&mut self) {
        self[2] = BatteryStatus::default()
            .pack()
            .expect("Failed to pack fake procon battery status")[0];
    }

    fn set_timer(&mut self) {
        self[1] = Instant::now().as_millis() as u8;
    }

    fn set_ack(&mut self, ack: u8) {
        self[13] = ack;
    }

    fn set_subcommand(&mut self, cmd: u8) {
        self[14] = cmd;
    }

    fn set_devinfo(&mut self) {
        self[15] = 0x04; // NS Firmware primary   (4.x)
        self[16] = 0x33; // NS Firmware secondary (x.21)

        self[17] = 0x03; // Controller ID primary (Pro Controller)
        self[18] = 0x02; // Controller ID secondary

        self[25] = 0x01;
        self[26] = 0x02;
    }

    fn sw_spi_readfromaddress(
        &mut self,
        offset_address: u8,
        address: u8,
        length: u8,
        switch_host_address: &[u8],
    ) {
        let read_info = [address, offset_address, 0x00, 0x00, length];
        self[15..(15 + read_info.len())].copy_from_slice(&read_info);

        let mut output_spi_data = [0u8; 30];

        output_spi_data.iter_mut().enumerate().for_each(|(i, e)| {
            *e = sw_spi_getaddressdata(offset_address, address + i as u8, switch_host_address)
        });

        self[20..(20 + length as usize)].copy_from_slice(&output_spi_data[..(length as usize)]);
    }

    fn set_trigerret(&mut self, time_10_ms: u16) {
        let [upper_ms, lower_ms] = time_10_ms.to_be_bytes();

        for i in 0..14 {
            self[15 + i] = upper_ms;
            self[16 + i] = lower_ms;
        }
    }
}

fn sw_spi_getaddressdata(offset_address: u8, address: u8, switch_host_address: &[u8]) -> u8 {
    match offset_address {
        0x00 => 0x00,
        0x20..=0x40 => match address {
            0x26 | 0x00 => 0x95,
            // Size of pairing data
            0x27 | 0x01 => 0x22,
            // Checksum
            0x28 | 0x29 | 0x02 | 0x03 => 0x00,
            // Host BT address (Big-endian)
            0x2A..=0x2F => switch_host_address[(address - 0x2a) as usize],
            0x04..=0x09 => switch_host_address[(address - 4) as usize],
            // Bluetooth LTK (Little-endian) NOT IMPLEMENTED YET
            0x30..=0x3F => 0x00,
            0x0A..=0x19 => 0x00,
            // Host capability 0x68 is Nintendo Switch. 0x08 is PC
            0x4A | 0x24 => 0x68,
            0x4B | 0x25 => 0,
            _ => 0x00,
        },
        0x50 => 0x00,
        0x60 => match address {
            0x00..0x0f => 0xff,
            0x12 => 0x03,
            0x13 => 0x02,
            0x1b => 0x01,
            0x20 => 35,
            0x21 => 0,
            0x22 => 185,
            0x23 => 255,
            0x24 => 26,
            0x25 => 1,
            0x26 => 0,
            0x27 => 64,
            0x28 => 0,
            0x29 => 64,
            0x2A => 0,
            0x2B => 64,
            0x2C => 1,
            0x2D => 0,
            0x2E => 1,
            0x2F => 0,
            0x30 => 1,
            0x31 => 0,
            0x32 => 0x3B,
            0x33 => 0x34,
            0x34 => 0x3B,
            0x35 => 0x34,
            0x36 => 0x3B,
            0x37 => 0x34,
            0x3d..=0x45 => mk_switch_analog_calibration_data()[(address - 0x3d) as usize],
            0x46..=0x4e => mk_switch_analog_calibration_data()[(address - 0x3d) as usize],
            0x4F => 0xFF,
            0x50 => 26,
            0x51 => 26,
            0x52 => 26,
            0x53..=0x55 => 94,
            0x56 => 255,
            0x57 => 255,
            0x58 => 255,
            0x59..=0x5B => 255,
            0x5C => 0x01,
            0x80 => 80,
            0x81 => 253,
            0x82 => 0,
            0x83 => 0,
            0x84 => 198,
            0x85 => 15,
            0x98 | 0x86 => 15,
            0x99 | 0x87 => 48,
            0x9A | 0x88 => 97,
            0x9B | 0x89 => 174,
            0x9C | 0x8A => 144,
            0x9D | 0x8B => 217,
            0x9E | 0x8C => 212,
            0x9F | 0x8D => 20,
            0xA0 | 0x8E => 84,
            0xA1 | 0x8F => 65,
            0xA2 | 0x90 => 21,
            0xA3 | 0x91 => 84,
            0xA4 | 0x92 => 199,
            0xA5 | 0x93 => 121,
            0xA6 | 0x94 => 156,
            0xA7 | 0x95 => 51,
            0xA8 | 0x96 => 54,
            0xA9 | 0x97 => 99,
            _ => 0,
        },
        0x80 => match address {
            0x10..=0x1a => 0xff,
            0x1b..=0x25 => 0xff,
            0x26..=0x3f => 0xff,
            _ => 0xff,
        },
        _ => 0xff,
    }
}

fn mk_switch_analog_calibration_data() -> [u8; 18] {
    fn switch_analog_encode(in_lower: u16, in_upper: u16) -> [u8; 3] {
        let mut out = [0u8; 3];

        [out[0], out[1]] = in_lower.to_le_bytes();

        out[1] |= ((in_upper & 0xf) << 4) as u8;
        out[2] = ((in_upper & 0xff0) >> 4) as u8;

        out
    }

    const MIN: u16 = 128 << 4;
    const MAXX: u16 = 128 << 4;
    const CENTER: u16 = 128 << 4;

    let mut out = [0u8; 18];

    out[0..3].copy_from_slice(&switch_analog_encode(MAXX, MAXX));
    out[3..6].copy_from_slice(&switch_analog_encode(CENTER, CENTER));
    out[6..9].copy_from_slice(&switch_analog_encode(MIN, MIN));

    out[9..12].copy_from_slice(&switch_analog_encode(CENTER, CENTER));
    out[12..15].copy_from_slice(&switch_analog_encode(MIN, MIN));
    out[15..18].copy_from_slice(&switch_analog_encode(MAXX, MAXX));

    info!("Returning switch data: {:x}", out);

    out
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct ProconButtonsRight {
    #[packed_field(bits = "0")]
    pub button_y: bool,
    #[packed_field(bits = "1")]
    pub button_x: bool,
    #[packed_field(bits = "2")]
    pub button_b: bool,
    #[packed_field(bits = "3")]
    pub button_a: bool,
    #[packed_field(bits = "4")]
    pub trigger_r_sr: bool,
    #[packed_field(bits = "5")]
    pub trigger_r_sl: bool,
    #[packed_field(bits = "6")]
    pub trigger_r: bool,
    #[packed_field(bits = "7")]
    pub trigger_zr: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct ProconButtonsShared {
    #[packed_field(bits = "0")]
    pub button_minus: bool,
    #[packed_field(bits = "1")]
    pub button_plus: bool,
    #[packed_field(bits = "2")]
    pub button_sb_right: bool,
    #[packed_field(bits = "3")]
    pub button_sb_left: bool,
    #[packed_field(bits = "4")]
    pub button_home: bool,
    #[packed_field(bits = "5")]
    pub button_capture: bool,
    #[packed_field(bits = "6")]
    pub none: bool,
    #[packed_field(bits = "7")]
    pub change_grip_active: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct ProconButtonsLeft {
    #[packed_field(bits = "0")]
    pub dpad_down: bool,
    #[packed_field(bits = "1")]
    pub dpad_up: bool,
    #[packed_field(bits = "2")]
    pub dpad_right: bool,
    #[packed_field(bits = "3")]
    pub dped_left: bool,
    #[packed_field(bits = "4")]
    pub trigger_l_sr: bool,
    #[packed_field(bits = "5")]
    pub trigger_l_sl: bool,
    #[packed_field(bits = "6")]
    pub trigger_l: bool,
    #[packed_field(bits = "7")]
    pub trigger_zl: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb", size_bytes = "9")]
pub struct ProconState {
    #[packed_field(bits = "0..=7")]
    pub buttons_right: ProconButtonsRight,
    #[packed_field(bits = "8..=15")]
    pub buttons_shared: ProconButtonsShared,
    #[packed_field(bits = "16..=23")]
    pub buttons_left: ProconButtonsLeft,
    #[packed_field(bits = "24..=39")]
    pub lstick_x: u16,
    #[packed_field(bits = "40..=47")]
    pub lstick_y: u8,
    #[packed_field(bits = "48..=63")]
    pub rstick_x: u16,
    #[packed_field(bits = "64..=71")]
    pub rstick_y: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", endian = "msb", size_bytes = "1")]
struct BatteryStatus {
    #[packed_field(bits = "0..=3")]
    connection: u8,
    #[packed_field(bits = "4..=7")]
    battery_level: u8,
}

impl Default for BatteryStatus {
    fn default() -> Self {
        Self {
            connection: 1,
            battery_level: 8,
        }
    }
}

impl From<&ControllerState> for ProconState {
    fn from(value: &ControllerState) -> Self {
        Self {
            buttons_left: ProconButtonsLeft {
                dpad_down: value.dpad_down,
                dpad_right: value.dpad_right,
                dpad_up: value.dpad_up,
                dped_left: value.dpad_left,
                trigger_l: value.trigger_zl,
                trigger_zl: value.trigger_l,
                ..Default::default()
            },
            buttons_right: ProconButtonsRight {
                button_a: value.button_a,
                button_b: value.button_b,
                button_x: value.button_x,
                button_y: value.button_y,
                trigger_r: value.trigger_zr,
                trigger_zr: value.trigger_r,
                ..Default::default()
            },
            buttons_shared: ProconButtonsShared {
                button_plus: value.button_start && !value.trigger_zr,
                button_home: value.button_start && value.trigger_zr,
                ..Default::default()
            },
            lstick_x: value.stick_state.ax as u16 * 16,
            lstick_y: value.stick_state.ay,
            rstick_x: value.stick_state.cx as u16 * 16,
            rstick_y: value.stick_state.cy,
        }
    }
}

pub struct ProconReportBuilder {
    switch_reporting_mode: u8,
    switch_mac_address: [u8; 6],
    switch_host_address: [u8; 6],
    switch_ltk: [u8; 16],
}

impl Default for ProconReportBuilder {
    fn default() -> Self {
        Self {
            switch_reporting_mode: 0,
            switch_mac_address: gen_switch_mac_address(),
            switch_host_address: [0u8; 6],
            switch_ltk: gen_switch_ltk(),
        }
    }
}

fn gen_switch_mac_address() -> [u8; 6] {
    let mut mac_addr = [0u8; 6];

    mac_addr.iter_mut().for_each(|e| {
        *e = RoscRng.next_u64() as u8;
    });

    mac_addr[0] &= 0xfe;
    mac_addr[5] = 0x9b;

    mac_addr
}

fn gen_switch_ltk() -> [u8; 16] {
    let mut switch_ltk = [0u8; 16];

    switch_ltk.iter_mut().for_each(|e| {
        *e = RoscRng.next_u64() as u8;
    });

    switch_ltk
}

impl ProconReportBuilder {
    fn get_info_report(&self, current_report_info: &ProconRequestInfo) -> [u8; 64] {
        let mut report = ProconByteReport([0u8; 64]);

        report.set_report_id(ProconResponseId::GetInfo as u8);

        if current_report_info.command_id == SW_INFO_SET_MAC {
            report[1] = SW_INFO_SET_MAC;
            report[3] = 0x03;

            self.switch_mac_address
                .iter()
                .rev()
                .enumerate()
                .for_each(|(i, e)| {
                    report[4 + i] = *e;
                });
        } else {
            report[1] = current_report_info.command_id;
        }

        *report
    }

    fn get_state_report(&self, state: &ProconState) -> [u8; 64] {
        static mut UNKNOWN: u8 = 0xA;

        unsafe {
            UNKNOWN = match UNKNOWN {
                0xA => 0xB,
                0xB => 0xC,
                _ => 0xA,
            };
        }

        let mut report = ProconByteReport([0u8; 64]);

        let data = state
            .pack()
            .expect("Failed to pack pro controller input data");

        report.set_report_id(ProconResponseId::GetState as u8);
        report.set_timer();
        report.set_battery_status();

        report[3..=11].copy_from_slice(&data);
        report[12] = unsafe { UNKNOWN };

        *report
    }

    fn get_command_report(&mut self, current_report_info: &ProconRequestInfo) -> [u8; 64] {
        let mut report = ProconByteReport([0u8; 64]);

        report.set_report_id(ProconResponseId::Command as u8);
        report.set_timer();
        report.set_battery_status();
        report.set_subcommand(current_report_info.command_id);

        match current_report_info.command_id {
            SW_CMD_SET_INPUT_MODE => {
                report.set_ack(ACK_GENERIC);
                self.switch_reporting_mode = current_report_info.raw_data[11];
                info!(
                    "Switch reporting mode is now {:x}",
                    self.switch_reporting_mode
                );
            }
            SW_CMD_GET_DEVINFO => {
                report.set_ack(ACK_GET_DEVINFO);
                report.set_devinfo();
            }
            SW_CMD_GET_SPI => {
                report.set_ack(ACK_GET_SPI);
                report.sw_spi_readfromaddress(
                    current_report_info.raw_data[12],
                    current_report_info.raw_data[11],
                    current_report_info.raw_data[15],
                    &self.switch_host_address,
                );
            }
            SW_CMD_SET_SHIPMODE => {
                report.set_ack(ACK_GENERIC);
            }
            SW_CMD_SET_PAIRING => {
                report.set_ack(ACK_SET_PAIRING);
                self.perform_pairing(&mut report, current_report_info);
            }
            SW_CMD_GET_TRIGGERET => {
                report.set_ack(ACK_GET_TRIGERET);
                report.set_trigerret(100);
            }
            _ => {
                report.set_ack(ACK_GENERIC);
            }
        }

        *report
    }

    fn perform_pairing(
        &mut self,
        report: &mut ProconByteReport,
        current_report_info: &ProconRequestInfo,
    ) {
        let pairing_phase = current_report_info.raw_data[11];
        let host_address = &current_report_info.raw_data[12..];

        match pairing_phase {
            1 => {
                self.switch_host_address
                    .iter_mut()
                    .enumerate()
                    .for_each(|(i, e)| *e = host_address[5 - i]);

                report[16..=21].copy_from_slice(&self.switch_mac_address);

                report[22..(22 + PRO_CONTROLLER_STRING.len())]
                    .copy_from_slice(&PRO_CONTROLLER_STRING);
            }
            2 => {
                report[15] = 2;
                report[16..(16 + self.switch_ltk.len())].copy_from_slice(&self.switch_ltk);
            }
            3 => {
                report[15] = 3;
            }
            _ => {}
        }
    }
}

impl HidReportBuilder<64> for ProconReportBuilder {
    async fn get_hid_report(&mut self, state: &ControllerState) -> [u8; 64] {
        let current_report_info = if self.switch_reporting_mode == RM_SEND_STATE {
            SIGNAL_PROCON_REQUEST.try_take()
        } else {
            Some(SIGNAL_PROCON_REQUEST.wait().await)
        };

        if let Some(current_report_info) = current_report_info {
            match current_report_info.report_id {
                ProconRequestId::GetInfo => self.get_info_report(&current_report_info),
                ProconRequestId::Command => self.get_command_report(&current_report_info),
                ProconRequestId::Rumble => self.get_state_report(&ProconState::from(state)),
            }
        } else {
            self.get_state_report(&ProconState::from(state))
        }
    }
}

pub struct ProconRequestHandler;

impl RequestHandler for ProconRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        trace!("Set report for {:?}: {:x}", id, data);

        let mut buf = [0u8; 64];
        let len_to_copy = buf.len().min(data.len());
        buf[..len_to_copy].copy_from_slice(&data[..len_to_copy]);

        if let ReportId::Out(id) = id {
            if id == ProconRequestId::GetInfo as u8 {
                SIGNAL_PROCON_REQUEST.signal(ProconRequestInfo {
                    command_id: buf[1],
                    report_id: ProconRequestId::GetInfo,
                    response_id: ProconResponseId::GetInfo,
                    raw_data: buf,
                });
            } else if id == ProconRequestId::Command as u8 {
                SIGNAL_PROCON_REQUEST.signal(ProconRequestInfo {
                    command_id: buf[10],
                    report_id: ProconRequestId::Command,
                    response_id: ProconResponseId::Command,
                    raw_data: buf,
                });
            } else if id == ProconRequestId::Rumble as u8 {
                // TODO: handle rumble
            }
        }

        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}
