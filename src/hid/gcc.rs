use defmt::{info, trace, Format};
use embassy_usb::{
    class::hid::{ReportId, RequestHandler},
    control::OutResponse,
};

use crate::{
    input::ControllerState,
    usb_comms::{HidReportBuilder, SIGNAL_RUMBLE},
};
use packed_struct::{derive::PackedStruct, PackedStruct};

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
pub struct GcButtons1 {
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
pub struct GcButtons2 {
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

///
/// Struct representing the controller state. Used for HID communications in
/// GCC adapter mode, as well as for the internal representation of the controller.
///
#[derive(Clone, Copy, Debug, PartialEq, Eq, PackedStruct, Format)]
#[packed_struct(bit_numbering = "msb0", size_bytes = "8")]
pub struct GcState {
    #[packed_field(bits = "0..=7")]
    pub buttons_1: GcButtons1,
    #[packed_field(bits = "8..=15")]
    pub buttons_2: GcButtons2,
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

impl From<ControllerState> for GcState {
    fn from(value: ControllerState) -> Self {
        Self {
            buttons_1: GcButtons1 {
                button_a: value.button_a,
                button_b: value.button_b,
                button_x: value.button_x,
                button_y: value.button_y,
                dpad_left: value.dpad_left,
                dpad_right: value.dpad_right,
                dpad_down: value.dpad_down,
                dpad_up: value.dpad_up,
            },
            buttons_2: GcButtons2 {
                button_start: value.button_start,
                button_z: value.trigger_zr,
                button_r: value.trigger_r,
                button_l: value.trigger_l,
                blank1: 0,
            },
            stick_x: value.stick_state.ax,
            stick_y: value.stick_state.ay,
            cstick_x: value.stick_state.cx,
            cstick_y: value.stick_state.cy,
            trigger_l: if value.trigger_l { 255 } else { 0 },
            trigger_r: if value.trigger_r { 255 } else { 0 },
        }
    }
}

impl Default for GcState {
    fn default() -> Self {
        Self {
            buttons_1: GcButtons1::default(),
            buttons_2: GcButtons2::default(),
            stick_x: 127,
            stick_y: 127,
            cstick_x: 127,
            cstick_y: 127,
            trigger_l: 0,
            trigger_r: 0,
        }
    }
}

#[derive(Default)]
pub struct GcReportBuilder {
    gc_first: bool,
}

impl HidReportBuilder<37> for GcReportBuilder {
    async fn get_hid_report(&mut self, state: &ControllerState) -> [u8; 37] {
        let mut buffer = [0u8; 37];

        buffer[0] = 0x21;
        buffer[1] |= 0x14;

        let gcc_state: GcState = (*state).into();

        let data = gcc_state.pack().expect("Failed to pack GC input data");

        if !self.gc_first {
            buffer[1] |= 0x04;
            buffer[10] |= 0x04;
            buffer[19] |= 0x04;
            buffer[28] |= 0x04;
            self.gc_first = true;
        } else {
            // controller in "port 1"
            buffer[2..=9].copy_from_slice(&data);
        }

        buffer
    }
}

pub struct GccRequestHandler;

impl RequestHandler for GccRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        trace!("Set report for {:?}: {:x}", id, data);

        if data.len() > 1 {
            SIGNAL_RUMBLE.signal((data[1] & 0x01) != 0);
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
