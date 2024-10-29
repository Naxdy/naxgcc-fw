///
/// # XInput Protocol Implementation
///
/// The implementations for `XInputReader` and `XInputWriter` and the logic surrounding them is
/// mostly taken from embassy.
///
/// Unfortunately, the embassy hid classes don't allow us to specify a custom interface protocol,
/// hence the little bit of code duplication.
///
use core::{
    mem::MaybeUninit,
    sync::atomic::{AtomicUsize, Ordering},
};

use defmt::{info, trace, warn, Format};
use embassy_usb::{
    class::hid::{Config, ReadError, ReportId, RequestHandler},
    control::{InResponse, OutResponse, Recipient, Request, RequestType},
    driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut},
    types::InterfaceNumber,
    Builder, Handler,
};
use packed_struct::{derive::PackedStruct, PackedStruct};

use crate::usb_comms::HidReportBuilder;

use super::{gcc::GcState, HidReaderWriterSplit, UsbReader, UsbWriter};

/// lol
pub const XINPUT_REPORT_DESCRIPTOR: &[u8] = &[];

const HID_DESC_DESCTYPE_HID: u8 = 0x21;
const HID_DESC_DESCTYPE_HID_REPORT: u8 = 0x22;

const HID_REQ_SET_IDLE: u8 = 0x0a;
const HID_REQ_GET_IDLE: u8 = 0x02;
const HID_REQ_GET_REPORT: u8 = 0x01;
const HID_REQ_SET_REPORT: u8 = 0x09;
const HID_REQ_GET_PROTOCOL: u8 = 0x03;
const HID_REQ_SET_PROTOCOL: u8 = 0x0b;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct XInputButtons1 {
    #[packed_field(bits = "0")]
    pub dpad_up: bool,
    #[packed_field(bits = "1")]
    pub dpad_down: bool,
    #[packed_field(bits = "2")]
    pub dpad_left: bool,
    #[packed_field(bits = "3")]
    pub dpad_right: bool,
    #[packed_field(bits = "4")]
    pub button_menu: bool,
    #[packed_field(bits = "5")]
    pub button_back: bool,
    #[packed_field(bits = "6")]
    pub button_stick_l: bool,
    #[packed_field(bits = "7")]
    pub button_stick_r: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "lsb0", size_bytes = "1")]
pub struct XInputButtons2 {
    #[packed_field(bits = "0")]
    pub bumper_l: bool,
    #[packed_field(bits = "1")]
    pub bumper_r: bool,
    #[packed_field(bits = "2")]
    pub button_guide: bool,
    #[packed_field(bits = "3")]
    pub blank_1: bool,
    #[packed_field(bits = "4")]
    pub button_a: bool,
    #[packed_field(bits = "5")]
    pub button_b: bool,
    #[packed_field(bits = "6")]
    pub button_x: bool,
    #[packed_field(bits = "7")]
    pub button_y: bool,
}

///
/// HID report that is sent back to the host.
///
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, PackedStruct, Format)]
#[packed_struct(bit_numbering = "msb0", endian = "lsb", size_bytes = "32")]
pub struct XInputReport {
    #[packed_field(bits = "0..=7")]
    pub report_id: u8,
    #[packed_field(bits = "8..=15")]
    pub report_size: u8,
    #[packed_field(bits = "16..=23")]
    pub buttons_1: XInputButtons1,
    #[packed_field(bits = "24..=31")]
    pub buttons_2: XInputButtons2,
    #[packed_field(bits = "32..=39")]
    pub analog_trigger_l: u8,
    #[packed_field(bits = "40..=47")]
    pub analog_trigger_r: u8,
    #[packed_field(bits = "48..=63")]
    pub stick_left_x: i16,
    #[packed_field(bits = "64..=79")]
    pub stick_left_y: i16,
    #[packed_field(bits = "80..=95")]
    pub stick_right_x: i16,
    #[packed_field(bits = "96..=111")]
    pub stick_right_y: i16,
    #[packed_field(bits = "112..=255")]
    pub reserved: [u8; 18],
}

impl From<&GcState> for XInputReport {
    fn from(value: &GcState) -> Self {
        Self {
            report_id: 0,
            report_size: 20,
            buttons_1: XInputButtons1 {
                dpad_up: value.buttons_1.dpad_up,
                dpad_down: value.buttons_1.dpad_down,
                dpad_right: value.buttons_1.dpad_right,
                dpad_left: value.buttons_1.dpad_left,
                button_menu: value.buttons_2.button_start,
                button_back: false,
                button_stick_l: false,
                button_stick_r: false,
            },
            buttons_2: XInputButtons2 {
                blank_1: false,
                bumper_l: false,
                bumper_r: value.buttons_2.button_z,
                button_a: value.buttons_1.button_a,
                button_b: value.buttons_1.button_b,
                button_x: value.buttons_1.button_x,
                button_y: value.buttons_1.button_y,
                button_guide: false,
            },
            analog_trigger_l: value.trigger_l,
            analog_trigger_r: value.trigger_r,
            stick_left_x: (value.stick_x as i16 - 127).clamp(-127, 127) * 257,
            stick_left_y: (value.stick_y as i16 - 127).clamp(-127, 127) * 257,
            stick_right_x: (value.cstick_x as i16 - 127).clamp(-127, 127) * 257,
            stick_right_y: (value.cstick_y as i16 - 127).clamp(-127, 127) * 257,
            reserved: [0u8; 18],
        }
    }
}

///
/// Takes in a GcState, converts it to an `XInputReport` and returns its packed version.
///
pub struct XInputReportBuilder;

impl HidReportBuilder<32> for XInputReportBuilder {
    async fn get_hid_report(&mut self, state: &super::gcc::GcState) -> [u8; 32] {
        XInputReport::from(state)
            .pack()
            .expect("Failed to pack XInput State")
    }
}

///
/// Handles packets sent from the host.
///
pub struct XInputRequestHandler;

impl RequestHandler for XInputRequestHandler {
    fn get_report(
        &mut self,
        id: embassy_usb::class::hid::ReportId,
        buf: &mut [u8],
    ) -> Option<usize> {
        let _ = (id, buf);
        None
    }

    fn set_report(
        &mut self,
        id: embassy_usb::class::hid::ReportId,
        data: &[u8],
    ) -> embassy_usb::control::OutResponse {
        let _ = (id, data);
        info!("Set report for {:?}: {:x}", id, data);
        embassy_usb::control::OutResponse::Accepted
    }

    fn get_idle_ms(&mut self, id: Option<embassy_usb::class::hid::ReportId>) -> Option<u32> {
        let _ = id;
        None
    }

    fn set_idle_ms(&mut self, id: Option<embassy_usb::class::hid::ReportId>, duration_ms: u32) {
        let _ = (id, duration_ms);
    }
}

/// Taken from embassy.
pub struct XInputWriter<'d, D: Driver<'d>, const N: usize> {
    ep_in: D::EndpointIn,
}

impl<'d, D: Driver<'d>, const N: usize> UsbWriter<'d, D, N> for XInputWriter<'d, D, N> {
    /// Waits for the interrupt in endpoint to be enabled.
    async fn ready(&mut self) {
        self.ep_in.wait_enabled().await;
    }

    /// Writes `report` to its interrupt endpoint.
    async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        assert!(report.len() <= N);

        let max_packet_size = usize::from(self.ep_in.info().max_packet_size);
        let zlp_needed = report.len() < N && (report.len() % max_packet_size == 0);
        for chunk in report.chunks(max_packet_size) {
            self.ep_in.write(chunk).await?;
        }

        if zlp_needed {
            self.ep_in.write(&[]).await?;
        }

        Ok(())
    }
}

/// Taken from embassy.
pub struct XInputReader<'d, D: Driver<'d>, const N: usize> {
    ep_out: D::EndpointOut,
    offset: &'d AtomicUsize,
}

impl<'d, D: Driver<'d>, const N: usize> UsbReader<'d, D, N> for XInputReader<'d, D, N> {
    async fn run<T: RequestHandler>(mut self, use_report_ids: bool, handler: &mut T) -> ! {
        let offset = self.offset.load(Ordering::Acquire);
        assert!(offset == 0);
        let mut buf = [0; N];
        loop {
            match self.read(&mut buf).await {
                Ok(len) => {
                    let id = if use_report_ids { buf[0] } else { 0 };
                    handler.set_report(ReportId::Out(id), &buf[..len]);
                }
                Err(ReadError::BufferOverflow) => warn!(
                    "Host ent output report larger than the configured maximum output report length ({})",
                    N
                ),
                Err(ReadError::Disabled) => self.ep_out.wait_enabled().await,
                Err(ReadError::Sync(_)) => unreachable!(),
            }
        }
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError> {
        assert!(N != 0);
        assert!(buf.len() >= N);

        // Read packets from the endpoint
        let max_packet_size = usize::from(self.ep_out.info().max_packet_size);
        let starting_offset = self.offset.load(Ordering::Acquire);
        let mut total = starting_offset;
        loop {
            for chunk in buf[starting_offset..N].chunks_mut(max_packet_size) {
                match self.ep_out.read(chunk).await {
                    Ok(size) => {
                        total += size;
                        if size < max_packet_size || total == N {
                            self.offset.store(0, Ordering::Release);
                            break;
                        }
                        self.offset.store(total, Ordering::Release);
                    }
                    Err(err) => {
                        self.offset.store(0, Ordering::Release);
                        return Err(err.into());
                    }
                }
            }

            // Some hosts may send ZLPs even when not required by the HID spec, so we'll loop as long as total == 0.
            if total > 0 {
                break;
            }
        }

        if starting_offset > 0 {
            Err(ReadError::Sync(starting_offset..total))
        } else {
            Ok(total)
        }
    }
}

/// Taken from embassy, with a few modifications to the descriptor.
pub struct XInputReaderWriter<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> {
    reader: XInputReader<'d, D, READ_N>,
    writer: XInputWriter<'d, D, WRITE_N>,
}

impl<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize>
    XInputReaderWriter<'d, D, READ_N, WRITE_N>
{
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut XInputState<'d>,
        config: Config<'d>,
    ) -> Self {
        let mut func = builder.function(0xff, 0x5d, 0x01);
        let mut iface = func.interface();
        let if_num = iface.interface_number();
        let mut alt = iface.alt_setting(0xff, 0x5d, 0x01, None);

        #[rustfmt::skip]
        alt.descriptor(0x21, &[
            0x10, 0x01, // bcdHID 1.10
            0x01,       // bCountryCode
            0x24,       // bNumDescriptors
            0x81,       // bDescriptorType[0] (Unknown 0x81)
            0x14, 0x03, // wDescriptorLength[0] 788
            0x00,       // bDescriptorType[1] (Unknown 0x00)
            0x03, 0x13, // wDescriptorLength[1] 4867
            0x02,       // bDescriptorType[2] (Unknown 0x02)
            0x00, 0x03, // wDescriptorLength[2] 768
            0x00,       // bDescriptorType[3] (Unknown 0x00)
        ]);

        let ep_in = alt.endpoint_interrupt_in(config.max_packet_size_in, config.poll_ms);
        let ep_out = alt.endpoint_interrupt_out(config.max_packet_size_out, config.poll_ms);

        drop(func);

        let control = state.control.write(XInputControl::new(
            if_num,
            config.report_descriptor,
            config.request_handler,
            &state.out_report_offset,
        ));
        builder.handler(control);

        Self {
            reader: XInputReader {
                ep_out,
                offset: &state.out_report_offset,
            },
            writer: XInputWriter { ep_in },
        }
    }
}

impl<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize>
    HidReaderWriterSplit<'d, D, READ_N, WRITE_N> for XInputReaderWriter<'d, D, READ_N, WRITE_N>
{
    fn split(
        self,
    ) -> (
        impl UsbReader<'d, D, READ_N>,
        impl UsbWriter<'d, D, WRITE_N>,
    ) {
        (self.reader, self.writer)
    }
}

pub struct XInputState<'d> {
    control: MaybeUninit<XInputControl<'d>>,
    out_report_offset: AtomicUsize,
}

impl<'d> Default for XInputState<'d> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> XInputState<'d> {
    pub const fn new() -> Self {
        XInputState {
            control: MaybeUninit::uninit(),
            out_report_offset: AtomicUsize::new(0),
        }
    }
}

/// Taken from embassy.
struct XInputControl<'d> {
    if_num: InterfaceNumber,
    report_descriptor: &'d [u8],
    request_handler: Option<&'d mut dyn RequestHandler>,
    out_report_offset: &'d AtomicUsize,
    hid_descriptor: [u8; 16],
}

impl<'d> XInputControl<'d> {
    fn new(
        if_num: InterfaceNumber,
        report_descriptor: &'d [u8],
        request_handler: Option<&'d mut dyn RequestHandler>,
        out_report_offset: &'d AtomicUsize,
    ) -> Self {
        XInputControl {
            if_num,
            report_descriptor,
            request_handler,
            out_report_offset,
            #[rustfmt::skip]
            hid_descriptor: [
                0x10,       // bLength
                0x21,       // bDescriptorType (HID)
                0x10, 0x01, // bcdHID 1.10
                0x01,       // bCountryCode
                0x24,       // bNumDescriptors
                0x81,       // bDescriptorType[0] (Unknown 0x81)
                0x14, 0x03, // wDescriptorLength[0] 788
                0x00,       // bDescriptorType[1] (Unknown 0x00)
                0x03, 0x13, // wDescriptorLength[1] 4867
                0x02,       // bDescriptorType[2] (Unknown 0x02)
                0x00, 0x03, // wDescriptorLength[2] 768
                0x00,       // bDescriptorType[3] (Unknown 0x00)
            ],
        }
    }
}

/// Helper function, since the function in `ReportId` is private.
const fn try_u16_to_report_id(value: u16) -> Result<ReportId, ()> {
    match value >> 8 {
        1 => Ok(ReportId::In(value as u8)),
        2 => Ok(ReportId::Out(value as u8)),
        3 => Ok(ReportId::Feature(value as u8)),
        _ => Err(()),
    }
}

impl<'d> Handler for XInputControl<'d> {
    fn reset(&mut self) {
        self.out_report_offset.store(0, Ordering::Release);
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Class,
                Recipient::Interface,
                self.if_num.0 as u16,
            )
        {
            return None;
        }

        trace!("HID control_out {:?} {=[u8]:x}", req, data);

        match req.request {
            HID_REQ_SET_IDLE => {
                if let Some(handler) = self.request_handler.as_mut() {
                    let id = req.value as u8;
                    let id = (id != 0).then_some(ReportId::In(id));
                    let dur = u32::from(req.value >> 8);
                    let dur = if dur == 0 { u32::MAX } else { 4 * dur };
                    handler.set_idle_ms(id, dur);
                }
                Some(OutResponse::Accepted)
            }
            HID_REQ_SET_REPORT => {
                match (
                    try_u16_to_report_id(req.value),
                    self.request_handler.as_mut(),
                ) {
                    (Ok(id), Some(handler)) => Some(handler.set_report(id, data)),
                    _ => Some(OutResponse::Rejected),
                }
            }
            HID_REQ_SET_PROTOCOL => {
                if req.value == 1 {
                    Some(OutResponse::Accepted)
                } else {
                    warn!("HID Boot Protocol is unsupported.");
                    Some(OutResponse::Rejected) // UNSUPPORTED: Boot Protocol
                }
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        match (req.request_type, req.recipient) {
            (RequestType::Standard, Recipient::Interface) => match req.request {
                Request::GET_DESCRIPTOR => match (req.value >> 8) as u8 {
                    HID_DESC_DESCTYPE_HID_REPORT => {
                        Some(InResponse::Accepted(self.report_descriptor))
                    }
                    HID_DESC_DESCTYPE_HID => Some(InResponse::Accepted(&self.hid_descriptor)),
                    _ => Some(InResponse::Rejected),
                },

                _ => Some(InResponse::Rejected),
            },
            (RequestType::Class, Recipient::Interface) => {
                trace!("HID control_in {:?}", req);
                match req.request {
                    HID_REQ_GET_REPORT => {
                        let size = match try_u16_to_report_id(req.value) {
                            Ok(id) => self
                                .request_handler
                                .as_mut()
                                .and_then(|x| x.get_report(id, buf)),
                            Err(_) => None,
                        };

                        if let Some(size) = size {
                            Some(InResponse::Accepted(&buf[0..size]))
                        } else {
                            Some(InResponse::Rejected)
                        }
                    }
                    HID_REQ_GET_IDLE => {
                        if let Some(handler) = self.request_handler.as_mut() {
                            let id = req.value as u8;
                            let id = (id != 0).then_some(ReportId::In(id));
                            if let Some(dur) = handler.get_idle_ms(id) {
                                let dur = u8::try_from(dur / 4).unwrap_or(0);
                                buf[0] = dur;
                                Some(InResponse::Accepted(&buf[0..1]))
                            } else {
                                Some(InResponse::Rejected)
                            }
                        } else {
                            Some(InResponse::Rejected)
                        }
                    }
                    HID_REQ_GET_PROTOCOL => {
                        // UNSUPPORTED: Boot Protocol
                        buf[0] = 1;
                        Some(InResponse::Accepted(&buf[0..1]))
                    }
                    _ => Some(InResponse::Rejected),
                }
            }
            _ => None,
        }
    }
}
