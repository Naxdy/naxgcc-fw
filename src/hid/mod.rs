use embassy_usb::{
    class::hid::{HidReader, HidReaderWriter, HidWriter, ReadError, RequestHandler},
    driver::{Driver, EndpointError},
};

pub mod gcc;
pub mod procon;
pub mod xinput;

/// Custom trait to unify the API between embassy's HID writer, and our XInput reader/writer (and any
/// custom writers we may create in the future)
pub trait HidReaderWriterSplit<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> {
    fn split(
        self,
    ) -> (
        impl UsbReader<'d, D, READ_N>,
        impl UsbWriter<'d, D, WRITE_N>,
    );
}

/// Custom trait to unify the API between embassy's HID writer, and our XInput reader (and any
/// custom writers we may create in the future)
pub trait UsbReader<'d, D: Driver<'d>, const READ_N: usize> {
    async fn run<T: RequestHandler>(self, use_report_ids: bool, handler: &mut T) -> !;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError>;
}

/// Custom trait to unify the API between embassy's HID writer, and our XInput writer (and any
/// custom writers we may create in the future)
pub trait UsbWriter<'d, D: Driver<'d>, const WRITE_N: usize> {
    async fn ready(&mut self);

    async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError>;
}

impl<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize>
    HidReaderWriterSplit<'d, D, READ_N, WRITE_N> for HidReaderWriter<'d, D, READ_N, WRITE_N>
{
    fn split(
        self,
    ) -> (
        impl UsbReader<'d, D, READ_N>,
        impl UsbWriter<'d, D, WRITE_N>,
    ) {
        self.split()
    }
}

impl<'d, D: Driver<'d>, const READ_N: usize> UsbReader<'d, D, READ_N> for HidReader<'d, D, READ_N> {
    async fn run<T: RequestHandler>(self, use_report_ids: bool, handler: &mut T) -> ! {
        self.run(use_report_ids, handler).await
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError> {
        self.read(buf).await
    }
}

impl<'d, D: Driver<'d>, const WRITE_N: usize> UsbWriter<'d, D, WRITE_N>
    for HidWriter<'d, D, WRITE_N>
{
    async fn ready(&mut self) {
        self.ready().await
    }

    async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        self.write(report).await
    }
}
