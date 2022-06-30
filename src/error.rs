use embedded_hal::{blocking::spi, digital::v2::OutputPin};

/// An error that can occur when sending or receiving data
pub enum Error<SPI, CS>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    CS: OutputPin,
{
    //Error occured while using the DW3000
    Dw3000Error(dw3000::Error<SPI, CS>),

    // Error from blocking_timer
    TimeoutError(embedded_timeout_macros::TimeoutError<dw3000::Error<SPI, CS>>),

    ModuleInconnu,

    ModuleAnchor,
    ModuleTag,
}

impl<SPI, CS> From<dw3000::Error<SPI, CS>> for Error<SPI, CS>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    CS: OutputPin,
{
    fn from(error: dw3000::Error<SPI, CS>) -> Self {
        Error::Dw3000Error(error)
    }
}

impl<SPI, CS> From<embedded_timeout_macros::TimeoutError<dw3000::Error<SPI, CS>>> for Error<SPI, CS>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    CS: OutputPin,
{
    fn from(error: embedded_timeout_macros::TimeoutError<dw3000::Error<SPI, CS>>) -> Self {
        Error::TimeoutError(error)
    }
}
