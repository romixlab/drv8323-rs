use embedded_hal as hal;
use hal::blocking::spi;
use hal::digital::v2::{InputPin, OutputPin};
use crate::registers;
use crate::faults::{DrvFault};
use embedded_hal::blocking::delay::DelayUs;
use crate::registers::DrvRegister;

/// DRV8323 driver
pub struct DRV8323<SPI, CS, EN, CAL, FAULT, DELAY> {
    spi: SPI,
    chip_select_pin: CS,
    enable_pin: EN,
    calibration_pin: CAL,
    nfault_pin: FAULT,
    delay: DELAY,
}

#[derive(Debug)]
pub enum DrvError {
    SpiErr,
    SpiIsBroken(u16),
    PinErr,
    DrvFault,
}
pub type DrvResult<T = ()> = Result<T, DrvError>;

impl<SPI, CS, EN, CAL, FAULT, SpiErr, PinErr, DELAY> DRV8323<SPI, CS, EN, CAL, FAULT, DELAY>
where 
    SPI: spi::Transfer<u8, Error = SpiErr> + spi::Write<u8, Error = SpiErr>,
    CS: OutputPin<Error = PinErr>,
    EN: OutputPin<Error = PinErr>,
    CAL: OutputPin<Error = PinErr>,
    FAULT: InputPin<Error = PinErr>,
    DELAY: DelayUs<u32>,
{
    /// Instantiates a new drv8323 from an SPI peripheral and four GPIO pins.
    pub fn new(spi: SPI, mut cs: CS, mut enable: EN, mut cal: CAL, nfault: FAULT, mut delay: DELAY) -> DrvResult<Self> {
        cs.set_high().ok();
        cal.set_low().ok();


        enable.set_high().ok();
        delay.delay_us(100_u32);
        enable.set_low().ok();
        delay.delay_us(100_u32);
        enable.set_high().ok();
        delay.delay_us(1500_u32); // tready * 1.5 after enable

        let mut drv = DRV8323 {
            spi,
            chip_select_pin: cs,
            enable_pin: enable,
            calibration_pin: cal,
            nfault_pin: nfault,
            delay,
        };
        for _ in 0..10 {
            let ocp_control = Self::read_register(&mut drv, DrvRegister::OcpControl)?;
        }
        let ocp_control = Self::read_register(&mut drv, DrvRegister::OcpControl)?;
        if ocp_control != 0b0_01_01_01_1001 {
            return Err(DrvError::SpiIsBroken(ocp_control));
        }
        Ok(drv)
    }

    pub fn enable(&mut self) {
        self.enable_pin.set_high().ok();
    }
    
    pub fn disable(&mut self) {
        self.enable_pin.set_low().ok();
    }

    fn read_register(&mut self, reg: DrvRegister) -> DrvResult<u16> {
        self.chip_select_pin.set_low().ok();
        let rw_flag = 0b1;
        let msb = (rw_flag << 7) | (reg.addr() << 3);
        let mut transfer_buffer: [u8; 2];
        transfer_buffer = [msb, 0];

        self.spi.transfer(&mut transfer_buffer).map_err(|_| DrvError::SpiErr)?;

        let upper_byte = transfer_buffer[1] as u16;
        let lower_byte = transfer_buffer[0] as u16;
        let received: u16 = (upper_byte << 7) | lower_byte;

        self.chip_select_pin.set_high().ok();
        self.delay.delay_us(1000u32);

        return Ok(received);
    }

    pub fn check_faults(&mut self) -> DrvResult {
        // let fs1 = registers::read_register(&mut self.spi, registers::DrvRegister::FaultStatus1);
        // let fs2 = registers::read_register(&mut self.spi, registers::DrvRegister::FaultStatus2);

        let error: DrvFault;
        Ok(())
    }
}