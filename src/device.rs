use embedded_hal as hal;
use hal::blocking::spi;
use hal::digital::v2::{InputPin, OutputPin};
use crate::registers;
use crate::faults::{DrvFault};
use embedded_hal::blocking::delay::DelayUs;
use crate::registers::{DrvRegister, PwmMode, DriveControl};
use crate::registers::DrvRegister::DriverControl;

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
    Verify,
    PinErr,
    DrvFault,
}
pub type DrvResult<T = ()> = Result<T, DrvError>;

impl<SPI, CS, EN, CAL, FAULT, SpiErr, PinErr, DELAY> DRV8323<SPI, CS, EN, CAL, FAULT, DELAY>
where 
    SPI: spi::Transfer<u16, Error = SpiErr> + spi::Write<u16, Error = SpiErr>,
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

    pub fn set_pwm_mode(&mut self, mode: PwmMode) -> DrvResult<()> {
        self.modify_register(
            DrvRegister::DriverControl,
            (mode as u16) << 5,
            DriveControl::PWM_MODE.bitmask()
        )
    }

    pub fn read_register(&mut self, reg: DrvRegister) -> DrvResult<u16> {
        self.chip_select_pin.set_low().ok();
        let read_flag = 1u16;
        let read_command = (read_flag << 15) | (reg.addr() << 11);
        let mut transfer_buffer = [read_command];

        self.spi.transfer(&mut transfer_buffer).map_err(|_| DrvError::SpiErr)?;

        self.chip_select_pin.set_high().ok();
        self.delay.delay_us(1000u32);

        return Ok(transfer_buffer[0] & 0b111_1111_1111);
    }

    fn write_register(&mut self, reg: DrvRegister, value: u16) -> DrvResult<()> {
        let value = value & 0b111_1111_1111;

        self.chip_select_pin.set_low().ok();
        let write_flag = 0u16;
        let write_command = (write_flag << 15) | (reg.addr() << 11) | value;
        let mut transfer_buffer = [write_command];

        self.spi.transfer(&mut transfer_buffer).map_err(|_| DrvError::SpiErr)?;

        self.chip_select_pin.set_high().ok();
        self.delay.delay_us(1000u32);

        let verify = self.read_register(reg)?;
        if verify != value {
            return Err(DrvError::Verify);
        }

        return Ok(());
    }

    fn modify_register(&mut self, reg: DrvRegister, value: u16, mask: u16) -> DrvResult {
        let value = value & 0b111_1111_1111 & mask;
        let current = self.read_register(reg)?;
        let modified = (current & !mask) | value;
        self.write_register(reg, modified)?;
        Ok(())
    }

    pub fn check_faults(&mut self) -> DrvResult {
        // let fs1 = registers::read_register(&mut self.spi, registers::DrvRegister::FaultStatus1);
        // let fs2 = registers::read_register(&mut self.spi, registers::DrvRegister::FaultStatus2);

        let error: DrvFault;
        Ok(())
    }
}