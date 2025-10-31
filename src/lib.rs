//! A generic driver for the MAX31865 RTD to Digital converter
//!
//! # References
//! - Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf

#![cfg_attr(not(test), no_std)]
extern crate alloc;

use alloc::vec::Vec;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::{Mode, Phase, Polarity, SpiBus};

#[cfg(feature = "doc")]
pub mod examples;

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub mod temp_conversion;

pub enum FilterMode {
    /// Set for 60 hz mains in your country.
    /// Temperature updated every 52 ms in continuous mode
    Filter60Hz = 0,
    /// Set for 50 hz mains in your country.
    /// Temperature updated every 62.5 ms in continuous mode
    Filter50Hz = 1,
}

pub enum SensorType {
    /// For two or 4 wire setups - including using a test resistor.
    /// For two wire setups, you need to configure jumpers to the outside pins.
    TwoOrFourWire = 0,
    /// For three wire sensors
    ThreeWire = 1,
}

pub struct Max31865<SPI, NCS, RDY> {
    spi: SPI,
    ncs: NCS,
    #[allow(unused)]
    rdy: RDY, // Only required for the unsupported one-shot mode.
    calibration: u32,
}

#[derive(Debug)]
pub enum Error {
    /// Error reading from the Max31856 chip registers
    SpiErrorRead,
    /// Erro writing from the Max31856 chip registers
    SpiErrorWrite,
    /// Error transferring data to/from Max31856 chip registers
    SpiErrorTransfer,
    /// Error setting the state of a pin in the GPIO bus
    PinError,
    /// Configuration parameters are invalid or unsupported.
    ConfigError,
    /// The Max31856 chip declared an error when converting temperatures.
    /// Use `read_fault_status()` for details.
    MAXError,
}

impl<SPI, NCS, RDY> Max31865<SPI, NCS, RDY>
where
    SPI: SpiBus<u8>,
    NCS: OutputPin,
    RDY: InputPin,
{
    /// Create a new MAX31865 module.
    ///
    /// # Arguments
    ///
    /// * `spi` - The SPI (Serial Peripheral Interface) module to communicate on.
    /// * `ncs` - The Chip Select pin (NCS: Negative Chip Select - active low)
    ///           which must be a push-pull output pin (`OutputPin`)
    ///           Use `into_output_high()` or a pullup resistor for reliable operation.
    /// * `rdy` - The Ready pin which is set low by the MAX31865 controller
    ///           whenever it has finished converting the output.
    ///           This pin is NOT USED in continuous sample mode (one_shot == 0).
    ///
    pub fn new(spi: SPI, mut ncs: NCS, rdy: RDY) -> Result<Max31865<SPI, NCS, RDY>, Error> {
        let default_calib = 40000;

        ncs.set_high().map_err(|_| Error::PinError)?;
        let max31865 = Max31865 {
            spi,
            ncs,
            rdy,
            calibration: default_calib, /* value in ohms multiplied by 100 */
        };

        Ok(max31865)
    }

    /// Updates the devices configuration.
    ///
    /// # Arguments
    /// * `vbias` - Set to `true` to enable V_BIAS voltage, which is required to
    ///             correctly perform conversion.Clone
    /// * `conversion_mode` - `true` to automatically perform conversion,
    ///                       otherwise normally off.
    /// * `one_shot` - Only perform detection once if set to `true`, otherwise
    ///             repeats conversion.
    ///             ONE-SHOT MODE NOT SUPPORTED BY THIS DRIVER!
    /// * `sensor_type` - Define whether a two, three or four wire sensor is
    ///                   used. Two and Four wire sensors are indistinguishable.
    /// * `filter_mode` - Specify the mains frequency that should be used to
    ///                   filter out noise, e.g. 50Hz in Europe.
    ///                   In continuous mode, temperatures are updated by the chip
    ///                   at a 16 hz rate for 50 hz filter mode, and 19 hz rate for 60 hz
    ///                   filter mode.
    ///
    /// # Remarks
    ///
    /// This will update the configuration register of the MAX31865 register. If
    /// the device doesn't properly react to this, add a delay after calling
    /// `new` to increase the time that the chip select line is set high.
    ///
    /// *Note*: The correct sensor configuration also requires changes to the
    /// PCB! Make sure to read the data sheet concerning this.
    pub fn configure(
        &mut self,
        vbias: bool,
        conversion_mode: bool,
        one_shot: bool,
        sensor_type: SensorType,
        filter_mode: FilterMode,
    ) -> Result<(), Error> {
        if one_shot { return Err(Error::ConfigError) };  // One_shot not yet supported.
        let conf: u8 = ((vbias as u8) << 7)
            | ((conversion_mode as u8) << 6)
            | ((one_shot as u8) << 5)
            | ((sensor_type as u8) << 4)
            | (filter_mode as u8);

        self.write(Register::CONFIG, conf)?;
        self.clear_fault()?; // Unlatch any boot faults (mimics Adafruit init)

        Ok(())
    }

    /// Clear latched faults (config reg bit 1 = 1)
    pub fn clear_fault(&mut self) -> Result<(), Error> {
        self.write(Register::CONFIG, 0x02)
    }

    /// Read and clear fault status reg (0x07) for bit-level  diagnostics (u8 LSB)
    pub fn read_fault_status(&mut self) -> Result<u8, Error> {
        let status = self.read(Register::FAULT_STATUS)?;
        self.clear_fault()?; // Clear after read (if auto-clear needed)
        Ok(status)
    }

    /// Set the calibration reference resistance. This can be used to calibrate
    /// inaccuracies of both the reference resistor and the PT100 element.
    ///
    /// # Arguments
    ///
    /// * `calib` - A 32-bit integer specifying the reference resistance in ohms
    ///             multiplied by 100, e.g. `40000` for 400 Ohms
    ///
    /// # Remarks
    ///
    /// You can perform calibration by putting the sensor in boiling (100
    /// degrees Celsius) water and then measuring the raw value using
    /// `read_raw`. Calculate `calib` as `(13851 << 15) / raw >> 1`.
    pub fn set_calibration(&mut self, calib: u32) {
        self.calibration = calib;
    }

    /// Read the raw resistance value.
    ///
    /// # Remarks
    ///
    /// The output value is the value in Ohms multiplied by 100.
    pub fn read_ohms(&mut self) -> Result<u32, Error> {
        let raw = self.read_raw()?;
        let ohms = ((raw >> 1) as u32 * self.calibration) >> 15;
        Ok(ohms)
    }

    /// Read resistance in ohms as f64
    pub fn read_resistance(&mut self) -> Result<f64, Error> {
        let ohms_raw = self.read_ohms()?; // u32 *100;
        Ok(ohms_raw as f64 / 100.0)
    }

    /// Read temperature in °C as f64
    pub fn read_temperature(&mut self) -> Result<f64, Error> {
        let temp_raw = self.read_default_conversion()?; // i32 *100
        Ok(temp_raw as f64 / 100.0)
    }

    /// Read the raw resistance value and then perform conversion to degrees Celsius.
    /// The output value is the value in degrees Celsius multiplied by 100.
    pub fn read_default_conversion(&mut self) -> Result<i32, Error> {
        let ohms = self.read_ohms()?;
        let temp = temp_conversion::LOOKUP_VEC_PT100.lookup_temperature(ohms as i32);
        Ok(temp)
    }

    /// Read the raw RTD value.
    ///
    /// # Remarks
    ///
    /// The raw value is the value of the combined MSB and LSB registers.
    /// The first 15 bits specify the ohmic value in relation to the reference
    /// resistor (i.e. 2^15 - 1 would be the exact same resistance as the reference
    /// resistor). See manual for further information.
    /// The last bit specifies if the conversion was successful.

    pub fn read_raw(&mut self) -> Result<u16, Error> {
        let buffer = self.read_two(Register::RTD_MSB)?; // Single read_two on MSB clocks MSB + LSB
        let raw = ((buffer[0] as u16) << 8) | (buffer[1] as u16); // buffer[0] = MSB, [1] = LSB
        if raw & 1 != 0 { // LSB bit 0 = 1 = fault during read
            return Err(Error::MAXError);
        }
        Ok(raw)
    }

    fn read(&mut self, reg: Register) -> Result<u8, Error> {
        let mut read_buffer = [0u8; 2]; // 2 bytes: dummy + data
        let mut write_buffer = [0u8; 2];
        write_buffer[0] = reg.read_address(); // Read addr for reg (e.g., 0x81 for 0x01)
        write_buffer[1] = 0; // Dummy data
        self.ncs.set_low().map_err(|_| Error::PinError)?;
        self.spi
            .transfer(&mut read_buffer, &write_buffer)
            .map_err(|_| Error::SpiErrorTransfer)?;
        self.ncs.set_high().map_err(|_| Error::PinError)?;
        Ok(read_buffer[1]) // Return result (ignore dummy [0])
    }

    fn read_two(&mut self, reg: Register) -> Result<[u8; 2], Error> {
        // The hardware is full duplex - you have to read and write the same number of bytes.
        // The first byte you write is the register offset, and the remaining
        // bytes are ignored when reading. To read two bytes you write three.
        // The two bytes we read are in the last two of the three bytes read.
        // NOTE: It reads and writes the minimum size of the read and write buffers
        let mut read_buffer = [0u8; 3]; // 3 bytes: dummy + MSB + LSB
        let mut write_buffer = [0u8; 3];
        write_buffer[0] = reg.read_address(); // Read addr for reg (e.g., 0x81 for 0x01)
        write_buffer[1] = 0; // Dummy for MSB
        write_buffer[2] = 0; // Dummy for LSB
        self.ncs.set_low().map_err(|_| Error::PinError)?;
        self.spi
            .transfer(&mut read_buffer, &write_buffer)
            .map_err(|_| Error::SpiErrorTransfer)?;
        self.ncs.set_high().map_err(|_| Error::PinError)?;
        Ok([read_buffer[1], read_buffer[2]]) // Return MSB, LSB (ignore dummy [0])
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), Error> {
        self.ncs.set_low().map_err(|_| Error::PinError)?;
        self.spi
            .write(&[reg.write_address(), val])
            .map_err(|_| Error::SpiErrorWrite)?;
        self.ncs.set_high().map_err(|_| Error::PinError)?;
        Ok(())
    }
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Register {    // All the lovely Max31856 register offsets
    CONFIG = 0x00,
    RTD_MSB = 0x01,
    RTD_LSB = 0x02,
    HIGH_FAULT_THRESHOLD_MSB = 0x03,
    HIGH_FAULT_THRESHOLD_LSB = 0x04,
    LOW_FAULT_THRESHOLD_MSB = 0x05,
    LOW_FAULT_THRESHOLD_LSB = 0x06,
    FAULT_STATUS = 0x07,
}

const R: u8 = 0 << 7;
const W: u8 = 1 << 7;

impl Register {
    fn read_address(&self) -> u8 {
        *self as u8 | R
    }

    fn write_address(&self) -> u8 {
        *self as u8 | W
    }
}

#[derive(Debug, Clone, Copy)]
pub enum FaultStatus {
    ShortToGnd(&'static str),  // Bit 0: RTDIN- short to GND
    OpenCircuit(&'static str),  // Bit 1: RTDIN+ open circuit
    OverUnderVoltage(&'static str),  // Bit 2: Over/under voltage
    OverTemperature(&'static str),  // Bit 3: Over temperature
    HighThreshold(&'static str),  // Bit 7: RTD high threshold (infinite R)
    // Add bits 4-6 if needed (reserved or future per datasheet 4.3)
}

/// Parse u8 fault status (reg 0x07) into Vec of set bits with descriptions (no_std safe).
pub fn parse_fault_status(status: u8) -> Vec<FaultStatus> {
    let mut faults = Vec::new();
    if status & 0x01 != 0 {
        faults.push(FaultStatus::ShortToGnd("RTDIN- short to GND (bit 0)"));
    }
    if status & 0x02 != 0 {
        faults.push(FaultStatus::OpenCircuit("RTDIN+ open circuit (bit 1)"));
    }
    if status & 0x04 != 0 {
        faults.push(FaultStatus::OverUnderVoltage("Over/under voltage (bit 2)"));
    }
    if status & 0x08 != 0 {
        faults.push(FaultStatus::OverTemperature("Over temperature (bit 3)"));
    }
    if status & 0x80 != 0 {
        faults.push(FaultStatus::HighThreshold("RTD high threshold (infinite R) (bit 7)"));
    }
    faults
}
use core::fmt;
impl fmt::Display for FaultStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            FaultStatus::ShortToGnd(desc) => write!(f, "{}", desc),
            FaultStatus::OpenCircuit(desc) => write!(f, "{}", desc),
            FaultStatus::OverUnderVoltage(desc) => write!(f, "{}", desc),
            FaultStatus::OverTemperature(desc) => write!(f, "{}", desc),
            FaultStatus::HighThreshold(desc) => write!(f, "{}", desc),
            // Add more variants as needed (e.g., if you expand bits 4-6)
        }
    }
}