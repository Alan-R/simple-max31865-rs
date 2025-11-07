//! A simplified driver for the MAX31865 RTD to Digital converter (Raspberry Pi focus)
//!
//! # References
//! - Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
//! - Wiring diagrams:  https://www.playingwithfusion.com/docs/1203
//!

// TODO: Update and improve README (see other branches), esp sample code
// TODO: Improve and test fault handling, add to README test case
// TODO: get down to a single Error type: Replace private Error with RtdError
// TODO: Enable no_std => ![cfg_attr(not(test), no_std)]
// TODO: Stub off hardware access in abstract Trait(s) and
//       create hardware-free unit tests with mocked hardware (see other branches for examples).
// TODO: Create an ice bath manual test program - watch temperatures go down and up
//
//  Requirements for ice bath test
//      1. Explain to the user what's going to happen
//      2. Verify temperature is in the range above 40, under 110 F
//      3. Prompt user to put the probe in the ice bath
//      4. Display temperatures every second in a loop,
//         which stops after 5 minutes, or when the temperature reaches
//         35 degrees F or so.
//      5. Instruct user to remove probe from the bath
//      6. Display temperatures every second in a loop,
//         which stops after 5 minutes, or when the temperature reaches
//         60 degrees F or so.// FIXME: figure out what to do about this...

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity, SpiBus};
extern crate alloc;

// Public enums and helpers (crate-level)
#[derive(Debug, Clone, Copy)]
/// RTD lead configurations supported by the MAX31865.
pub enum RTDLeads {
    Two = 2,
    Three = 3,
    Four = 4,
}

#[derive(Debug, Clone, Copy)]
/// Noise filter settings based on mains frequency.
pub enum FilterHz {
    /// 50 Hz filter (updates ~16 Hz).
    Fifty = 1,
    /// 60 Hz filter (updates ~19 Hz).
    Sixty = 0,
}

#[derive(Debug)]
/// An enumeration of all the different faults the API can report back.
pub enum RtdError {
    InvalidChipSelect,	// The chip select lead given is out of range
    Init(String),	// Initialization failed
    Read(String),	// Reading or writing the SPI bus failed
    Fault(u8),		// An error was reported by the MAX31865
}

#[derive(Debug, Clone, Copy)]
/// All the errors the MAX31865 can report to us.
pub enum MaxFault {
    RtdInMinusUndervoltage,    // Bit 0: RTDIN- undervoltage
    RtdInPlusOvervoltage,      // Bit 1: RTDIN+ overvoltage
    RtdInMinusOvervoltage,     // Bit 2: RTDIN- overvoltage
    RtdInPlusOpen,             // Bit 3: RTDIN+ open circuit
    RtdInMinusOpen,            // Bit 4: RTDIN- open circuit
    RtdUnderOrOvertemp,        // Bit 5: RTD under/over temperature
    RtdOverOrUnderBiasVoltage, // Bit 6: RTD over/under bias voltage
    AutoConversionFault,       // Bit 7: Auto-conversion fault
}

impl MaxFault {
    /// Returns the bitmask (u8) for this MAX31865 fault type.
    pub fn bit(self) -> u8 {
        match self {
            MaxFault::RtdInMinusUndervoltage => 0b00000001,
            MaxFault::RtdInPlusOvervoltage => 0b00000010,
            MaxFault::RtdInMinusOvervoltage => 0b00000100,
            MaxFault::RtdInPlusOpen => 0b00001000,
            MaxFault::RtdInMinusOpen => 0b00010000,
            MaxFault::RtdUnderOrOvertemp => 0b00100000,
            MaxFault::RtdOverOrUnderBiasVoltage => 0b01000000,
            MaxFault::AutoConversionFault => 0b10000000,
        }
    }

    /// Returns a human-readable description for this MAX31865 fault.
    pub fn description(self) -> &'static str {
        match self {
            MaxFault::RtdInMinusUndervoltage => "RTD IN- Undervoltage",
            MaxFault::RtdInPlusOvervoltage => "RTD IN+ Overvoltage",
            MaxFault::RtdInMinusOvervoltage => "RTD IN- Overvoltage",
            MaxFault::RtdInPlusOpen => "RTD IN+ Open Circuit",
            MaxFault::RtdInMinusOpen => "RTD IN- Open Circuit",
            MaxFault::RtdUnderOrOvertemp => "RTD Under/Over Temperature",
            MaxFault::RtdOverOrUnderBiasVoltage => "RTD Over/Under Bias Voltage",
            MaxFault::AutoConversionFault => "Auto-Conversion Fault",
        }
    }
}

/// Public helper to decode a full fault status byte into a list of active faults (for users).
/// Returns a Vec of descriptions for set bits; empty if no faults.
pub fn decode_fault_status(status: u8) -> Vec<&'static str> {
    let mut faults = Vec::new();
    let all_faults = [
        (MaxFault::RtdInMinusUndervoltage, 0b00000001),
        (MaxFault::RtdInPlusOvervoltage, 0b00000010),
        (MaxFault::RtdInMinusOvervoltage, 0b00000100),
        (MaxFault::RtdInPlusOpen, 0b00001000),
        (MaxFault::RtdInMinusOpen, 0b00010000),
        (MaxFault::RtdUnderOrOvertemp, 0b00100000),
        (MaxFault::RtdOverOrUnderBiasVoltage, 0b01000000),
        (MaxFault::AutoConversionFault, 0b10000000),
    ];
    for (fault, bit) in all_faults {
        if status & bit != 0 {
            faults.push(fault.description());
        }
    }
    faults
}

#[cfg(feature = "doc")]
pub mod examples;

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub mod temp_conversion;

// Public simplified wrapper API (contains only RTDReader)
pub mod rtd_reader {
    use crate::private::{Max31865, Error as InternalError};
    use rppal::gpio::{Gpio, OutputPin as GpioOutputPin};
    use rppal::spi::{Spi, Bus, SlaveSelect, Mode as SpiMode};
    use crate::{RtdError, RTDLeads, FilterHz};  // Root public enum

    /// Simplified high-level interface for Raspberry Pi (continuous mode only).
    /// Hides SPI/GPIO setup, RDY pin (unused), and low-level details.
    /// Assumes PT100 sensor; configure with CS pin, leads, and filter.
    pub struct RTDReader {
        inner: Max31865<Spi, GpioOutputPin>,
    }

    impl RTDReader {
        /// Create a new RTDReader (Raspberry Pi only).
        ///
        /// # Arguments
        /// * `cs_pin` - GPIO pin for Chip Select (NCS, active low).
        /// * `leads` - Number of wires in the RTD setup (2/3/4).
        /// * `filter` - Noise filter based on mains frequency (50/60 Hz).
        ///
        /// Configures continuous mode (vbias=true, auto-conversion=true, one-shot=false).
        /// Defaults to 400Ω calibration. RDY pin is not used (can float).
        pub fn new(cs_pin: u8, leads: RTDLeads, filter: FilterHz) -> Result<Self, RtdError> {
            let gpio = Gpio::new().map_err(|e| RtdError::Init(format!("GPIO init failed: {}", e)))?;
            let ncs = gpio.get(cs_pin).map_err(|e| RtdError::Init(format!("NCS pin {} invalid: {}", cs_pin, e)))?.into_output_high();
            let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, SpiMode::Mode3)
                .map_err(|e| RtdError::Init(format!("SPI init failed: {}", e)))?;

            let mut inner = Max31865::new(spi, ncs).map_err(|e| RtdError::Init(match e {
                InternalError::GpioError => "NCS pin setup failed".to_string(),
                _ => "MAX31865 init failed".to_string(),
            }))?;
            inner.configure(true, true, leads, filter)
                .map_err(|e| RtdError::Init(format!("Configure failed: {:?}", e)))?;

            Ok(RTDReader { inner })
        }

        /// Read temperature in °C as f64 (PT100 lookup).
        pub fn get_temperature(&mut self) -> Result<f64, RtdError> {
            self.inner.read_temperature().map_err(map_internal_error)
        }

        /// Read resistance in ohms as f64.
        pub fn get_resistance(&mut self) -> Result<f64, RtdError> {
            self.inner.read_resistance().map_err(map_internal_error)
        }

        /// Read temperature as scaled integer (degrees Celsius * 100).
        pub fn read_temp_100(&mut self) -> Result<i32, RtdError> {
            self.inner.read_default_conversion().map_err(map_internal_error)
        }

        /// Read resistance as scaled integer (ohms * 100).
        pub fn get_ohms_100(&mut self) -> Result<u32, RtdError> {
            self.inner.read_ohms().map_err(map_internal_error)
        }

        /// Read raw RTD value (u16, for testing/low-level).
        pub fn get_raw_data(&mut self) -> Result<u16, RtdError> {
            self.inner.read_raw().map_err(map_internal_error)
        }

        /// Check if an error is a MAX31865 fault (RtdError::Fault variant).
        pub fn is_max_fault(&self, e: &RtdError) -> bool {
            matches!(e, RtdError::Fault(_))
        }

        /// Read fault status (u8 from reg 0x07; auto-clears).
        pub fn read_fault_status(&mut self) -> Result<u8, RtdError> {
            self.inner.read_fault_status().map_err(map_internal_error)
        }

        /// Clear any latched faults (no-op if none).
        pub fn clear_fault(&mut self) -> Result<(), RtdError> {
            self.inner.clear_fault().map_err(|e| RtdError::Read(match e {
                InternalError::SpiErrorTransfer => "Clear fault SPI write failed".to_string(),
                _ => "Clear fault failed".to_string(),
            }))
        }

        /// Set calibration (ohms * 100, e.g., 40000 for 400Ω).
        pub fn set_calibration(&mut self, calib: u32) {
            self.inner.set_calibration(calib);
        }
    }

    /// Map internal low-level errors to public RtdError.
    fn map_internal_error(e: InternalError) -> RtdError {
        match e {
            InternalError::SpiErrorTransfer | InternalError::GpioError => {
                RtdError::Read("SPI/GPIO transfer failed".to_string())
            }
            InternalError::MAXError => RtdError::Fault(0),  // Placeholder; call read_fault_status() for real status
        }
    }
}

// Re-export RTDReader at root for flat imports (agreed API consistency)
pub use rtd_reader::RTDReader;

// Private module for low-level driver (opaque to users)
mod private {
    use super::*;

    #[derive(Debug)]
    pub enum Error {
        /// Error transferring data to/from Max31865 chip registers
        SpiErrorTransfer,
        /// Error setting the state of a pin in the GPIO bus
        GpioError,
        /// The Max31865 chip declared an error when converting temperatures.
        /// Use `read_fault_status()` for details.
        MAXError,
    }

    pub struct Max31865<SPI, NCS> {
        spi: SPI,
        ncs: NCS,
        calibration: u32,
    }

    impl<SPI, NCS> Max31865<SPI, NCS>
    where
        SPI: SpiBus<u8>,
        NCS: OutputPin,
    {
        /// Create a new MAX31865 module (internal use only).
        pub fn new(spi: SPI, mut ncs: NCS) -> Result<Max31865<SPI, NCS>, Error> {
            let default_calib = 40000;

            ncs.set_high().map_err(|_| Error::GpioError)?;
            let max31865 = Max31865 {
                spi,
                ncs,
                calibration: default_calib, /* value in ohms multiplied by 100 */
            };

            Ok(max31865)
        }

        /// Updates the devices configuration (internal use only).
        pub fn configure(
            &mut self,
            vbias: bool,
            conversion_mode: bool,
            sensor_type_enum: RTDLeads,  // From public RTDLeads cast
            filter_mode_enum: FilterHz,   // From public FilterHz cast
        ) -> Result<(), Error> {

            // Compute sensor type and filter mode bits directly
            let sensor_type = match sensor_type_enum {
                RTDLeads::Three => 1u8,
                RTDLeads::Two | RTDLeads::Four => 0u8,  // Two or Four = 0
            };
            let filter_mode = match filter_mode_enum {
                FilterHz::Fifty => 1u8,  // Fifty = 1 (low order bit)
                FilterHz::Sixty => 0u8,  // Sixty = 0 (no lower order bits)
            };
            let conf: u8 = ((vbias as u8) << 7)
                | ((conversion_mode as u8) << 6)
                | (sensor_type << 4)  // Bit 4: sensor type (0 for 2/4-wire, 1 for 3-wire)
                | filter_mode;          // Bit 0: filter (0 for 60Hz, 1 for 50Hz)

            self.write(Register::CONFIG, conf)?;
            self.clear_fault()?; // Unlatch any boot faults (mimics Adafruit init)

            Ok(())
        }

        /// Clear latched faults (config reg bit 1 = 1)
        pub fn clear_fault(&mut self) -> Result<(), Error> {
            self.write(Register::CONFIG, 0x02)
        }

        /// Read and clear fault status reg (0x07) for bit-level diagnostics (u8 LSB)
        pub fn read_fault_status(&mut self) -> Result<u8, Error> {
            let status = self.read(Register::FAULT_STATUS)?;
            self.clear_fault()?; // Clear after read (if auto-clear needed)
            Ok(status)
        }

        /// Set the calibration reference resistance (internal use only).
        pub fn set_calibration(&mut self, calib: u32) {
            self.calibration = calib;
        }

        /// Read the raw resistance value.
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
            let temp = super::temp_conversion::LOOKUP_VEC_PT100.lookup_temperature(ohms as i32);
            Ok(temp)
        }

        /// Read the raw RTD value.
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
            self.ncs.set_low().map_err(|_| Error::GpioError)?;
            self.spi
                .transfer(&mut read_buffer, &write_buffer)
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioError)?;
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
            self.ncs.set_low().map_err(|_| Error::GpioError)?;
            self.spi
                .transfer(&mut read_buffer, &write_buffer)
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioError)?;
            Ok([read_buffer[1], read_buffer[2]]) // Return MSB, LSB (ignore dummy [0])
        }

        fn write(&mut self, reg: Register, val: u8) -> Result<(), Error> {
            self.ncs.set_low().map_err(|_| Error::GpioError)?;
            self.spi
                .write(&[reg.write_address(), val])
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioError)?;
            Ok(())
        }
    }

    #[allow(non_camel_case_types)]
    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    enum Register {    // All the lovely Max31865 register offsets
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
}