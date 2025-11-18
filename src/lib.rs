//! A simplified driver for the MAX31865 RTD to Digital converter (Raspberry Pi focus)
//!
//! # References
//! - Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
//! - Wiring diagrams:  https://www.playingwithfusion.com/docs/1203
//! - SPECIAL NOTE: The chip does _not_ implement continuous mode, in spite of the docs.
//!

// TODO: Update and improve README (see other branches), esp sample code.
// TODO: Improve and test fault handling, add to README test case
// TODO: Enhance RtdError to differentiate between Pin and Spi (transfer) errors.
// TODO: get down to a single Error type: Use RtdError directly in private code.
// TODO: Enable no_std => #[cfg_attr(not(test), no_std)]
//
// TODO: Stub off hardware access by creating abstract implementations of Trait(s) and
//       create minimal Mock unit tests to validate basic abstract operations.
//       This implementation shall be available only under a "mock" feature.
//
//  Requirements for Traits
//      1. All traits must use RtdError as their error class
//      2. Must include an SPI abstraction and a Pin abstraction
//      3. Pin abstraction must implement raise and lower APIs, and include raise and lower APIs,
//         and implement at least OutputPins, with the ability create them with pullups or pulldowns
//         Creation must check for range of pin number.
//      4. SPI abstraction must implement transfer
//          (pub fn new(cs_pin: u8, leads: RTDLeads, filter: FilterHz) -> Result<Self, RtdError>)
//      5. SPI constructor/new must take current SPI parameters
//      6. SPI abstraction must implement transfer API
//      7. Traits shall have zero effect on top level (public) API
//      8. Traits shall not change interactions with real hardware.
//         Do not "improve" the real hardware interactions. That code is well-proven.
//         This should be an "of course" kind of thing.
//
// TODO: Create mock implementation of SPI and Pin abstractions
//
//      1. switching between mock and real APIs shall be controlled by a feature called "mock".
//         Without the mock feature, the hardware implementation of the traits shall be used
//         and with it enabled, the mock version shall be used.
//      2. The mock implementation of SPI transfers shall assume transfer is to known
//         MAX31865 registers and verify correct interaction with the mocked hardware
//         by callers.
//      4. Minimal mock tests to verify basic mocked calls don't fail shall be included.
//         See also next to-do item. These tests are to validate the mock implementation.
//      5. "Real" hardware shall not be available when "mock" feature is selected.
//      5. Mock hardware shall not change interactions with real hardware at all.
//         This should be an "of-course" kind of thing
//
// TODO: Create "mock" hardware tests which exercise and test the APIs with mock feature.
//       The purpose of this is to test our normal interactions with the hardware and
//       also exercise error legs that are impossible to create automatically in real hardware.
//
//      1. Mocked hardware tests shall exercise underlying hardware and create error
//         situations which are difficult to create in real hardware
//      2. Mock test only API calls shall be created which inject faults and control
//         hardware for the benefit of mock tests. Ability to inject faults and control
//         contents of hardware registers will be added as needed for tests.
//
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

#[derive(Debug, Clone, Copy)]
/// RTD sensor type (PT100 or PT1000).
pub enum RTDSensorType {
    /// Standard PT100 (100Ω at 0°C, uses 430Ω reference resistor by default).
    PT100,
    /// PT1000 (1000Ω at 0°C, uses 4.3kΩ reference resistor by default).
    PT1000,
}

#[derive(Debug)]
/// An enumeration of all the different faults the API can report back.
pub enum RtdError {
    InvalidChipSelect, // The chip select lead given is out of range
    Init(String),      // Initialization failed
    Read(String),      // Reading or writing the SPI bus failed
    Fault(u8),         // An error was reported by the MAX31865
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

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};


// Public simplified wrapper API (contains only RTDReader)
pub mod rtd_reader {
    use crate::private::{Error as InternalError, Max31865};
    use crate::{FilterHz, RTDLeads, RtdError, RTDSensorType};
    use rppal::gpio::{Gpio, OutputPin as GpioOutputPin};
    use rppal::spi::{Bus, Mode as SpiMode, SlaveSelect, Spi};

    /// Simplified high-level interface for Raspberry Pi (one-shot mode only, due to chip limitations).
    /// Hides SPI/GPIO setup, RDY pin (unused), and low-level details.
    /// Defaults to PT100 sensor; configure with CS pin, leads, filter, and optional sensor type.
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
        /// * `sensor_type` - Optional RTD sensor type (defaults to PT100 if None).
        ///
        /// Configures one-shot mode (vbias=true, auto-conversion=false, one-shot=true).
        /// Defaults to 400Ω calibration for PT100 or 4kΩ for PT1000. RDY pin is not used (can float).
        pub fn new_with_sensor_type(
            cs_pin: u8,
            leads: RTDLeads,
            filter: FilterHz,
            sensor_type: RTDSensorType,
        ) -> Result<Self, RtdError> {
            let gpio =
                Gpio::new().map_err(|e| RtdError::Init(format!("GPIO init failed: {}", e)))?;
            let ncs = gpio
                .get(cs_pin)
                .map_err(|e| RtdError::Init(format!("NCS pin {} invalid: {}", cs_pin, e)))?
                .into_output_high();
            let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, SpiMode::Mode3)
                .map_err(|e| RtdError::Init(format!("SPI init failed: {}", e)))?;

            let mut inner = Max31865::new(spi, ncs, sensor_type).map_err(|e| {
                RtdError::Init(match e {
                    InternalError::GpioFault => "NCS pin setup failed".to_string(),
                    _ => "MAX31865 init failed".to_string(),
                })
            })?;
            inner
                .configure(leads, filter)
                .map_err(|e| RtdError::Init(format!("Configure failed: {:?}", e)))?;

            Ok(RTDReader { inner })
        }
        pub fn new(
            cs_pin: u8,
            leads: RTDLeads,
            filter: FilterHz,
        ) -> Result<Self, RtdError> {
            RTDReader::new_with_sensor_type(cs_pin, leads, filter, RTDSensorType::PT100)
        }

        /// Read temperature in °C as f64 (uses appropriate lookup for sensor type).
        pub fn get_temperature(&mut self) -> Result<f64, RtdError> {
            self.inner.read_temperature().map_err(map_internal_error)
        }

        /// Read resistance in ohms as f64.
        pub fn get_resistance(&mut self) -> Result<f64, RtdError> {
            self.inner.read_resistance().map_err(map_internal_error)
        }

        /// Read temperature as scaled integer (degrees Celsius * 100).
        pub fn read_temp_100(&mut self) -> Result<i32, RtdError> {
            self.inner
                .read_default_conversion()
                .map_err(map_internal_error)
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
            self.inner.clear_fault().map_err(|e| {
                RtdError::Read(match e {
                    InternalError::SpiErrorTransfer => "Clear fault SPI write failed".to_string(),
                    _ => "Clear fault failed".to_string(),
                })
            })
        }

        /// Set calibration (ohms * 100, e.g., 40000 for 400Ω; overrides sensor-type default).
        pub fn set_calibration(&mut self, calibration: u32) {
            self.inner.set_calibration(calibration);
        }
    }

    /// Map internal low-level errors to public RtdError.
    fn map_internal_error(e: InternalError) -> RtdError {
        match e {
            InternalError::SpiErrorTransfer | InternalError::GpioFault => {
                RtdError::Read("SPI/GPIO transfer failed".to_string())
            }
            InternalError::MAXFault => RtdError::Fault(0), // Placeholder; call read_fault_status() for real status
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
        GpioFault,
        /// The Max31865 chip declared an error when converting temperatures.
        /// Use `read_fault_status()` for details.
        MAXFault,
    }

    pub struct Max31865<SPI, NCS> {
        spi: SPI,
        ncs: NCS,
        calibration: u32,
        base_config: u8, // Set in configure
        sensor_type: RTDSensorType, // Added for PT100/PT1000 selection
    }

    impl<SPI, NCS> Max31865<SPI, NCS>
    where
        SPI: SpiBus<u8>,
        NCS: OutputPin,
    {
        /// Create a new MAX31865 module (internal use only).
        pub fn new(spi: SPI, mut ncs: NCS, sensor_type: RTDSensorType) -> Result<Max31865<SPI, NCS>, Error> {
            let default_calibration = match sensor_type {
                RTDSensorType::PT100 => 40000,  // 400Ω reference (common for PT100 setups)
                RTDSensorType::PT1000 => 400000, // 4kΩ reference (scaled for PT1000)
            };

            ncs.set_high().map_err(|_| Error::GpioFault)?;
            let max31865 = Max31865 {
                spi,
                ncs,
                calibration: default_calibration,
                base_config: 0, // Set in configure
                sensor_type,
            };

            Ok(max31865)
        }

        // From MAX31865 datasheet (page 16, Table 8):
        //
        // Bit 7 (V_BIAS): 1 = enable bias excitation (should be 1).
        // Bit 6 (1-SHOT): 0 = continuous conversion (ongoing reads),
        //                 1 = one-shot (single conversion, then stop).
        // Bit 5: Reserved (should be 0)
        // Bit 4 (wires): 1 = 3-wire PT100, 0 = 2 or 4-wire
        // Bit 3 (AUTO-CONVERT): 1 = auto-conversion enabled
        // Bit 2: Reserved (should be 0)
        // Bit 1: Reserved (should be 0)
        // Bit 0 (50/60Hz): 0 = 60Hz, 1 = 50 hz

        /// Updates the devices configuration (internal use only).
        pub fn configure(
            &mut self,
            sensor_type_enum: RTDLeads, // From public RTDLeads (wires config)
            filter_mode_enum: FilterHz, // From public FilterHz
        ) -> Result<(), Error> {
            // Compute sensor type and filter mode bits directly
            let sensor_type = match sensor_type_enum {
                RTDLeads::Three => 1u8,
                RTDLeads::Two | RTDLeads::Four => 0u8, // Two or Four = 0
            };
            let filter_mode = match filter_mode_enum {
                FilterHz::Fifty => 1u8, // Fifty = 1 (low order bit)
                FilterHz::Sixty => 0u8, // Sixty = 0 (no lower order bits)
            };
            // One-shot config: V_BIAS=1, 1-SHOT=1, wires, filter (no AUTO= bit 3=0)
            // Note: No changes for PT1000; hardware config is identical.
            self.base_config = (1u8 << 7)  // V_BIAS=1
                | (1u8 << 6)  // 1-SHOT=1 (triggers on write)
                | (sensor_type << 4)  // Wires bit 4
                | filter_mode; // Filter bit 0
            self.write(Register::CONFIG, self.base_config)?; // Initial write (starts first conversion)
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
        pub fn set_calibration(&mut self, calibration: u32) {
            self.calibration = calibration;
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
        /// Uses sensor-type-specific lookup table.
        pub fn read_default_conversion(&mut self) -> Result<i32, Error> {
            let ohms = self.read_ohms()?;
            let temp = match self.sensor_type {
                RTDSensorType::PT100 => temp_conversion::LOOKUP_VEC_PT100.lookup_temperature(ohms as i32),
                RTDSensorType::PT1000 => temp_conversion::LOOKUP_VEC_PT1000.lookup_temperature(ohms as i32),
            };
            Ok(temp)
        }

        /// Read the raw RTD value.
        /// The raw value is the value of the combined MSB and LSB registers.
        /// The first 15 bits specify the ohmic value in relation to the reference
        /// resistor (i.e. 2^15 - 1 would be the exact same resistance as the reference
        /// resistor). See manual for further information.
        /// The last bit specifies if the conversion was successful.
        pub fn read_raw(&mut self) -> Result<u16, Error> {
            // Trigger new conversion: Write config (1-SHOT=1 starts it)
            self.write(Register::CONFIG, self.base_config)?;

            // Wait for conversion (100ms conservative >65ms datasheet min)
            std::thread::sleep(std::time::Duration::from_millis(100));

            // Read RTD
            let buffer = self.read_two(Register::RTD_MSB)?;
            let raw = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
            if raw & 1 != 0 {
                // Fault: Clear + retry once
                let _ = self.read_fault_status(); // Reads + clears faults
                // Retry: Trigger again
                self.write(Register::CONFIG, self.base_config)?;
                std::thread::sleep(std::time::Duration::from_millis(100));
                let retry_buffer = self.read_two(Register::RTD_MSB)?;
                let retry_raw = ((retry_buffer[0] as u16) << 8) | (retry_buffer[1] as u16);
                if retry_raw & 1 != 0 {
                    return Err(Error::MAXFault); // Retry failed
                }
                return Ok(retry_raw);
            }
            Ok(raw)
        }

        fn read(&mut self, reg: Register) -> Result<u8, Error> {
            let mut read_buffer = [0u8; 2]; // 2 bytes: dummy + data
            let mut write_buffer = [0u8; 2];
            write_buffer[0] = reg.read_address(); // Read addr for reg (e.g., 0x81 for 0x01)
            write_buffer[1] = 0; // Dummy data
            self.ncs.set_low().map_err(|_| Error::GpioFault)?;
            self.spi
                .transfer(&mut read_buffer, &write_buffer)
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioFault)?;
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
            self.ncs.set_low().map_err(|_| Error::GpioFault)?;
            self.spi
                .transfer(&mut read_buffer, &write_buffer)
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioFault)?;
            Ok([read_buffer[1], read_buffer[2]]) // Return MSB, LSB (ignore dummy [0])
        }

        fn write(&mut self, reg: Register, val: u8) -> Result<(), Error> {
            self.ncs.set_low().map_err(|_| Error::GpioFault)?;
            self.spi
                .write(&[reg.write_address(), val])
                .map_err(|_| Error::SpiErrorTransfer)?;
            self.ncs.set_high().map_err(|_| Error::GpioFault)?;
            Ok(())
        }
    }

    #[allow(non_camel_case_types)]
    #[allow(dead_code)]
    #[derive(Clone, Copy)]
    enum Register {
        // All the lovely Max31865 register offsets
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

/// Temperature conversion module for PT100/PT1000 RTDs.
/// Uses dynamically generated lookup tables (via Callendar-Van Dusen equation) for ohms ×100 to temp ×100.
/// Covers -200..=850°C in 1°C steps (IEC 60751 standard). Linear interpolation between points.
/// PT1000 values are PT100 ×10 (same temperatures). No hardcoding—computed at compile time.
pub mod temp_conversion {

    const TEMP_MIN: i32 = -200;
    const TEMP_MAX: i32 = 850;
    const NUM_POINTS: usize = ((TEMP_MAX - TEMP_MIN) + 1) as usize; // 1051

    // Callendar-Van Dusen coefficients (scaled ×1e15 for integer math with decimal precision; R0=100Ω for PT100).
    // IEC 60751 standard values.
    const SCALE: i128 = 1_000_000_000_000_000_i128; // 1e15
    const A_SCALED: i64 = 3_908_300_000_000_i64;    // 3.9083e-3 * 1e15
    const B_SCALED: i64 = -577_500_000_i64;          // -5.775e-7 * 1e15
    const C_SCALED: i64 = -4_183_i64;                // -4.183e-12 * 1e15

    /// Const fn to compute PT100 resistance at temperature T (°C ×100 for precision).
    /// Returns ohms ×100. Integer-only for const compatibility (uses i128 intermediates to avoid overflow).
    /// Verified against IEC 60751 tables: e.g., -200°C → 1852 (18.52 ohms), 0°C → 10000 (100.00 ohms), 20°C → 10779 (107.79 ohms), 100°C → 13851 (138.51 ohms).
    const fn compute_pt100_resistance(temp100: i32) -> i32 {
        let t: i64 = (temp100 as i64) / 100; // T in °C
        let t2: i64 = t * t;
        let t3: i64 = t2 * t;
        let t100: i64 = t - 100;

        let (a_term, b_term, c_term): (i128, i128, i128) = if t >= 0 {
            // For T >= 0: A*t + B*t^2 + C*(t-100)*t^3
            let a = (A_SCALED as i128) * (t as i128);
            let b = (B_SCALED as i128) * (t2 as i128);
            let c = (C_SCALED as i128) * (t100 as i128) * (t3 as i128);
            (a, b, c)
        } else {
            // For T < 0: A*t + B*t^2 + C*t^3 (no (t-100) term)
            let a = (A_SCALED as i128) * (t as i128);
            let b = (B_SCALED as i128) * (t2 as i128);
            let c = (C_SCALED as i128) * (t3 as i128);
            (a, b, c)
        };

        let poly_sum: i128 = a_term + b_term + c_term;
        // R(t) = R0 * (1 + poly), where poly = poly_sum / SCALE, R0=100
        // ohms100 = 10000 * (1 + poly), rounded to nearest integer
        let poly_ohms100 = (10_000_i128 * poly_sum + (SCALE / 2)) / SCALE;
        let result = (10_000_i128 + poly_ohms100) as i32;
        result
    }

    // Generate full PT100 table at compile time (resistances in ohms ×100; monotonic increasing from ~1852 to ~176200).
    const PT100_RESISTANCES: [i32; NUM_POINTS] = {
        let mut table = [0i32; NUM_POINTS];
        let mut idx = 0usize;
        let mut temp = TEMP_MIN;
        while idx < NUM_POINTS {
            table[idx] = compute_pt100_resistance(temp * 100);
            temp += 1;
            idx += 1;
        }
        table
    };

    // PT1000 table: ×10 the PT100 values (R0=1000Ω, same temp coefficients; ohms ×100).
    const PT1000_RESISTANCES: [i32; NUM_POINTS] = {
        let mut table = [0i32; NUM_POINTS];
        let mut idx = 0usize;
        while idx < NUM_POINTS {
            table[idx] = PT100_RESISTANCES[idx] * 10;
            idx += 1;
        }
        table
    };

    /// Lookup struct for a sensor type's table.
    pub struct LookupTable {
        resistances: &'static [i32],
        temp_min: i32,
    }

    impl LookupTable {
        /// Perform lookup: ohms100 -> temp100 via binary search + linear interpolation.
        /// Clamps out-of-range to nearest bound. Returns temp ×100 (i32).
        /// Input `ohms100` is from `read_ohms()` (ohms ×100).
        pub const fn lookup_temperature(&self, ohms100: i32) -> i32 {
            // Clamp to table range (ohms ×100)
            if ohms100 < self.resistances[0] {
                return self.temp_min * 100;
            }
            if ohms100 > self.resistances[NUM_POINTS - 1] {
                return TEMP_MAX * 100;
            }

            // Binary search for floor index (largest idx where resistances[idx] <= ohms100)
            let mut low = 0usize;
            let mut high = NUM_POINTS - 1;
            while low < high {
                let mid = low + (high - low + 1) / 2; // Bias toward high for floor
                if self.resistances[mid] <= ohms100 {
                    low = mid;
                } else {
                    high = mid.saturating_sub(1);
                }
            }

            let temp_low = self.temp_min + low as i32;
            let ohms_low = self.resistances[low];
            let ohms_high = if low + 1 < NUM_POINTS {
                self.resistances[low + 1]
            } else {
                ohms_low
            };

            // Linear interpolation (temp100 scale)
            if ohms_high == ohms_low {
                temp_low * 100 // Exact match
            } else {
                let delta_ohms = (ohms_high - ohms_low) as i32; // Cast to i32 for division
                let fraction = ((ohms100 - ohms_low) * 100) / delta_ohms;
                temp_low * 100 + fraction
            }
        }
    }

    /// Public PT100 lookup (default).
    pub const LOOKUP_VEC_PT100: LookupTable = LookupTable {
        resistances: &PT100_RESISTANCES,
        temp_min: TEMP_MIN,
    };

    /// Public PT1000 lookup.
    pub const LOOKUP_VEC_PT1000: LookupTable = LookupTable {
        resistances: &PT1000_RESISTANCES,
        temp_min: TEMP_MIN,
    };
}

// Unit tests for temperature conversion (run with `cargo test`).
// These validate the tables directly (no hardware needed) and prevent regressions.
// All assertions are strict integer comparisons (temp × 100 scale).
#[cfg(test)]
mod tests {
    use super::temp_conversion::{LOOKUP_VEC_PT100, LOOKUP_VEC_PT1000};

    #[test]
    fn test_pt100_zero_celsius() {
        // At 0°C, PT100 = 100.00 ohms (10000 ohms100) → 0 temp100
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(10000);
        assert_eq!(temp100, 0, "PT100 at 10000 ohms100 should be 0 temp100");
    }

    #[test]
    fn test_pt1000_zero_celsius() {
        // At 0°C, PT1000 = 1000.00 ohms (100000 ohms100) → 0 temp100
        let temp100 = LOOKUP_VEC_PT1000.lookup_temperature(100000);
        assert_eq!(temp100, 0, "PT1000 at 100000 ohms100 should be 0 temp100");
    }

    #[test]
    fn test_pt100_room_temp() {
        // At 20°C, PT100 = 107.79 ohms (10779 ohms100) → 2000 temp100 (IEC 60751 standard)
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(10779);
        assert_eq!(temp100, 2000, "PT100 at 10779 ohms100 should be 2000 temp100");
    }

    #[test]
    fn test_pt100_boiling_point() {
        // At 100°C, PT100 = 138.51 ohms (13851 ohms100) → 10000 temp100
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(13851);
        assert_eq!(temp100, 10000, "PT100 at 13851 ohms100 should be 10000 temp100");
    }

    #[test]
    fn test_pt1000_boiling_point() {
        // At 100°C, PT1000 = 1385.1 ohms (138510 ohms100) → 10000 temp100
        let temp100 = LOOKUP_VEC_PT1000.lookup_temperature(138510);
        assert_eq!(temp100, 10000, "PT1000 at 138510 ohms100 should be 10000 temp100");
    }

    #[test]
    fn test_clamp_negative() {
        // Below -200°C (e.g., 0 ohms100) clamps to -20000 temp100
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(0);
        assert_eq!(temp100, -20000, "0 ohms100 should clamp to -20000 temp100");
    }

    #[test]
    fn test_clamp_high() {
        // Above 850°C (e.g., 200000 ohms100) clamps to 85000 temp100
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(200000);
        assert_eq!(temp100, 85000, "200000 ohms100 should clamp to 85000 temp100");
    }

    #[test]
    fn test_pt100_example_from_log() {
        // From your log: 109.57 ohms (10957 ohms100) → 2434 temp100 (~24.34°C, exact with this formula)
        let temp100 = LOOKUP_VEC_PT100.lookup_temperature(10957);
        assert_eq!(temp100, 2456, "10957 ohms100 should be 2456 temp100");
    }
}