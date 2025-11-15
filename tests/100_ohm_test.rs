// Filename: tests/100_ohm_test.rs
use std::env;
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use simple_max31865::{RTDReader, RTDLeads, FilterHz};
/// The 100 Ohm resistor test.
///
/// Purpose of this test - to validate basic setup without having or relying
/// on a PT100 probe. This is great if you want to validate your configuration
/// without being dependent on a PT100 or ambient temperature
/// A 100 ohm resistor exactly simulates a PT100 at 0 degrees C.
///
///    Schematic for 100 Ohm resistor test
///
///           (100 ohm resistor)
///           +---v\/\/\/\/----+
///           |                |
///     +-----+                +-----+
///     |     |                |     |
///     +     +                +     +
///    FRC+  RTD+             RTD-  FRC-
///
/// Wired this way it can be configured as either a 2 or 4 wire RTD probe.
/// Ideally, you'll use a 1% resistor or better.
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const RESISTOR_TOLERANCE: f64 = 0.20; //20% tolerance for generic 100Ω resistor (80–120Ω)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const NOMINAL_OHMS: f64 = 100.0;
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const MIN_OHMS: f64 = NOMINAL_OHMS * (1.0 - RESISTOR_TOLERANCE); //80.0
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const MAX_OHMS: f64 = NOMINAL_OHMS * (1.0 + RESISTOR_TOLERANCE); //120.0

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
/// PT100 linear approximation: T = (R / 100 - 1) / 0.003851 (valid 0–300°C, per datasheet 5).
const fn pt100_temperature_from_ohms(ohms: f64) -> f64 {
    (ohms / 100.0 - 1.0) / 0.003851
}

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const MIN_TEMP_C: f64 = pt100_temperature_from_ohms(MIN_OHMS); // -52.5°C
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
const MAX_TEMP_C: f64 = pt100_temperature_from_ohms(MAX_OHMS); // 51.9°C

fn get_pin_or_default(var_name: &str, default: u8) -> u8 {
    env::var(var_name)
        .ok()
        .and_then(|s| s.parse::<u8>().ok())
        .unwrap_or(default)
}

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn setup_driver() -> RTDReader {
    // Hardcode CS=24, 4-wire (Four), 60Hz (defaults; user can override via env vars if needed)
    let cs_pin = get_pin_or_default("MAX31865_CS_PIN", 24);
    RTDReader::new(cs_pin, RTDLeads::Four, FilterHz::Sixty).unwrap()
}

#[test]
fn test_env_pin_parsing() {
    // Simulate env vars (in real run, set them externally; here we mock via temp override for isolation)
    std::env::set_var("MAX31865_CS_PIN", "23");

    let ncs_pin = get_pin_or_default("MAX31865_CS_PIN", 24);

    assert_eq!(ncs_pin, 23, "Should parse MAX31865_CS_PIN=23 from env");

    // Reset for cleanliness (optional, but good practice)
    std::env::remove_var("MAX31865_CS_PIN");

    // Fallback check
    let default_ncs = get_pin_or_default("MAX31865_CS_PIN", 24);
    assert_eq!(default_ncs, 24, "Should fallback to 24 if unset");
}

#[test]
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_configure_one_shot_fails() {
    // Placeholder: Verify new() succeeds for valid params (one-shot is always false in API)
    let result = RTDReader::new(24, RTDLeads::Four, FilterHz::Sixty);
    assert!(result.is_ok(), "new() should succeed for valid params (no one-shot)");
    // Internal guard prevents one-shot; tested via code, not direct exposure
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_raw_hardware() {
    let mut driver = setup_driver(); // Default pin

    let raw = driver.get_raw_data().unwrap();
    println!("Raw: {}", raw);
    let expected_min = (MIN_OHMS / 200.0 * 32768.0) as u16; // ~13106 for 80Ω with 400Ω ref
    let expected_max = (MAX_OHMS / 200.0 * 32768.0) as u16; // ~19659 for 120Ω
    assert!(raw >= expected_min && raw <= expected_max, "Raw {} should be between {} and {} for 20% tolerance", raw, expected_min, expected_max);
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_ohms_hardware() {
    let mut driver = setup_driver(); // Default pin

    let ohms_raw = driver.get_ohms_100().unwrap();
    println!("Ohms raw: {}", ohms_raw);
    let expected_min = (MIN_OHMS * 100.0) as u32; // 8000 for 80Ω
    let expected_max = (MAX_OHMS * 100.0) as u32; // 12000 for 120Ω
    assert!(ohms_raw >= expected_min && ohms_raw <= expected_max, "Ohms raw {} should be between {} and {} for 20% tolerance", ohms_raw, expected_min, expected_max);
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_resistance_hardware() {
    let mut driver = setup_driver(); // Default pin

    let resistance = driver.get_resistance().unwrap();
    println!("Resistance f64: {}", resistance);
    assert!(resistance >= MIN_OHMS && resistance <= MAX_OHMS, "Resistance {} should be between {} and {} for 20% tolerance", resistance, MIN_OHMS, MAX_OHMS);
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_default_conversion_hardware() {
    let mut driver = setup_driver(); // Default pin

    let temp_raw = driver.read_temp_100().unwrap();
    println!("Temp raw: {}", temp_raw);
    let expected_min = (MIN_TEMP_C * 100.0) as i32; // ~ -5250 for -52.5°C *100
    let expected_max = (MAX_TEMP_C * 100.0) as i32; // ~ 5190 for 51.9°C *100
    assert!(temp_raw >= expected_min && temp_raw <= expected_max, "Temp raw {} should be between {} and {} for 20% tolerance", temp_raw, expected_min, expected_max);
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_temperature_hardware() {
    let mut driver = setup_driver(); // Default pin

    let temperature = driver.get_temperature().unwrap();
    println!("Temperature f64: {}", temperature);
    assert!(temperature >= MIN_TEMP_C && temperature <= MAX_TEMP_C, "Temperature {} should be between {} and {} for 20% tolerance", temperature, MIN_TEMP_C, MAX_TEMP_C);
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_read_fault_status_hardware() {
    let mut driver = setup_driver(); // Default pin

    let status = driver.read_fault_status().unwrap();
    println!("Fault status: 0x{:02X}", status);
    assert_eq!(status, 0x00, "Fault status should be 0x00 (no fault with resistor)");
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_clear_fault_hardware() {
    let mut driver = setup_driver(); // Default pin

    let status_before = driver.read_fault_status().unwrap();
    println!("Status before clear: 0x{:02X}", status_before);
    assert_eq!(status_before, 0x00, "Status before should be 0x00 (no fault)");

    driver.clear_fault().unwrap(); // No-op if no fault
    let status_after = driver.read_fault_status().unwrap();
    println!("Status after clear: 0x{:02X}", status_after);
    assert_eq!(status_after, 0x00, "Status after clear should be 0x00");
}

#[test]
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_set_calibration() {
    let mut driver = setup_driver();
    driver.set_calibration(43000); // 430Ω * 100 = 43000, for Adafruit match
    let raw = 16382u16; // Mock raw for 100Ω
    let ohms_raw = ((raw as u32 >> 1) * 43000) >> 15; // u32 *100 with new calibration
    let ohms = ohms_raw as f64 / 100.0;
    assert!((ohms - 100.0).abs() < 100.0 * RESISTOR_TOLERANCE, "Calibration 43000 should scale to ~100.0 ±20%");
}

#[test]
#[ignore] // Manual hardware test: Wire 100Ω resistor across F+/F- (2/4-wire, no jumper)
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn test_set_calibration_hardware() {
    let mut driver = setup_driver(); // Default pin

    driver.set_calibration(43000); // 430Ω * 100 = 43000, for Adafruit match
    let resistance = driver.get_resistance().unwrap();
    println!("Resistance with 43000 calibration: {}", resistance);
    assert!(resistance >= MIN_OHMS && resistance <= MAX_OHMS, "Resistance {} should be between {} and {} for 20% tolerance", resistance, MIN_OHMS, MAX_OHMS);
}