//! Ice Bath Manual Test for MAX31865 RTD Driver
//! Verifies sensor accuracy: ~32°F in ice bath, recovery to room temp.
//! Run on Raspberry Pi; follow prompts.
//!
//! Supports CLI args (e.g., --cs-pin 24), env vars (e.g., MAX31865_CS_PIN=24), and defaults.
//! CLI: cargo run --example ice_bath_test -- --cs-pin 24 --leads Three --filter Sixty
//! Env: export MAX31865_CS_PIN=24; export MAX31865_LEADS=Three; export MAX31865_FILTER=Sixty
//! MAX31865_CS_PIN: u8 (e.g., 24).
// MAX31865_LEADS: String parsed to enum (e.g., Three or 3).
// MAX31865_FILTER: String parsed to enum (e.g., 50 or Fifty).
// Precedence: CLI arg > env var > default.

use simple_max31865::{decode_fault_status, FilterHz, RTDLeads, RTDReader};
use std::env;
use std::io;
use std::time::{Duration, Instant};

/// Parse RTDLeads from string (e.g., "Three", "3", "two").
fn parse_rtd_leads(s: &str) -> Result<RTDLeads, String> {
    match s.to_lowercase().as_str() {
        "two" | "2" => Ok(RTDLeads::Two),
        "three" | "3" => Ok(RTDLeads::Three),
        "four" | "4" => Ok(RTDLeads::Four),
        _ => Err(format!("Invalid leads: {}. Must be Two/2, Three/3, or Four/4.", s)),
    }
}

/// Parse FilterHz from string (e.g., "Sixty", "60", "fifty").
fn parse_filter_hz(s: &str) -> Result<FilterHz, String> {
    match s.to_lowercase().as_str() {
        "fifty" | "50" => Ok(FilterHz::Fifty),
        "sixty" | "60" => Ok(FilterHz::Sixty),
        _ => Err(format!("Invalid filter: {}. Must be Fifty/50 or Sixty/60.", s)),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse CLI args (simple manual scan for --key value; cargo passes after --)
    let args: Vec<String> = env::args().collect();
    let mut cli_cs_pin: Option<u8> = None;
    let mut cli_leads: Option<RTDLeads> = None;
    let mut cli_filter: Option<FilterHz> = None;

    let mut i = 1;  // Skip binary name (args[0])
    while i < args.len() {
        if args[i].starts_with("--") {
            let key = &args[i][2..];  // Strip "--"
            if i + 1 >= args.len() {
                return Err(format!("--{} requires a value", key).into());
            }
            let value = &args[i + 1];
            match key {
                "cs-pin" => {
                    cli_cs_pin = Some(value.parse().map_err(|e| format!("Invalid --cs-pin '{}': {}", value, e))?);
                }
                "leads" => {
                    cli_leads = Some(parse_rtd_leads(value).map_err(|e| format!("Invalid --leads '{}': {}", value, e))?);
                }
                "filter" => {
                    cli_filter = Some(parse_filter_hz(value).map_err(|e| format!("Invalid --filter '{}': {}", value, e))?);
                }
                _ => {
                    eprintln!("Unknown CLI arg: --{}", key);
                    eprintln!("Usage: --cs-pin <u8> | --leads <Two/2|Three/3|Four/4> | --filter <Fifty/50|Sixty/60>");
                    std::process::exit(1);
                }
            }
            i += 2;  // Skip key + value
        } else {
            eprintln!("Unexpected arg: {}", args[i]);
            i += 1;
        }
    }

    // Fallback to env vars, then defaults (CLI > env > default)
    let cs_pin_str = cli_cs_pin.map(|p| p.to_string())
        .or_else(|| env::var("MAX31865_CS_PIN").ok())
        .unwrap_or_else(|| "24".to_string());
    let cs_pin: u8 = cs_pin_str.parse().map_err(|e| format!("Invalid CS_PIN '{}': {}", cs_pin_str, e))?;

    let leads_str = cli_leads.map(|l| match l {
        RTDLeads::Two => "Two".to_string(),
        RTDLeads::Three => "Three".to_string(),
        RTDLeads::Four => "Four".to_string(),
    }).or_else(|| env::var("MAX31865_LEADS").ok())
        .unwrap_or_else(|| "Four".to_string());
    let leads = parse_rtd_leads(&leads_str).map_err(|e| format!("Invalid LEADS '{}': {}", leads_str, e))?;

    let filter_str = cli_filter.map(|f| match f {
        FilterHz::Fifty => "Fifty".to_string(),
        FilterHz::Sixty => "Sixty".to_string(),
    }).or_else(|| env::var("MAX31865_FILTER").ok())
        .unwrap_or_else(|| "Sixty".to_string());
    let filter = parse_filter_hz(&filter_str).map_err(|e| format!("Invalid FILTER '{}': {}", filter_str, e))?;

    println!("=== MAX31865 Ice Bath Test ===");
    println!("This test verifies your RTD sensor accuracy using an ice bath (~32°F/0°C).");
    println!("It checks initial room temp, monitors cooling in the bath, and recovery after removal.");
    println!("Safety: Ensure probe is fully submerged (stir ice/water mix); keep water away from electronics.");
    println!("Expected: Temps stabilize near 32°F in bath (±2°F ok), rise to 60°F+ after ~1-2 min.");
    println!("If temps don't change or faults occur, check wiring/probe.");
    println!("Config: CS pin={}, {} leads, {} Hz filter", cs_pin, leads as u8, match filter { FilterHz::Fifty => 50, _ => 60 });
    println!("(From CLI/env vars: --cs-pin/--leads/--filter or MAX31865_*; defaults used if unset)");
    println!();

    let mut reader = RTDReader::new(cs_pin, leads, filter)
        .map_err(|e| format!("Failed to init RTDReader: {:?}", e))?;

    // Step 1: Validate initial room temp (40-110°F)
    println!("Step 1: Measuring initial room temperature...");
    let raw_result = reader.get_raw_data();
    let raw = match raw_result {
        Ok(r) => r,
        Err(e) => {
            let status = reader.read_fault_status().unwrap_or(0);
            eprintln!("Raw read failed: {:?}. Fault status: 0x{:02X} -> {:?}", e, status, decode_fault_status(status));
            return Err(format!("Raw read failed in initial phase: {:?}", e).into());
        }
    };
    let resistance_result = reader.get_resistance();
    let resistance = match resistance_result {
        Ok(r) => r,
        Err(e) => {
            let status = reader.read_fault_status().unwrap_or(0);
            eprintln!("Resistance read failed: {:?}. Last raw: 0x{:04X} (LSB fault: {}). Fault status: 0x{:02X} -> {:?}", e, raw, raw & 1, status, decode_fault_status(status));
            return Err(format!("Resistance read failed in initial phase: {:?}", e).into());
        }
    };
    let temp_result = reader.get_temperature();
    let temp_c = match temp_result {
        Ok(t) => t,
        Err(e) => {
            let status = reader.read_fault_status().unwrap_or(0);
            eprintln!("Temperature read failed: {:?}. Raw: 0x{:04X} (LSB fault: {}), Resistance: {:.2} ohms. Fault status: 0x{:02X} -> {:?}", e, raw, raw & 1, resistance, status, decode_fault_status(status));
            return Err(format!("Temperature read failed in initial phase: {:?}", e).into());
        }
    };
    let temp_f = temp_c * 9.0 / 5.0 + 32.0;

    println!("Raw RTD value: 0x{:04X} (LSB fault bit: {})", raw, raw & 1);
    println!("Resistance: {:.2} ohms", resistance);
    println!("Temperature: {:.1}°F ({:.1}°C)", temp_f, temp_c);

    if !(40.0..=110.0).contains(&temp_f) {

        eprintln!("Initial temp {:.1}°F out of range (expected 40-110°F).", temp_f);
        eprintln!("Raw: 0x{:04X} (LSB fault: {}) | Resistance: {:.2} ohms | Temp: {:.1}°C", raw, raw & 1, resistance, temp_c);
        eprintln!("Likely wiring/sensor issue: Check RTD connections (expected ~108 ohms at room temp, raw ~0x6A80).");
        eprintln!("If raw ~0x0000 or resistance ~0 ohms: Short circuit (check for solder bridges).");
        eprintln!("If raw ~0x7FFF or resistance very high: Open circuit (check wire continuity).");
        return Err("Aborting test—fix hardware first.".into());
    }
    println!("Initial temp: {:.1}°F ({:.1}°C) - OK to proceed.", temp_f, temp_c);
    let initial_temp_f = temp_f;  // Save for summary
    println!();

    // Step 2: Ice Bath Phase
    println!("Step 2: Prepare ice bath (ice + water, stirred). Submerge probe fully.");
    println!("Press Enter when probe is in the bath and ready...");
    io::stdin().read_line(&mut String::new())?;
    println!("Starting bath monitoring (stops at <=35°F or 5 min)...");
    let bath_start = Instant::now();
    let mut min_temp_f = f64::INFINITY;
    let mut consecutive_errors = 0;
    let mut last_temp_f = initial_temp_f;  // For stuck detection
    let mut stuck_warning = false;
    loop {
        if consecutive_errors >= 5 {
            return Err("Too many consecutive read errors (5+). Aborting test—check hardware.".into());
        }

        match read_temp_with_faults(&mut reader, "Bath") {
            Ok(temp_f) => {
                consecutive_errors = 0;  // Reset on success
                min_temp_f = min_temp_f.min(temp_f);
                let elapsed = bath_start.elapsed().as_secs();
                let temp_c = (temp_f - 32.0) * 5.0 / 9.0;
                println!("Elapsed: {}s | Temp: {:.1}°F ({:.1}°C) | Min so far: {:.1}°F", elapsed, temp_f, temp_c, min_temp_f);

                // Simple stuck check: Warn if no change for 10s (approx 10 reads)
                if (temp_f - last_temp_f).abs() < 0.1 && elapsed > 10 && !stuck_warning {
                    println!("Warning: Temperature not changing—stir bath or check for stuck reads.");
                    stuck_warning = true;
                }
                last_temp_f = temp_f;

                if temp_f <= 35.0 || elapsed >= 300 {
                    break;
                }
            }
            Err(e) => {
                consecutive_errors += 1;
                eprintln!("Read error (#{}) in bath phase: {}", consecutive_errors, e);
                if consecutive_errors >= 3 {
                    eprintln!("Multiple errors—consider aborting if persists.");
                }
            }
        }

        std::thread::sleep(Duration::from_secs(1));
    }
    let bath_min_c = (min_temp_f - 32.0) * 5.0 / 9.0;
    let reason = if bath_start.elapsed().as_secs() >= 300 { "timeout" } else { "threshold hit" };
    println!("\nBath phase done ({}). Min temp: {:.1}°F ({:.1}°C)", reason, min_temp_f, bath_min_c);
    if min_temp_f > 34.0 {
        println!("Note: Expected ~32°F; yours was higher—ensure full immersion/stirring.");
    }
    println!();

    // Step 3: Removal/Recovery Phase
    println!("Step 3: Remove probe from bath and let it air-dry/warm to room temp.");
    println!("Press Enter when probe is removed and ready...");
    io::stdin().read_line(&mut String::new())?;
    println!("Starting recovery monitoring (stops at >=60°F or 5 min)...");
    let recovery_start = Instant::now();
    let mut max_temp_f = min_temp_f;
    consecutive_errors = 0;
    last_temp_f = min_temp_f;
    stuck_warning = false;
    loop {
        if consecutive_errors >= 5 {
            return Err("Too many consecutive read errors (5+) in recovery. Aborting.".into());
        }

        match read_temp_with_faults(&mut reader, "Recovery") {
            Ok(temp_f) => {
                consecutive_errors = 0;
                max_temp_f = max_temp_f.max(temp_f);
                let elapsed = recovery_start.elapsed().as_secs();
                let temp_c = (temp_f - 32.0) * 5.0 / 9.0;
                println!("Elapsed: {}s | Temp: {:.1}°F ({:.1}°C) | Max so far: {:.1}°F", elapsed, temp_f, temp_c, max_temp_f);

                // Stuck check
                if (temp_f - last_temp_f).abs() < 0.1 && elapsed > 10 && !stuck_warning {
                    println!("Warning: Temperature not changing—ensure probe is exposed to air.");
                    stuck_warning = true;
                }
                last_temp_f = temp_f;

                if temp_f >= 60.0 || elapsed >= 300 {
                    break;
                }
            }
            Err(e) => {
                consecutive_errors += 1;
                eprintln!("Read error (#{}) in recovery phase: {}", consecutive_errors, e);
            }
        }

        std::thread::sleep(Duration::from_secs(1));
    }
    let final_temp_f = read_temp_with_faults(&mut reader, "Final")?;
    let final_temp_c = (final_temp_f - 32.0) * 5.0 / 9.0;
    let reason = if recovery_start.elapsed().as_secs() >= 300 { "timeout" } else { "threshold hit" };
    println!("\nRecovery phase done ({}). Final temp: {:.1}°F ({:.1}°C)", reason, final_temp_f, final_temp_c);
    println!();

    // Summary
    println!("=== Test Summary ===");
    println!("Initial: {:.1}°F", initial_temp_f);
    println!("Bath min: {:.1}°F (expected ~32°F)", min_temp_f);
    println!("Recovery final: {:.1}°F (expected near initial)", final_temp_f);
    if  !(30.0..=34.0).contains(&min_temp_f){
        println!("Potential issue: Bath temp off from 32°F. Recheck setup or compare to reference code.");
    } else {
        println!("Test passed! Sensor behaving as expected.");
    }
    println!("If stuck temps or faults occurred, inspect hardware/logs.");

    Ok(())
}

/// Read temp in °F, check/clear faults, log any. Returns Err on hard failure (after recovery attempt).
fn read_temp_with_faults(reader: &mut RTDReader, phase: &str) -> Result<f64, Box<dyn std::error::Error>> {
    // Read temp
    let temp_c = match reader.get_temperature() {
        Ok(t) => Ok(t),
        Err(e) => {
            // Attempt recovery for MAX faults
            if reader.is_max_fault(&e) {
                match reader.read_fault_status() {
                    Ok(status) if status != 0 => {
                        let faults = decode_fault_status(status);
                        eprintln!("MAX fault in {} phase: {:?} (clearing...)", phase, faults);
                        if let Err(clear_e) = reader.clear_fault() {
                            eprintln!("Failed to clear fault: {:?}", clear_e);
                        }
                        // Retry the temp read once after clear
                        match reader.get_temperature() {
                            Ok(t) => Ok(t),
                            Err(retry_e) => Err(format!("Retry failed after MAX fault: {:?}", retry_e)),
                        }
                    }
                    _ => Err(format!("MAX fault during read (status unavailable): {:?}", e)),
                }
            } else {
                Err(format!("Hard read error in {} phase: {:?}", phase, e))
            }
        }
    }?;

    let temp_f = temp_c * 9.0 / 5.0 + 32.0;
    // Double-check faults post-read (non-blocking)
    if let Ok(status) = reader.read_fault_status() {
        if status != 0 {
            let faults = decode_fault_status(status);
            eprintln!("Post-read fault in {} phase: {:?} (cleared)", phase, faults);
            let _ = reader.clear_fault();  // Ignore clear error
        }
    }
    Ok(temp_f)
}