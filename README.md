# simple-max31865

[![Crates.io](https://img.shields.io/crates/v/simple-max31865.svg)↗](https://crates.io/crates/simple-max31865) [![Docs.rs](https://docs.rs/simple-max31865/badge.svg)↗](https://docs.rs/simple-max31865) [![License](https://img.shields.io/badge/license-MIT%20or%20Apache-2.0-blue.svg)](LICENSE-APACHE)

An easy-to-use Raspberry Pi driver for the MAX31865 RTD to Digital converter

## [Documentation↗](https://docs.rs/simple-max31865)

## [Examples↗](https://github.com/Alan-R/simple-max31865-rs/tree/main/examples)

Examples are in the *examples* directory from the root of the tree.

## What works

*   Reading the raw value and the converted temperature value either in f64 or scaled integer forms.
*   Detecting and recovering from errors.
*   Setting the ohmic calibration value.
*   Interaction with the chip works well.
*   Reading temperatures and resistances from PT-100 sensors.
*   Fault detection and recovery.
*   Support for Raspberry Pi (via `rppal` crate).
*   Tested with Max31856 hardware from Playing With Fusion and an Adafruit clone board, both with a PT100 sensor.

## TODO

See src/lib.rs for details.

Future: Testing/Mocking

*   Stub off hardware access by creating abstract implementations of Trait(s) and create minimal Mock unit tests (under a "mock" feature).
*   Create mock implementation of SPI and Pin abstractions (controlled by a "mock" feature).
*   Create "mock" hardware tests which exercise the APIs with mock feature.

Many other thoughts listed in the `src/lib.rs` file.

## Quick Start

### Raspberry Pi OS Configuration

*   You need a Raspberry Pi with a working Max31865 HAT attached with a known chip select pin.
*   Enable GPIO/SPI via `raspi-config` or your OS settings before running.

### Add to `Cargo.toml`

```plaintext
[dependencies]
simple-max31865 = "1.0"
```

### Basic Usage (Raspberry Pi)

```rust
use simple_max31865::{decode_fault_status, RTDReader, RTDLeads, FilterHz};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut max = RTDReader::new(24, RTDLeads::Three, FilterHz::Sixty)?; // CS pin 24, 3-wire PT100, 60Hz filter
    let temperature = max.get_temperature()?;
    println!("Temperature: {:.2}°C", temperature);
    let resistance = max.get_resistance()?;
    println!("Resistance: {:.2} Ω", resistance);

    match max.read_temp_100() {
        Ok(temp) => println!("Temperature: {:.2}°C", temp as f64 / 100.0),
        Err(e) => {
            if max.is_max_fault(&e) {
                let status = max.read_fault_status()?;
                println!("Fault: {:?}", decode_fault_status(status));  // Fixed: No ? needed
                let _ = max.clear_fault();
            }
            return Err(e.into());
        }
    }
    Ok(())
}
```

### Configuration Options

The examples (e.g., `ice_bath_test`) support overriding defaults via CLI arguments or environment variables.
This is useful for customizing the chip select pin, lead configuration, or filter without modifying code.

*   **CLI Arguments** (passed after `--`):

    *   `--cs-pin <u8>`: GPIO pin for chip select (default: 24).
    *   `--leads <Two|Three|Four>`: RTD lead configuration (default: "Three").
    *   `--filter <Fifty|Sixty>`: Noise filter (default: "Sixty").

    Example:

    ```
    cargo run --example ice_bath_test -- --cs-pin 8 --leads Four --filter Fifty
    ```

*   **Environment Variables** (fallback if CLI not provided):

    *   `MAX31865_CS_PIN`: GPIO pin for chip select (u8, e.g., "8").
    *   `MAX31865_LEADS`: Lead configuration (e.g., "Four").
    *   `MAX31865_FILTER`: Noise filter (e.g., "Fifty").

    Example:

    ```
    export MAX31865_CS_PIN=8
    export MAX31865_LEADS=Four
    export MAX31865_FILTER=Fifty
    cargo run --example ice_bath_test
    ```


Precedence: CLI args > env vars > hardcoded defaults.

### Running Hardware Tests

The hardware validation tests (e.g., in `tests/100_ohm_test.rs`) access shared SPI/GPIO resources and use the `serial_test` crate to run serially by default.
No special flags are needed—just run them normally:

```plaintext
cargo test test_read_resistance_hardware -- --ignored
```

For the full suite of ignored tests:

```plaintext
cargo test -- --ignored
```

## License

Licensed under either of

*   Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
*   MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT) at your option.

# References

*   [MAX31865 Datasheet↗](https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf)
*   [PT100 Sensor wiring diagrams↗](https://www.playingwithfusion.com/docs/1203) - a great reference on wiring the various types of PT100 sensors.

# Credits and License

The simple-max31865 crate is derived from version 1.0.0 of the [rs-max31865↗](https://github.com/emeric-martineau/rs-max31865)
crate by Rudi Horn and Emeric Martineau, with significant modifications:

*   Greatly simplified, opaque API hiding hardware details.
*   Added Raspberry Pi support via rppal.
*   Floating-point temperature and resistance reads.

Some original code snippets (e.g., register configs) and temperature conversion etc are reused under the MIT/Apache-2.0 license.
See the original repo for their contributions.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you,
as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.