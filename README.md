# `max31865`

A generic driver for the MAX31865 RTD to Digital converter

## [Documentation](https://rudihorn.github.io/max31865/max31865/index.html)

## [Examples](https://github.com/Alan-R/simple-max31865-rs/tree/updates/examples)
Examples are in the *examples* directory from the root of the tree.

## What works

- reading the raw value and the converted temperature value either in f64 or scaled integer forms.
- Detecting and recovering from errors
- setting the ohmic calibration value
- Interaction with the chip works well.
- Reading temperatures and resistances from PT-100 sensors.

## TODO

- Working with the PT-1000 sensors
- Many other thoughts listed in the src/lib.rs file.


## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
  at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.

