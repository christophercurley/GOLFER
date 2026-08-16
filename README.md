# GOLFER

GOLFER is an open-source portable RF survey and telemetry platform built around the Raspberry Pi Pico 2 and LoRa radio hardware.

The project combines embedded firmware, GPS positioning, RF telemetry, persistent device configuration, host-side tooling, and purpose-built hardware into a reproducible platform for real-world radio surveying and experimentation.

> **Project status:** GOLFER is under active development and is not yet a v1 release. Hardware, protocols, APIs, and repository structure may change as the system matures.

## Repository Structure

```text
GOLFER/
├── firmware/
│   └── golfer/              # RP2350 GOLFER firmware
├── crates/
│   └── golfer-protocol/     # Shared GOLFER protocol definitions
├── apps/                    # Host-side applications and tools
├── hardware/                # CAD, PCB, and BOM files
└── docs/                    # Project documentation
```

Not all planned directories or components exist yet.

## Current Hardware

The current development platform includes:

* Raspberry Pi Pico 2 / RP2350
* SX1262 LoRa radio
* GPS receiver
* SPI TFT display

The firmware is written in Rust using the Embassy async embedded framework.

## Building and Running the Firmware

The repository is organized as a Cargo workspace.

Build the RP2350 firmware:

```sh
cargo golfer-build
```

Build, flash, and attach to the device:

```sh
cargo golfer-run
```

These commands use the `thumbv8m.main-none-eabihf` target and `probe-rs`.

## Current Development

Implemented or in progress:

* Immutable hardware-derived GOLFER identity
* User-configurable persistent device name
* Persistent system configuration in onboard flash
* Boot and runtime display interfaces
* LoRa receive and RF telemetry
* GPS acquisition and parsing
* Shared `golfer-protocol` crate

Next development is focused on the USB protocol and host-side CLI tooling.

## Open Source

GOLFER software is dual-licensed under your choice of:

* [MIT License](LICENSE-MIT)
* [Apache License 2.0](LICENSE-APACHE)

Dependency licensing is checked with `cargo-deny`.

Licensing for future hardware design files and documentation will be specified separately as those parts of the project mature.

## Contributing

GOLFER is still in early development. Contribution guidelines and additional technical documentation will be added as the project approaches its first public release.
