# ba-firmware-remotepump

Firmware for a stepper pump auto water change hub

# Quickstart
- [Install Rust](https://www.rust-lang.org/tools/install).
- Install the compilation target for your MCU. Eg run `rustup target add thumbv7em-none-eabihf`.
- Install flash and debug tools: `cargo install flip-link`, `cargo install probe-run`.
- Connect your device. Run `cargo run --release` to compile and flash.
