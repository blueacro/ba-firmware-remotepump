# ba-firmware-remotepump

Firmware for a stepper pump auto water change hub

# Quickstart
- [Install Rust](https://www.rust-lang.org/tools/install).
- Install the compilation target for your MCU. Eg run `rustup target add thumbv7em-none-eabihf`.
- Install flash and debug tools: `cargo install flip-link`, `cargo install probe-run`.
- Connect your device. Run `cargo run --release` to compile and flash.

# MCU Pinout

- PA0 10 TXEN
- PA1 11 TRQ
- PA2 12 TX
- PA3 13 RX
- PA4 14 DIR
- PA5 15 STEP
- PA6 16 EN0
- PA7 17 EN1
- PA8 29 nSLEEP
- PA9 30 VBUS
- PA10 31 SEN2
- PA11 32 USBDM
- PA12 33 USBDP
- PA13 34 SWDIO
- PA14 37 SWCLK

- PB0 18 M1
- PB1 19 M0
- PB3 39 SWO
- PB4 40 nFAULT1
- PB5 41 nFAULT2
- PB6 42 BOOT0 SW5
- PB7 43 SDA
- PB8 44 SCL
- PB9 46 SW4
- PB10 21 SW3
- PB12 25 SW2
- PB13 26 SW1
- PB14 27 LED2
- PB15 28 LED1

- PC13 2 ROUT1 (relay FET)
- PC14 3 ROUT2

