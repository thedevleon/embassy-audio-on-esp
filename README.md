# Embassy on ESP with Audio
with no_std, async, embassy, i2s and more!

## Hardware
[ESP32-C3 Pico](https://www.wemos.cc/en/latest/c3/c3_pico.html) (for now...)

## Setup
1. Install Rust >=1.75 via rustup (https://rustup.rs/)
2. `rustup target add riscv32imac-unknown-none-elf`


## How to run?
Attach the ESP32 via the USB port that is hooked up to the ESP directly (not the UART port), and run `cargo run`.

On initial flash you'll need to put the ESP into bootloader mode by holding the GPIO 9 button and resseting the ESP (via the EN/reset button).