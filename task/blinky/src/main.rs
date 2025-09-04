//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin to blink on the rp235x.

#![no_std]
#![no_main]

use cortex_m::asm::delay;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use userlib as _;

// Alias for our GPIO crate
use rp235x_gpio::{pico_led_init, pico_led_set};

#[export_name = "main"]
fn main() -> ! {
    const LED_PIN: usize = 22;

    // Initialize GPIO22 as output
    let gpio_peripherals = pico_led_init(LED_PIN);

    loop {
        // Turn LED on
        pico_led_set(&gpio_peripherals, LED_PIN, true);
        // Wait for a while
        delay(1_20_30_000);
        // Turn LED off
        pico_led_set(&gpio_peripherals, LED_PIN, false);
        // Wait for a while
        delay(1_20_30_000);
    }
}
