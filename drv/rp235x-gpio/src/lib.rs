//! RP235x GPIO helper library

#![no_std]

use rp235x_pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A;
use rp235x_pac::{IO_BANK0, PADS_BANK0, RESETS, SIO};

// Initialize the necessary underlying peripherals.
// This MUST be called once at boot before any other function in this module.
pub fn init_peripherals() {
    // Take control of the RESETS peripheral.
    let resets = unsafe { RESETS::steal() };

    // Define the bitmask for the peripherals required for GPIO operations.
    let peripherals_to_enable = (1 << 6)   // IO_BANK0
        | (1 << 9)   // PADS_BANK0
        | (1 << 11); // PIO0 (which includes SIO)

    // The RESET register holds peripherals in reset when their bit is '1'.
    // We need to clear these bits to take them out of reset.
    // This is a safe read-modify-write operation using the PAC API.
    resets.reset().modify(|_r, w| unsafe {
        // Read the current reset state, clear our bits, and write back.
        w.bits(resets.reset().read().bits() & !peripherals_to_enable)
    });

    // Wait for the reset to complete. The RESET_DONE register's bits
    // will go HIGH when the corresponding peripherals are ready.
    loop {
        if (resets.reset_done().read().bits() & peripherals_to_enable) == peripherals_to_enable {
            break;
        }
    }
}

pub struct GpioPeripherals {
    pub sio: SIO,
    pub io_bank0: IO_BANK0,
    pub pads_bank0: PADS_BANK0,
}

impl GpioPeripherals {
    pub fn steal() -> Self {
        Self {
            sio: unsafe { SIO::steal() },
            io_bank0: unsafe { IO_BANK0::steal() },
            pads_bank0: unsafe { PADS_BANK0::steal() },
        }
    }
}

pub fn pico_led_init(led_pin: usize) -> GpioPeripherals {
    let peripherals = GpioPeripherals::steal();
    let mask = 1u32 << led_pin as u32;

    // Set GPIO as output
    peripherals
        .sio
        .gpio_oe_set()
        .write(|w| unsafe { w.bits(mask) });

    // Configure pad settings
    peripherals.pads_bank0.gpio(led_pin).modify(|_, w| {
        // Set input enable on, output disable off
        // RP2350: input enable defaults to off, so this is important!
        w.ie().set_bit();
        // w.od().clear_bit();
        // RP2350: remove pad isolation now a function is wired up
        // w.iso().clear_bit();
        w
    });

    // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
    // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
    unsafe {
        peripherals
            .io_bank0
            .gpio(led_pin)
            .gpio_ctrl()
            .write_with_zero(|w| w.funcsel().variant(FUNCSEL_A::SIO));
    };

    peripherals
}

pub fn pico_led_set(peripherals: &GpioPeripherals, led_pin: usize, led_on: bool) {
    let mask = 1u32 << led_pin as u32;
    if led_on {
        peripherals
            .sio
            .gpio_out_set()
            .write(|w| unsafe { w.bits(mask) });
    } else {
        peripherals
            .sio
            .gpio_out_clr()
            .write(|w| unsafe { w.bits(mask) });
    }
}
