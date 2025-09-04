//! RP235x GPIO helper library

#![no_std]

use rp235x_pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A;
use rp235x_pac::{IO_BANK0, PADS_BANK0, SIO};

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
        w.od().clear_bit();
        // RP2350: remove pad isolation now a function is wired up
        w.iso().clear_bit();
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
