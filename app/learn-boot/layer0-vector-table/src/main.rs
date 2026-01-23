//! Layer 0: Boot Process Fundamentals
//!
//! LEARNING OBJECTIVES:
//! - Understand how Hubris uses cortex-m-rt for startup
//! - See the boot sequence from reset to main()
//! - Control GPIO using the PAC (Peripheral Access Crate)
//!
//! BOOT SEQUENCE (handled by cortex-m-rt):
//! 1. CPU loads SP from vector_table[0]
//! 2. CPU loads PC from vector_table[1] -> Reset handler
//! 3. cortex-m-rt's Reset handler:
//!    a. Initializes .data (copies from flash to RAM)
//!    b. Zeros .bss section
//!    c. Calls main()
//! 4. Our main() runs!
//!
//! This example uses the same crates as Hubris:
//! - cortex-m-rt: Startup code and vector table
//! - rp235x-pac: Type-safe peripheral access
//! - panic-halt: Simple panic handler

#![no_std]
#![no_main]

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use rp235x_pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A;

// Link in the PAC's interrupt vector table
// This is required by cortex-m-rt to populate the vector table
use rp235x_pac as _;

// Panic handler - halts the CPU on panic (same as Hubris uses)
use panic_halt as _;

// ============================================================================
// IMAGE_DEF BLOCK FOR RP2350 BOOT ROM
// ============================================================================
//
// The RP2350 boot ROM scans the first 4KB of the image looking for this block.
// It tells the boot ROM what kind of image this is and how to boot it.
//
// See RP2350 datasheet section 5.9.5 "Minimum viable image metadata"
#[link_section = ".image_def"]
#[used]
pub static RP235X_IMAGE_DEF: [u32; 5] = [
    0xFFFF_DED3, // START marker
    0x1021_0142, // IMAGE_TYPE: EXE | S-mode | ARM | RP2350
    0x0000_01FF, // LAST item marker
    0x0000_0000, // next block pointer = self
    0xAB12_3579, // END marker
];

// ============================================================================
// ENTRY POINT
// ============================================================================
//
// The #[entry] macro from cortex-m-rt marks this as the entry point.
// cortex-m-rt's Reset handler calls this after initializing .data and .bss.
//
// This is the same pattern Hubris uses in app/demo-pi-pico-2/src/main.rs
#[entry]
fn main() -> ! {
    // LED is on GPIO25 on the Pico 2
    const LED_PIN: usize = 25;
    let mask = 1u32 << LED_PIN;

    // Get peripheral access using the PAC
    // This is the same pattern used throughout Hubris
    let resets = unsafe { rp235x_pac::RESETS::steal() };
    let io_bank0 = unsafe { rp235x_pac::IO_BANK0::steal() };
    let pads_bank0 = unsafe { rp235x_pac::PADS_BANK0::steal() };
    let sio = unsafe { rp235x_pac::SIO::steal() };

    // Step 1: Take IO_BANK0 and PADS_BANK0 out of reset
    resets.reset().modify(|_, w| {
        w.io_bank0().clear_bit();
        w.pads_bank0().clear_bit()
    });

    // Wait for peripherals to come out of reset
    while !resets.reset_done().read().io_bank0().bit() {}
    while !resets.reset_done().read().pads_bank0().bit() {}

    // Step 2: Configure the pad for GPIO25
    // - Clear isolation (ISO) - RP2350 specific
    // - Enable output (clear OD)
    // - Enable input (set IE)
    pads_bank0.gpio(LED_PIN).modify(|_, w| {
        w.iso().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });

    // Step 3: Set GPIO function to SIO (software control)
    unsafe {
        io_bank0
            .gpio(LED_PIN)
            .gpio_ctrl()
            .write_with_zero(|w| w.funcsel().variant(FUNCSEL_A::SIO));
    }

    // Step 4: Enable GPIO25 as output
    sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });

    // Step 5: Blink forever!
    loop {
        // Turn LED ON
        sio.gpio_out_set().write(|w| unsafe { w.bits(mask) });
        delay(500_000);

        // Turn LED OFF
        sio.gpio_out_clr().write(|w| unsafe { w.bits(mask) });
        delay(500_000);
    }
}
