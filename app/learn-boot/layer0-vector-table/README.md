# Layer 0: Boot Process Fundamentals

This is the first layer in the Hubris boot learning series. It demonstrates the boot process on ARM Cortex-M33 (RP2350 / Pico 2) using the same approach as Hubris.

## Learning Objectives

1. **Understand the ARM Cortex-M33 boot sequence**
   - CPU loads SP from vector_table[0]
   - CPU loads PC from vector_table[1] -> Reset handler
   - cortex-m-rt initializes .data and .bss
   - Our main() is called

2. **Understand the Hubris startup approach**
   - Uses cortex-m-rt for standard Cortex-M startup
   - Custom linker script (like kernel-link.x) for chip-specific needs
   - PAC crate for type-safe peripheral access

3. **Understand RP2350-specific requirements**
   - IMAGE_DEF block for boot ROM recognition
   - XIP flash at 0x10000000
   - Memory layout configuration

## Dependencies (same as Hubris uses)

| Crate | Purpose |
|-------|---------|
| `cortex-m-rt` | Startup code, vector table, .data/.bss init |
| `rp235x-pac` | Type-safe peripheral access for RP2350 |
| `cortex-m` | ARM Cortex-M utilities (delay, asm, critical-section) |
| `panic-halt` | Simple panic handler |
| `rtt-target` | RTT debug output through debug probe |

## Key Files

| File | Purpose |
|------|---------|
| `src/main.rs` | Entry point using `#[entry]`, GPIO blinky |
| `link.x` | Linker script (like Hubris's kernel-link.x) |
| `memory.x` | Memory region definitions |
| `build.rs` | Copies linker scripts to OUT_DIR |

## Boot Sequence

```
Power On
    |
    v
RP2350 Boot ROM runs
    |
    v
Boot ROM finds IMAGE_DEF in first 4KB
    |
    v
Boot ROM validates image, jumps to vector table at 0x10000000
    |
    v
CPU reads vector_table[0] -> SP = 0x20082000
    |
    v
CPU reads vector_table[1] -> PC = Reset handler
    |
    v
cortex-m-rt Reset handler:
  1. Copies .data from flash to RAM
  2. Zeros .bss section
  3. Calls main()
    |
    v
Our #[entry] fn main() runs!
    |
    v
GPIO initialized, LED blinks
```

## Binary Layout

```
Address      Section        Size    Description
0x10000000   .vector_table  1024    SP, Reset, exception handlers, interrupts
0x10000400   .image_def     20      RP2350 boot ROM metadata
0x10000414   .text          248     Code (Reset handler, main, etc.)
0x20000000   .data/.bss     0       Initialized/zero data (empty in this example)
0x20082000   Stack          -       Grows downward from top of RAM
```

## Building

```bash
cd app/learn-boot/layer0-vector-table
cargo +nightly build --release
```

Binary at: `target/thumbv8m.main-none-eabihf/release/layer0-vector-table`

## RTT Debug Output

This example uses RTT (Real-Time Transfer) to print debug messages through the debug probe's SWD connection. No UART wiring is needed - output goes directly to your terminal.

### Why RTT?
- **No extra wiring** - output goes through the debug probe's SWD connection
- **probe-rs integration** - `probe-rs run` automatically shows RTT output
- **Simple setup** - just use `rprintln!` macro

### RTT Code Pattern
```rust
use rtt_target::{rprintln, rtt_init_print};

#[entry]
fn main() -> ! {
    rtt_init_print!();  // Initialize RTT (call once at startup)
    rprintln!("Hello from Layer 0!");
    // ...
}
```

## How to Flash

### Option 1: UF2 (no debug probe needed)

1. Install elf2uf2-rs:
   ```bash
   cargo install elf2uf2-rs
   ```

2. Put Pi Pico 2 in bootloader mode:
   - Hold BOOTSEL button
   - Connect USB (or press RESET while holding BOOTSEL)
   - Release BOOTSEL - a USB drive named "RP2350" appears

3. Flash the binary:
   ```bash
   elf2uf2-rs -d target/thumbv8m.main-none-eabihf/release/layer0-vector-table
   ```
   This converts to UF2 and copies to the mounted drive automatically.

### Option 2: probe-rs (with debug probe) - Recommended

1. Install probe-rs:
   ```bash
   cargo install probe-rs-tools
   ```

2. Connect a debug probe (e.g., Picoprobe, J-Link, or another Pi Pico running debugprobe firmware)

3. Flash and run with RTT output:
   ```bash
   probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/layer0-vector-table
   ```

   RTT output will appear automatically in your terminal:
   ```
   Hello from Layer 0!
   Boot sequence complete, starting blink...
   LED ON
   LED OFF
   LED ON
   ...
   ```

### Option 3: picotool

1. Install [picotool](https://github.com/raspberrypi/picotool)

2. Put Pi Pico 2 in bootloader mode (see Option 1)

3. Flash:
   ```bash
   picotool load -x target/thumbv8m.main-none-eabihf/release/layer0-vector-table
   ```

## Comparison with Hubris

This example follows the same patterns as `app/demo-pi-pico-2`:

| Aspect | Layer 0 | Hubris demo-pi-pico-2 |
|--------|---------|----------------------|
| Entry | `#[entry] fn main()` | `#[entry] fn main()` |
| Startup | cortex-m-rt | cortex-m-rt |
| Linker | Custom link.x | kernel-link.x |
| PAC | rp235x-pac | rp235x-pac |
| IMAGE_DEF | In main.rs | In main.rs |

## Key Code Patterns

### Entry Point (same as Hubris)
```rust
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // Your code here
    loop {}
}
```

### IMAGE_DEF Block
```rust
#[link_section = ".image_def"]
#[used]
pub static RP235X_IMAGE_DEF: [u32; 5] = [
    0xFFFF_DED3, // START
    0x1021_0142, // EXE | ARM | RP2350
    0x0000_01FF, // LAST
    0x0000_0000, // next = self
    0xAB12_3579, // END
];
```

### Peripheral Access (using PAC)
```rust
let resets = unsafe { rp235x_pac::RESETS::steal() };
resets.reset().modify(|_, w| w.io_bank0().clear_bit());
```

## What's Next

Proceed to later layers to learn about:
- **Layer 2**: Task table initialization
- **Layer 3**: Privilege drop and first task launch
- **Layer 4**: Context switching between tasks
- **Layer 5**: IPC and system calls
- **Layer 6**: Memory protection (MPU)

## Important Notes for Future Layers

When implementing subsequent layers, remember:
1. **Always use cortex-m-rt** - It's the Hubris way
2. **Reuse existing crates** - PAC, panic-halt, cortex-m
3. **Follow Hubris patterns** - Look at existing code in sys/kern/, app/
4. **Study kernel-link.x** - It shows how Hubris extends cortex-m-rt's linker script
