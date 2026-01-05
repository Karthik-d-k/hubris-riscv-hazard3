# Hubris Startup Process - Complete Walkthrough

## Using LPC55xpresso as Example Application

This document traces the **complete boot sequence** from power-on/reset to running Hubris tasks on the LPC55xpresso board, with actual code excerpts and detailed explanations.

---

## Table of Contents
1. [Boot Sequence Overview](#boot-sequence-overview)
2. [Phase 1: Hardware Reset to Reset Handler](#phase-1-hardware-reset-to-reset-handler)
3. [Phase 2: Reset Handler to Rust main()](#phase-2-reset-handler-to-rust-main)
4. [Phase 3: Application-Specific Setup](#phase-3-application-specific-setup)
5. [Phase 4: Kernel Initialization](#phase-4-kernel-initialization)
6. [Phase 5: Task Initialization](#phase-5-task-initialization)
7. [Phase 6: Starting the First Task](#phase-6-starting-the-first-task)
8. [Phase 7: Transition to User Mode](#phase-7-transition-to-user-mode)
9. [Memory Layout](#memory-layout)
10. [Visual Summary](#visual-summary)

---

## Boot Sequence Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        POWER ON / RESET                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 1: Hardware Reset                                         │
│ • Cortex-M33 loads SP from 0x0000_0000                          │
│ • Cortex-M33 loads PC from 0x0000_0004 (reset vector)           │
│ • Jumps to Reset Handler                                        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 2: cortex_m_rt Reset Handler                              │
│ • Initialize stack pointer (if needed)                          │
│ • Enable FPU                                                    │
│ • Copy .data from Flash → RAM                                   │
│ • Zero .bss section                                             │
│ • Call main() (#[entry] function)                               │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 3: app/lpc55xpresso/src/main.rs                           │
│ • Get core and peripheral instances                             │
│ • Determine clock speed                                         │
│ • Call lpc55_rot_startup::startup()                             │
│   - Enable MPU for USB RAM                                      │
│   - Verify flash images                                         │
│   - Run DICE attestation                                        │
│   - Enable debugging if beacon set                              │
│   - Prepare handoff memory                                      │
│ • Call kern::startup::start_kernel()                            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 4: sys/kern/src/startup.rs::start_kernel()                │
│ • Set clock frequency                                           │
│ • Read HUBRIS_TASK_DESCS from Flash                             │
│ • Initialize task table in RAM                                  │
│ • For each task: Task::from_descriptor()                        │
│ • For each task: arch::reinitialize()                           │
│ • Select first task to run (highest priority with start=true)   │
│ • Call arch::start_first_task()                                 │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 5: sys/kern/src/arch/arm_m.rs::start_first_task()         │
│ • Enable fault handlers                                         │
│ • Set exception priorities                                      │
│ • Configure SysTick timer                                       │
│ • Enable MPU                                                    │
│ • Set CURRENT_TASK_PTR                                          │
│ • Load PSP (Process Stack Pointer) for first task               │
│ • Restore task registers (r4-r11)                               │
│ • Execute SVC #0xFF to trap into kernel                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 6: SVCall Handler (assembly in arm_m.rs)                  │
│ • Detect startup mode (LR == 0xFFFFFFF9)                        │
│ • Switch to unprivileged mode (CONTROL = 0x03)                  │
│ • Load task's exception return value (0xFFFFFFFD)               │
│ • Pop hardware-saved registers from task stack                  │
│ • Return from exception → TASK IS NOW RUNNING                   │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ PHASE 7: Task Execution (e.g., task-jefe)                       │
│ • Running in unprivileged Thread mode                           │
│ • Using Process Stack Pointer (PSP)                             │
│ • Protected by MPU                                              │
│ • Can make syscalls via SVC instruction                         │
│ • Can be preempted by higher-priority tasks                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Hardware Reset to Reset Handler

### What Happens

When you power on or reset the LPC55S69 microcontroller, the Cortex-M33 core performs these steps automatically in hardware:

1. **Load Stack Pointer (SP)**: Read 32-bit value at address `0x0000_0000` → SP (Main Stack Pointer)
2. **Load Program Counter (PC)**: Read 32-bit value at address `0x0000_0004` → PC (Reset Vector)
3. **Jump**: Start executing code at the address loaded into PC

### Memory Layout at Reset

The first 8 bytes of Flash memory contain the **vector table**:

```
Address       Value                  Description
──────────────────────────────────────────────────────────────
0x0000_0000   0x2004_4000           Initial SP (top of RAM)
0x0000_0004   0x0000_1234           Reset Handler address
0x0000_0008   0x0000_5678           NMI Handler
0x0000_000C   0x0000_9ABC           HardFault Handler
...           ...                    Other exception handlers
```

**Key Point**: This vector table is generated by the linker and placed at the start of Flash memory.

### Code Location

The vector table is defined by the `cortex_m_rt` crate, which Hubris uses. The reset handler is also provided by `cortex_m_rt`.

---

## Phase 2: Reset Handler to Rust main()

### Reset Handler (cortex_m_rt)

The reset handler is responsible for preparing the environment for Rust code. From the `cortex_m_rt` documentation, it performs:

```rust
// Pseudocode of what cortex_m_rt's reset handler does:

Reset_Handler:
    // 1. Initialize data section (.data)
    //    Copy initialized variables from Flash to RAM
    for addr in _sdata.._edata {
        *addr = *(addr + _sidata)
    }

    // 2. Zero BSS section (.bss)
    //    Clear uninitialized static variables
    for addr in _sbss.._ebss {
        *addr = 0
    }

    // 3. Enable FPU (on Cortex-M33)
    //    Required for floating-point operations
    CPACR |= (0xF << 20)  // Enable CP10 and CP11

    // 4. Call application's main function
    //    The function marked with #[entry]
    main()
```

### Memory Sections

**Flash Memory (.text, .rodata):**
- Contains executable code and constants
- Read-only, never changes after flashing

**RAM (.data, .bss):**
- `.data`: Initialized global/static variables (copied from Flash)
- `.bss`: Uninitialized global/static variables (zeroed)
- Stack and heap

**Why This Matters:**
Rust requires all static variables to be properly initialized before any Rust code runs. The reset handler ensures this contract is upheld.

---

## Phase 3: Application-Specific Setup

### LPC55xpresso main() Function

**File**: `app/lpc55xpresso/src/main.rs`

```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use lpc55_rot_startup::{get_clock_speed, startup};
use unwrap_lite::UnwrapLite;

#[entry]
fn main() -> ! {
    // 1. Get peripheral instances
    let core_peripherals = cortex_m::Peripherals::take().unwrap_lite();
    let peripherals = lpc55_pac::Peripherals::take().unwrap_lite();

    // 2. Determine clock speed
    let (cycles_per_ms, _div) = get_clock_speed(&peripherals);

    // 3. LPC55-specific startup sequence
    startup(&core_peripherals, &peripherals);

    // 4. Start Hubris kernel (never returns)
    unsafe { kern::startup::start_kernel(cycles_per_ms * 1_000) }
}
```

**Annotations:**
- `#![no_std]`: No standard library (embedded environment)
- `#![no_main]`: We provide our own main via #[entry]
- `#[entry]`: Marks this as the reset handler target
- `-> !`: Never returns (diverging function)

### get_clock_speed()

**File**: `lib/lpc55-rot-startup/src/lib.rs` (lines 239-267)

```rust
pub fn get_clock_speed(peripherals: &lpc55_pac::Peripherals) -> (u32, u8) {
    let syscon = &peripherals.SYSCON;

    let a = syscon.mainclksela.read().bits();
    let b = syscon.mainclkselb.read().bits();
    let div = syscon.ahbclkdiv.read().bits();

    // Expected configuration from ROM bootloader
    const EXPECTED_MAINCLKSELA: u32 = 3;  // FRO 96 MHz
    const EXPECTED_MAINCLKSELB: u32 = 0;  // Main Clock A

    // Verify expected configuration
    if a != EXPECTED_MAINCLKSELA || b != EXPECTED_MAINCLKSELB {
        panic!();  // Configuration mismatch
    }

    // Return clock speed in kHz
    if div == 0 {
        (96, div as u8)      // 96 MHz
    } else {
        (48, div as u8)      // 48 MHz (divided)
    }
}
```

**Purpose**: Reads the SYSCON registers to determine the actual CPU clock frequency. This is critical for:
- SysTick timer configuration (needs to know CPU speed)
- Flash programming operations
- UART baud rate calculations
- Debug trace (ITM) timing

### startup() Function

**File**: `lib/lpc55-rot-startup/src/lib.rs` (lines 137-235)

This is a **complex** LPC55-specific initialization function. Let's break it down:

```rust
pub fn startup(
    core_peripherals: &cortex_m::Peripherals,
    peripherals: &lpc55_pac::Peripherals,
) {
    // 1. VERIFY ROM VERSION
    // Read SYSCON_DIEID register to check ROM version
    let val = unsafe { core::ptr::read_volatile(0x50000ffc as *const u32) };
    if val & 1 != ROM_VER {
        panic!()  // ROM version mismatch
    }

    // 2. LOCK FLASH (if "locked" feature enabled)
    #[cfg(feature = "locked")]
    lock_flash();

    // 3. CONFIGURE MPU FOR USB RAM
    // USB RAM needs special attributes (not device memory)
    let mpu = &core_peripherals.MPU;
    apply_memory_protection(mpu);

    // 4. PREPARE FLASH DRIVER
    let mut flash = drv_lpc55_flash::Flash::new(&peripherals.FLASH);

    // 5. TURN ON HANDOFF MEMORY
    // Special RAM region for passing data to Hubris tasks
    let handoff = Handoff::turn_on(&peripherals.SYSCON);

    // 6. CONFIGURE HASHCRYPT INTERRUPT
    // Pre-main uses ROM signature verification
    set_hashcrypt_rom();

    // 7. VERIFY FLASH IMAGES
    // Get information about all flash banks
    let (slot_a, img_a) = images::Image::get_image_a(&mut flash, &peripherals.SYSCON);
    let (slot_b, img_b) = images::Image::get_image_b(&mut flash, &peripherals.SYSCON);

    // 8. DETERMINE WHICH IMAGE IS RUNNING
    // Use current PC to figure out if we're in slot A or B
    let here = startup as *const u8 as u32;
    let active = if slot_a.contains(&here) {
        RotSlot::A
    } else if slot_b.contains(&here) {
        RotSlot::B
    } else {
        panic!();
    };

    // 9. RUN DICE ATTESTATION (if enabled)
    #[cfg(any(feature = "dice-mfg", feature = "dice-self"))]
    {
        let slot = if active == RotSlot::A { &slot_a } else { &slot_b };
        dice::run(&handoff, peripherals, &mut flash, &slot.fwid());
    }

    // 10. NUKE THE STACK
    // Overwrite unused stack with zeros (security measure)
    nuke_stack();

    // 11. VERIFY PUF STATE (if DICE enabled)
    #[cfg(any(feature = "dice-mfg", feature = "dice-self"))]
    puf_check(&peripherals.PUF);

    // 12. GET STAGE0 IMAGES
    let (slot_stage0, img_stage0) =
        images::Image::get_image_stage0(&mut flash, &peripherals.SYSCON);
    let (slot_stage0next, img_stage0next) =
        images::Image::get_image_stage0next(&mut flash, &peripherals.SYSCON);

    // 13. SWITCH HASHCRYPT TO NORMAL MODE
    set_hashcrypt_default();

    // 14. WRITE BOOT STATE TO HANDOFF RAM
    let details = RotBootStateV2 {
        active,
        a: RotImageDetailsV2 {
            digest: slot_a.fwid(),
            status: img_a.map(|_| ()),
        },
        b: RotImageDetailsV2 {
            digest: slot_b.fwid(),
            status: img_b.map(|_| ()),
        },
        stage0: RotImageDetailsV2 {
            digest: slot_stage0.fwid(),
            status: img_stage0.map(|_| ()),
        },
        stage0next: RotImageDetailsV2 {
            digest: slot_stage0next.fwid(),
            status: img_stage0next.map(|_| ()),
        },
    };
    handoff.store(&details);

    // 15. ENABLE DEBUG (if beacon set)
    enable_debug(peripherals);
}
```

### MPU Configuration for USB RAM

**File**: `lib/lpc55-rot-startup/src/lib.rs` (lines 31-63)

```rust
fn apply_memory_protection(mpu: &MPU) {
    // Disable MPU temporarily
    unsafe { disable_mpu(mpu); }

    const USB_RAM_BASE: u32 = 0x4010_0000;
    const USB_RAM_SIZE: u32 = 0x4000;      // 16 KB
    const USB_RAM_REGION_NUMBER: u32 = 0;

    // Calculate limit address (inclusive, so -32)
    let rlar = (USB_RAM_BASE + USB_RAM_SIZE - 32) | (1 << 0);

    // Configure attributes
    let ap = 0b01;   // read-write by any privilege level
    let sh = 0b00;   // non-shareable
    let xn = 1;      // eXecute Never
    let rbar = USB_RAM_BASE | (sh as u32) << 3 | ap << 1 | xn;

    // Memory attribute: write-back transient, not shared
    let mair0 = 0b0111_0100;

    unsafe {
        mpu.rnr.write(USB_RAM_REGION_NUMBER);     // Select region 0
        mpu.mair[0].write(mair0);                 // Set attributes
        mpu.rbar.write(rbar);                     // Base address + flags
        mpu.rlar.write(rlar);                     // Limit address + enable

        enable_mpu(mpu, true);                    // Enable with privileged default
    }
}
```

**Why This Matters:**
- USB RAM is memory-mapped in device memory region by default
- Device memory doesn't allow unaligned access
- We need to treat it as normal RAM for the kernel
- This MPU region will be **replaced** when kernel starts

### Stack Nuking

**File**: `lib/lpc55-rot-startup/src/lib.rs` (lines 288-328)

```rust
#[naked]
extern "C" fn nuke_stack() {
    extern "C" {
        static _stack_base: u32;
    }

    // ARM uses "full descending" stack:
    // - SP points to valid data
    // - Stack grows down (towards lower addresses)
    //
    // Goal: Zero all memory from _stack_base to SP (exclusive)
    //
    // This is a security measure to prevent leaking secrets
    // that might have been on the stack during DICE operations

    core::arch::naked_asm!("
        ldr r0, ={stack_base}   @ Load stack base address
        mov r1, sp              @ Copy current SP
        mov r2, #0              @ Zero value
        mov r3, #0              @ Also zero r3

    0:  cmp r1, r0              @ Compare SP with base
        beq 1f                  @ If equal, done

        str r2, [r1, #-4]!      @ Store zero, decrement SP
        b 0b                    @ Loop

    1:  bx lr                   @ Return
        ",
        stack_base = sym _stack_base,
    )
}
```

**Security Context:**
After DICE attestation, the stack contains cryptographic secrets. This function overwrites the unused portion of the stack to ensure those secrets don't leak to Hubris tasks.

---

## Phase 4: Kernel Initialization

### start_kernel() Entry Point

**File**: `sys/kern/src/startup.rs` (lines 24-90)

```rust
pub unsafe fn start_kernel(tick_divisor: u32) -> ! {
    // 1. SET CLOCK FREQUENCY (for debuggers)
    unsafe {
        crate::arch::set_clock_freq(tick_divisor);
    }

    // 2. GET REFERENCES TO STATIC DATA
    // HUBRIS_TASK_DESCS: Flash-based task descriptors (generated at build time)
    let task_descs = &HUBRIS_TASK_DESCS;

    // HUBRIS_TASK_TABLE_SPACE: RAM space for task runtime state
    let task_table = unsafe {
        &mut *core::ptr::addr_of_mut!(HUBRIS_TASK_TABLE_SPACE)
    };

    // 3. INITIALIZE TASK TABLE
    // Convert MaybeUninit<[Task; N]> to [MaybeUninit<Task>; N]
    let task_table: &mut [MaybeUninit<Task>; HUBRIS_TASK_COUNT] =
        unsafe { &mut *(task_table as *mut _ as *mut _) };

    // For each task descriptor, create Task instance
    for (i, task) in task_table.iter_mut().enumerate() {
        task.write(Task::from_descriptor(&task_descs[i]));
    }

    // 4. ASSUME INITIALIZATION COMPLETE
    // Shed the MaybeUninit wrapper
    let task_table: &mut [Task; HUBRIS_TASK_COUNT] =
        unsafe { &mut *(task_table as *mut _ as *mut _) };

    // 5. REINITIALIZE ARCHITECTURE-SPECIFIC STATE
    // Set up initial register values, stack frames, etc.
    for task in task_table.iter_mut() {
        crate::arch::reinitialize(task);
    }

    // 6. SELECT FIRST TASK
    // Pick highest-priority task with START_AT_BOOT flag
    let first_task = crate::task::select(task_table.len() - 1, task_table);

    // 7. APPLY MEMORY PROTECTION FOR FIRST TASK
    crate::arch::apply_memory_protection(first_task);

    // 8. MARK TASK TABLE AS AVAILABLE
    TASK_TABLE_IN_USE.store(false, Ordering::Release);

    // 9. START FIRST TASK (never returns)
    crate::arch::start_first_task(tick_divisor, first_task)
}
```

### Auto-Generated Configuration

At the end of `startup.rs`:

```rust
include!(concat!(env!("OUT_DIR"), "/kconfig.rs"));
```

This includes a **build-time generated** file containing:

```rust
// Example of what kconfig.rs contains (generated from app.toml):

const HUBRIS_TASK_COUNT: usize = 13;  // Number of tasks

static HUBRIS_TASK_DESCS: [TaskDesc; HUBRIS_TASK_COUNT] = [
    TaskDesc {
        entry_point: 0x2000_0000,      // Task's main() address
        initial_stack: 0x2000_1000,     // Initial SP value
        memory_regions: &[0, 1, 2],     // MPU regions this task can access
        priority: 0,                     // Task priority (0 = highest)
        flags: TaskFlags::START_AT_BOOT, // Start immediately
    },
    // ... 12 more tasks
];

static mut HUBRIS_TASK_TABLE_SPACE: MaybeUninit<[Task; HUBRIS_TASK_COUNT]> =
    MaybeUninit::uninit();
```

**Build Process:**
1. `cargo xtask dist` reads `app/lpc55xpresso/app.toml`
2. Build system generates `kconfig.rs` with task descriptors
3. Linker places task code at specified addresses
4. Final binary has all tasks embedded

### Task Descriptor Structure

**File**: `sys/kern/src/descs.rs`

```rust
#[repr(C)]
pub struct TaskDesc {
    pub entry_point: u32,           // Address of task's main function
    pub initial_stack: u32,          // Initial stack pointer value
    pub memory_regions: &'static [usize],  // Indices into region table
    pub priority: u8,                // 0 = highest, 255 = lowest
    pub flags: TaskFlags,            // START_AT_BOOT, etc.
}

bitflags! {
    pub struct TaskFlags: u32 {
        const START_AT_BOOT = 1 << 0;   // Start this task immediately
        // Other flags...
    }
}
```

---

## Phase 5: Task Initialization

### Task::from_descriptor()

**File**: `sys/kern/src/task.rs` (lines 55-72)

```rust
pub fn from_descriptor(descriptor: &'static TaskDesc) -> Self {
    Task {
        // Copy priority from descriptor
        priority: Priority(descriptor.priority),

        // Set initial state based on START_AT_BOOT flag
        state: if descriptor.flags.contains(TaskFlags::START_AT_BOOT) {
            TaskState::Healthy(SchedState::Runnable)
        } else {
            TaskState::default()  // Stopped
        },

        // Save pointer to descriptor (for restarts)
        descriptor,

        // Initialize other fields to defaults
        generation: 0,
        timer: TimerState::default(),
        notifications: 0,
        save: SavedState::default(),
    }
}
```

**Task Runtime Structure:**

```rust
pub struct Task {
    priority: Priority,           // Scheduling priority
    state: TaskState,             // Healthy/Faulted + Runnable/Blocked
    timer: TimerState,            // Deadline for timed waits
    notifications: u32,           // Pending notification bits
    save: SavedState,             // Saved CPU registers
    generation: u32,              // Increments on restart
    descriptor: &'static TaskDesc, // Pointer to Flash descriptor
}
```

### arch::reinitialize()

**File**: `sys/kern/src/arch/arm_m.rs` (lines 279-341)

```rust
pub fn reinitialize(task: &mut task::Task) {
    // 1. CLEAR SAVED REGISTERS
    *task.save_mut() = SavedState::default();

    // 2. GET INITIAL STACK FROM DESCRIPTOR
    let initial_stack = task.descriptor().initial_stack as usize;

    // 3. VERIFY STACK ALIGNMENT
    // ARM requires 8-byte alignment
    uassert!(initial_stack & 0x7 == 0);

    // 4. CREATE EXCEPTION FRAME ON TASK STACK
    // When task first runs, hardware will pop this frame

    // Calculate frame location (stack grows down)
    let frame_addr = initial_stack - core::mem::size_of::<ExceptionFrame>();

    // Get mutable reference to stack memory
    let frame = unsafe {
        &mut *(frame_addr as *mut ExceptionFrame)
    };

    // 5. INITIALIZE EXCEPTION FRAME
    *frame = ExceptionFrame {
        r0: 0,                              // Argument 0
        r1: 0,                              // Argument 1
        r2: 0,                              // Argument 2
        r3: 0,                              // Argument 3
        r12: 0,                             // Scratch register
        lr: 0,                              // Link register (unused)
        pc: task.descriptor().entry_point,  // Task's main() function
        xpsr: 0x0100_0000,                  // Thumb bit set
    };

    // 6. SET INITIAL STACK POINTER
    // Points just below the exception frame
    task.save_mut().psp = frame_addr as u32;

    // 7. SET EXCEPTION RETURN VALUE
    // This magic value tells CPU to:
    // - Return to Thread mode
    // - Use Process Stack Pointer (PSP)
    // - Execute in unprivileged mode
    task.save_mut().exc_return = EXC_RETURN_CONST;  // 0xFFFFFFFD
}
```

### Exception Frame Structure

**File**: `sys/kern/src/arch/arm_m.rs`

```rust
#[repr(C)]
struct ExceptionFrame {
    // Hardware-saved registers (pushed/popped automatically)
    r0: u32,      // General purpose
    r1: u32,
    r2: u32,
    r3: u32,
    r12: u32,
    lr: u32,      // Return address
    pc: u32,      // Program counter (where to resume)
    xpsr: u32,    // Status register (must have Thumb bit set)
}
```

**Stack Layout After Initialization:**

```
High Address
┌──────────────────┐
│   Unused Stack   │
├──────────────────┤
│      xPSR        │ ← 0x0100_0000 (Thumb bit)
│       PC         │ ← Task entry point
│       LR         │ ← 0
│       R12        │ ← 0
│       R3         │ ← 0
│       R2         │ ← 0
│       R1         │ ← 0
│       R0         │ ← 0
├──────────────────┤ ← PSP points here
│   (will grow)    │
│      down        │
│      ...         │
└──────────────────┘
Low Address
```

---

## Phase 6: Starting the First Task

### Task Selection

**File**: `sys/kern/src/task.rs`

```rust
pub fn select(from: usize, tasks: &[Task]) -> &Task {
    // Start scanning from task after 'from'
    let start_index = from.wrapping_add(1);

    // Find highest-priority runnable task
    for i in 0..tasks.len() {
        let index = (start_index + i) % tasks.len();
        let task = &tasks[index];

        // Check if task is runnable
        if let TaskState::Healthy(SchedState::Runnable) = task.state {
            return task;
        }
    }

    // No runnable task found - should never happen
    panic!("No runnable task!");
}
```

**For LPC55xpresso:**

From `app/lpc55xpresso/app.toml`, tasks with `start = true`:

```toml
[tasks.jefe]
priority = 0          # Highest
start = true

[tasks.syscon_driver]
priority = 2
start = true

[tasks.gpio_driver]
priority = 3
start = true

# ... many more ...

[tasks.idle]
priority = 7          # Lowest
start = true
```

**Result:** Task `jefe` (priority 0) will be selected as the first task.

### start_first_task()

**File**: `sys/kern/src/arch/arm_m.rs` (lines 625-833)

```rust
pub fn start_first_task(tick_divisor: u32, task: &task::Task) -> ! {
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;

        // 1. ENABLE FAULT HANDLERS
        // ARMv8-M: Enable MemManage, BusFault, UsageFault, SecureFault
        scb.shcsr.modify(|x| x | 0b1111 << 16);

        // 2. SET EXCEPTION PRIORITIES
        // Faults: Priority 0 (highest)
        scb.shpr[0].write(0x00);  // MemManage
        scb.shpr[1].write(0x00);  // BusFault
        scb.shpr[2].write(0x00);  // UsageFault

        // Kernel entry points: Priority 0xFF (lowest)
        scb.shpr[7].write(0xFF);   // SVCall
        scb.shpr[10].write(0xFF);  // SysTick
        scb.shpr[11].write(0xFF);  // PendSV

        // 3. ENABLE DIVIDE-BY-ZERO TRAP
        const DIV_0_TRP: u32 = 1 << 4;
        scb.ccr.modify(|x| x | DIV_0_TRP);

        // 4. SET ALL EXTERNAL INTERRUPT PRIORITIES TO 0xFF
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        let icb = &*cortex_m::peripheral::ICB::PTR;
        let ictr = icb.ictr.read();
        let irq_count = ((ictr as usize & 0xF) + 1) * 32;

        for i in 0..irq_count {
            nvic.ipr[i].write(0xFFu8);
        }
    }

    unsafe {
        // 5. CONFIGURE SYSTICK TIMER
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(tick_divisor - 1);  // Reload value
        syst.cvr.write(0);                  // Clear current value
        syst.csr.modify(|v| v | 0b111);     // Enable counter + interrupt
    }

    // 6. ENABLE MPU
    let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
    const ENABLE: u32 = 0b001;
    const PRIVDEFENA: u32 = 0b100;  // Privileged default region
    unsafe {
        mpu.ctrl.write(ENABLE | PRIVDEFENA);
    }

    // 7. SET CURRENT TASK POINTER
    CURRENT_TASK_PTR.store(task as *const _ as *mut _, Ordering::Relaxed);

    // 8. SET MSP LIMIT (ARMv8-M only)
    #[cfg(armv8m)]
    unsafe {
        extern "C" { static _stack_base: u32; }
        cortex_m::register::msplim::write(
            core::ptr::addr_of!(_stack_base) as u32
        );
    }

    // 9. SET PROCESS STACK POINTER
    unsafe {
        cortex_m::register::psp::write(task.save().psp);
    }

    // 10. RESTORE CALLEE-SAVE REGISTERS AND TRAP INTO KERNEL
    #[cfg(any(armv7m, armv8m))]
    unsafe {
        arch::asm!("
            @ Restore callee-save registers (r4-r11)
            ldm {task}, {{r4-r11}}

            @ Trap into kernel with special SVC number
            svc #0xFF

            @ Should never reach here
            ",
            task = in(reg) &task.save().r4,
            options(noreturn),
        )
    }
}
```

**Register State Before SVC:**

```
r4-r11: Loaded from task.save() (all zeros initially)
PSP:    Points to task's exception frame
MSP:    Kernel stack
Mode:   Privileged Thread mode (CONTROL = 0x00)
Stack:  Using Main Stack (MSP)
```

---

## Phase 7: Transition to User Mode

### SVCall Exception Handler

**File**: `sys/kern/src/arch/arm_m.rs` (lines 935-1017)

```assembly
.section .text.SVCall
.globl SVCall
.type SVCall,function
SVCall:
    @ ────────────────────────────────────────────────────────
    @ STEP 1: Detect if this is kernel startup or syscall
    @ ────────────────────────────────────────────────────────
    @ On exception entry, LR contains EXC_RETURN value:
    @   0xFFFFFFF9 = Return to Handler mode (kernel startup)
    @   0xFFFFFFFD = Return to Thread mode using PSP (syscall)

    mov r0, lr
    ldr r1, =0xFFFFFFF9
    cmp r0, r1
    bne 1f              @ Not startup, handle as syscall

    @ ────────────────────────────────────────────────────────
    @ KERNEL STARTUP PATH
    @ ────────────────────────────────────────────────────────

    @ Switch to unprivileged Thread mode using PSP
    @ CONTROL register: [1] = use PSP, [0] = unprivileged
    mov r0, #0x03
    msr CONTROL, r0
    isb                 @ Instruction sync barrier

    @ Load task's exception return value
    ldr r0, =CURRENT_TASK_PTR
    ldr r0, [r0]
    ldr lr, [r0, #36]   @ Load exc_return from SavedState

    @ Return from exception
    @ This will:
    @   - Pop exception frame from PSP
    @   - Load PC with task entry point
    @   - Load xPSR with Thumb bit set
    @   - Switch to unprivileged mode
    @   - Start executing task code
    bx lr

1:  @ ────────────────────────────────────────────────────────
    @ SYSCALL PATH (not executed during startup)
    @ ────────────────────────────────────────────────────────
    @ ... syscall handling code ...
```

### What Happens on "bx lr"

The `bx lr` instruction returns from the exception. The processor performs these steps **automatically in hardware**:

1. **Read exception return value** from LR (0xFFFFFFFD)
2. **Determine stack pointer**: Use PSP (Process Stack Pointer)
3. **Pop exception frame** from PSP:
   ```
   Pop r0, r1, r2, r3, r12, lr, pc, xpsr
   ```
4. **Update PSP**: Increment by 32 bytes (8 registers × 4 bytes)
5. **Load PC**: Set program counter to task entry point
6. **Load xPSR**: Restore processor status (Thumb bit must be set)
7. **Enter Thread mode**: Exit Handler mode
8. **Continue execution** at task entry point in unprivileged mode

**Result:**

```
PC:      Task entry point (e.g., task-jefe main function)
Mode:    Unprivileged Thread mode
Stack:   Using PSP
Registers: r0-r3, r12 = 0 (from exception frame)
          r4-r11 = 0 (from saved state)
MPU:     Protecting task memory regions
```

---

## Memory Layout

### Physical Address Space

```
┌─────────────────────────────────────────────────────────────┐
│ 0x0000_0000 - 0x0009_FFFF  Flash (640 KB)                   │
│   ├─ 0x0000_0000: Vector table                              │
│   ├─ 0x0000_xxxx: Kernel code                               │
│   ├─ 0x0001_xxxx: Task code (jefe)                          │
│   ├─ 0x0002_xxxx: Task code (syscon_driver)                 │
│   └─ ...                                                    │
├─────────────────────────────────────────────────────────────┤
│ 0x2000_0000 - 0x2004_3FFF  SRAM (272 KB)                    │
│   ├─ 0x2000_0000: Kernel .data + .bss                       │
│   ├─ 0x2000_1000: Kernel stack                              │
│   ├─ 0x2000_2000: Task RAM (jefe)                           │
│   ├─ 0x2000_3000: Task RAM (syscon_driver)                  │
│   └─ ...                                                    │
├─────────────────────────────────────────────────────────────┤
│ 0x4000_0000 - 0x5FFF_FFFF  Peripherals                      │
│   ├─ 0x4000_0000: GPIO                                      │
│   ├─ 0x4008_0000: USART                                     │
│   ├─ 0x5000_0000: SYSCON                                    │
│   └─ ...                                                    │
├─────────────────────────────────────────────────────────────┤
│ 0xE000_0000 - 0xE00F_FFFF  System (CPU internal)            │
│   ├─ 0xE000_E000: SysTick                                   │
│   ├─ 0xE000_ED00: MPU                                       │
│   └─ 0xE000_EF00: NVIC                                      │
└─────────────────────────────────────────────────────────────┘
```

### Task Memory Regions (Example: jefe)

From MPU perspective, task `jefe` can access:

```
Region 0: Flash code
  Base:  0x0001_0000
  Size:  16 KB
  Attrs: Read + Execute (no write)

Region 1: RAM data
  Base:  0x2000_2000
  Size:  4 KB
  Attrs: Read + Write (no execute)

Region 2: Stack
  Base:  0x2000_3000
  Size:  2 KB
  Attrs: Read + Write (no execute)

Region 3: SYSCON peripheral
  Base:  0x5000_0000
  Size:  4 KB
  Attrs: Device memory, Read + Write
```

**Any access outside these regions** → MemManage fault → Task restart

---

## Visual Summary

### Complete Boot Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    POWER ON / RESET                         │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
              ╔════════════════╗
              ║   HARDWARE     ║
              ║   (Cortex-M33) ║
              ╚════════╤═══════╝
                       │
                       ├─► Load SP from 0x0000_0000
                       ├─► Load PC from 0x0000_0004
                       └─► Jump to Reset Handler
                       │
                       ▼
              ╔════════════════╗
              ║  cortex_m_rt   ║
              ║ Reset Handler  ║
              ╚════════╤═══════╝
                       │
                       ├─► Copy .data (Flash → RAM)
                       ├─► Zero .bss
                       ├─► Enable FPU
                       └─► Call main()
                       │
                       ▼
              ╔════════════════════════╗
              ║ app/lpc55xpresso/main  ║
              ╚════════╤═══════════════╝
                       │
                       ├─► Take peripheral instances
                       ├─► Get clock speed
                       ├─► Call lpc55_rot_startup::startup()
                       │   │
                       │   ├─► Verify ROM version
                       │   ├─► Configure MPU for USB RAM
                       │   ├─► Verify flash images
                       │   ├─► Run DICE attestation
                       │   ├─► Nuke stack
                       │   ├─► Write boot state to handoff RAM
                       │   └─► Enable debug
                       │
                       └─► Call kern::startup::start_kernel()
                       │
                       ▼
              ╔════════════════════╗
              ║  start_kernel()    ║
              ╚════════╤═══════════╝
                       │
                       ├─► Set clock frequency
                       ├─► Read HUBRIS_TASK_DESCS from Flash
                       ├─► Initialize task table in RAM
                       ├─► For each task:
                       │   ├─► Task::from_descriptor()
                       │   └─► arch::reinitialize()
                       ├─► Select first task (jefe, priority 0)
                       ├─► Apply MPU for first task
                       └─► Call start_first_task()
                       │
                       ▼
              ╔════════════════════╗
              ║ start_first_task() ║
              ╚════════╤═══════════╝
                       │
                       ├─► Enable fault handlers
                       ├─► Set exception priorities
                       ├─► Configure SysTick timer
                       ├─► Enable MPU
                       ├─► Set CURRENT_TASK_PTR
                       ├─► Set PSP to task stack
                       ├─► Restore r4-r11 registers
                       └─► Execute SVC #0xFF
                       │
                       ▼
              ╔════════════════════╗
              ║  SVCall Handler    ║
              ╚════════╤═══════════╝
                       │
                       ├─► Detect startup mode (LR = 0xFFFFFFF9)
                       ├─► Set CONTROL = 0x03 (unprivileged + PSP)
                       ├─► Load task's exc_return value
                       └─► Return from exception (bx lr)
                       │
                       ▼         Hardware automatically:
              ╔════════════════════╗
              ║    HARDWARE        ║  ├─► Pop exception frame from PSP
              ║  Exception Return  ║  ├─► Load PC with task entry point
              ╚════════╤═══════════╝  ├─► Load xPSR
                       │               └─► Enter unprivileged mode
                       ▼
              ╔════════════════════╗
              ║   TASK RUNNING     ║
              ║   (e.g., jefe)     ║
              ╚════════════════════╝
                       │
                       ├─► Unprivileged Thread mode
                       ├─► Using PSP
                       ├─► MPU protecting memory
                       └─► Can make syscalls via SVC
```

### State Transitions

```
┌──────────────────────────┐
│  Before start_kernel()   │
│                          │
│  Mode:  Privileged       │
│  Stack: MSP (kernel)     │
│  MPU:   Minimal config   │
└────────────┬─────────────┘
             │
             │ start_kernel()
             │ initializes tasks
             │
             ▼
┌──────────────────────────┐
│  Before start_first_task │
│                          │
│  Mode:  Privileged       │
│  Stack: MSP              │
│  Tasks: Initialized      │
│  MPU:   Task 0 regions   │
└────────────┬─────────────┘
             │
             │ Restore regs
             │ SVC #0xFF
             │
             ▼
┌──────────────────────────┐
│  In SVCall handler       │
│                          │
│  Mode:  Privileged       │
│  Stack: MSP              │
│  About to switch...      │
└────────────┬─────────────┘
             │
             │ Set CONTROL = 0x03
             │ bx lr (exc_return)
             │
             ▼
┌──────────────────────────┐
│  Task Running!           │
│                          │
│  Mode:  UNPRIVILEGED     │
│  Stack: PSP (task)       │
│  PC:    Task entry point │
│  MPU:   Task regions     │
└──────────────────────────┘
```

---

## Key Concepts Summary

### 1. Exception Return Values

ARM Cortex-M uses special "magic" values in LR during exceptions:

```
0xFFFFFFF1 = Return to Handler mode using MSP
0xFFFFFFF9 = Return to Thread mode using MSP (kernel startup uses this)
0xFFFFFFFD = Return to Thread mode using PSP (syscalls use this)
0xFFFFFFE1 = Return to Handler mode using MSP (with FP state)
0xFFFFFFE9 = Return to Thread mode using MSP (with FP state)
0xFFFFFFED = Return to Thread mode using PSP (with FP state)
```

### 2. Stack Pointers

ARM Cortex-M has **two** stack pointers:

- **MSP (Main Stack Pointer)**: Used by kernel and interrupt handlers
- **PSP (Process Stack Pointer)**: Used by tasks

The `CONTROL` register selects which one:
- `CONTROL[1] = 0`: Use MSP
- `CONTROL[1] = 1`: Use PSP

### 3. Privilege Levels

- **Privileged**: Can execute all instructions, access all memory
- **Unprivileged**: Limited instructions, MPU enforces memory access

The `CONTROL` register controls this:
- `CONTROL[0] = 0`: Privileged
- `CONTROL[0] = 1`: Unprivileged

**During startup:**
```
CONTROL = 0x00  →  Privileged + MSP (kernel)
CONTROL = 0x03  →  Unprivileged + PSP (tasks)
```

### 4. Memory Protection Unit (MPU)

The MPU enforces memory access rules for unprivileged code:

- Configured by kernel for each task
- Defines up to 8 regions (ARMv8-M can use more)
- Each region has: base address, size, attributes (R/W/X)
- Any violation → MemManage fault → Task restart

### 5. Exception Frame

When an exception occurs, hardware automatically pushes registers:

```
Stack before exception:
┌─────┐
│ ... │
└─────┘ ← SP

Exception occurs, hardware pushes:
┌─────┐
│xPSR │  ← Pushed 8th
│ PC  │  ← Pushed 7th (return address)
│ LR  │
│ R12 │
│ R3  │
│ R2  │
│ R1  │
│ R0  │  ← Pushed 1st
├─────┤ ← SP (after push)
│ ... │
└─────┘
```

On return (`bx lr`), hardware automatically pops these registers.

---

## Conclusion

The Hubris startup sequence is a carefully orchestrated dance between:

1. **Hardware** (Cortex-M33 reset behavior)
2. **cortex_m_rt** (Rust runtime initialization)
3. **Application-specific setup** (LPC55 security, clocks, peripherals)
4. **Kernel initialization** (task table, MPU, timers)
5. **Architecture-specific transition** (switching to unprivileged mode)

By the time the first task starts running:
- All tasks are initialized in RAM
- The MPU is configured for the first task
- The SysTick timer is running
- The kernel is ready to handle syscalls and context switches
- The task is running in **unprivileged mode** with **memory protection**

This design ensures:
- **Safety**: Tasks can't corrupt kernel or other tasks
- **Isolation**: Faults are contained to the faulting task
- **Determinism**: Fixed priority scheduling with no surprises
- **Debuggability**: Physical addressing makes debugging simple

Understanding this boot sequence is crucial for:
- Debugging startup failures
- Adding new boards/processors
- Implementing custom startup hooks
- Understanding the security model
- Optimizing boot time

---

## Further Reading

**Code Files Referenced:**
- `app/lpc55xpresso/src/main.rs` - Application entry point
- `lib/lpc55-rot-startup/src/lib.rs` - LPC55-specific startup
- `sys/kern/src/startup.rs` - Kernel initialization
- `sys/kern/src/task.rs` - Task management
- `sys/kern/src/arch/arm_m.rs` - ARM architecture-specific code
- `sys/kern/src/descs.rs` - Descriptor structures

**Documentation:**
- `doc/startup.adoc` - Startup overview
- `doc/tasks.adoc` - Task system details
- ARM Cortex-M33 Technical Reference Manual
- LPC55S69 User Manual (UM11126)

**Build System:**
- `build/xtask/` - Build tool implementation
- `build/kconfig/` - Configuration generation
- `build/kernel-link.x` - Kernel linker script
- `build/task-link.x` - Task linker script
