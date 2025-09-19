// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Architecture support for RISC-V.
//!
//! Currently only the hazard3 core as found on the RP Pico 2 and RP Pico 2 W boards
//! is supported, but adding support for other cores should be straight-forward.
//!
//! As the hazard3 core does not support it there is no Supervisor mode support,
//! the kernel runs exclusively in Machine mode with tasks running in User mode.
//!
//! Interrupts (other than the Machine Timer used to advance the kernel timestamp)
//! are not yet supported.

use core::arch;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use zerocopy::FromBytes;

use crate::atomic::AtomicExt;
use crate::descs::RegionAttributes;
use crate::startup::with_task_table;
use crate::task;
use crate::time::Timestamp;

use abi::{FaultInfo, FaultSource, InterruptNum, UsageError};

use riscv::interrupt::Exception::*;
use riscv::interrupt::Interrupt::*;
use riscv::interrupt::{Exception, Interrupt, Trap};
use riscv::register::{self, mcause, mstatus::MPP};

/// Log things from kernel context. This macro is made visible to the rest of
/// the kernel by a chain of `#[macro_use]` attributes, but its implementation
/// is very architecture-specific at the moment.
///
/// On RISC-V we are limited to semihosting, despite its shortcomings.
///
/// In the future, we will likely want to add at least one more mechanism for
/// logging (one that can be presumably be made neutral with respect to
/// architecure), whereby kernel logs can be produced somewhere (e.g., a ring
/// buffer) from which they can be consumed by some entity for shipping
/// elsewhere.
///
#[macro_export]
macro_rules! klog {
    ($s:expr) => {
        let _ = riscv_semihosting::hprintln!($s);
    };
    ($s:expr, $($tt:tt)*) => {
        let _ = riscv_semihosting::hprintln!($s, $($tt)*);
    };
}

#[macro_export]
#[allow(unused_macros)]
macro_rules! kdbg {
    ($s:expr) => {
        let _ = riscv_semihosting::dbg!($s);
    };
    ($s:expr, $($tt:tt)*) => {
        let _ = riscv_semihosting::dbg!($s, $($tt)*);
    };
}

#[macro_export]
macro_rules! uassert {
    ($cond : expr) => {
        if !$cond {
            panic!("Assertion failed!");
        }
    };
}

/// On RISC-V we use a global to record the current task pointer.  
/// It's possible to use the mscratch register instead. (TODO: Future Work)
#[no_mangle]
static CURRENT_TASK_PTR: AtomicPtr<task::Task> =
    AtomicPtr::new(core::ptr::null_mut());

/// To allow our clock frequency to be easily determined from a debugger, we
/// store it in memory.
#[no_mangle]
static CLOCK_FREQ_KHZ: AtomicU32 = AtomicU32::new(0);

/// RISC-V volatile registers that must be saved across context switches.
#[repr(C)]
#[derive(Clone, Debug, Default, FromBytes)]
pub struct SavedState {
    // NOTE: the following fields must be kept contiguous!
    ra: u32,
    sp: u32,
    gp: u32,
    tp: u32,
    t0: u32,
    t1: u32,
    t2: u32,
    s0: u32, // or fp -> Saved register or frame pointer
    s1: u32,
    a0: u32,
    a1: u32,
    a2: u32,
    a3: u32,
    a4: u32,
    a5: u32,
    a6: u32,
    a7: u32,
    s2: u32,
    s3: u32,
    s4: u32,
    s5: u32,
    s6: u32,
    s7: u32,
    s8: u32,
    s9: u32,
    s10: u32,
    s11: u32,
    t3: u32,
    t4: u32,
    t5: u32,
    t6: u32,
    pc: u32, // Additional save value for task program counter
}

/// Map the volatile registers to (architecture-independent) syscall argument
/// and return slots.
impl task::ArchState for SavedState {
    fn stack_pointer(&self) -> u32 {
        self.sp
    }

    /// Reads syscall argument register 0.
    fn arg0(&self) -> u32 {
        self.a0
    }
    fn arg1(&self) -> u32 {
        self.a1
    }
    fn arg2(&self) -> u32 {
        self.a2
    }
    fn arg3(&self) -> u32 {
        self.a3
    }
    fn arg4(&self) -> u32 {
        self.a4
    }
    fn arg5(&self) -> u32 {
        self.a5
    }
    fn arg6(&self) -> u32 {
        self.a6
    }

    fn syscall_descriptor(&self) -> u32 {
        self.a7
    }

    /// Writes syscall return argument 0.
    fn ret0(&mut self, x: u32) {
        self.a0 = x
    }
    fn ret1(&mut self, x: u32) {
        self.a1 = x
    }
    fn ret2(&mut self, x: u32) {
        self.a2 = x
    }
    fn ret3(&mut self, x: u32) {
        self.a3 = x
    }
    fn ret4(&mut self, x: u32) {
        self.a4 = x
    }
    fn ret5(&mut self, x: u32) {
        self.a5 = x
    }
}

/// Sets the kernel's notion of our clock frequency measured in kHz.
///
/// This is used for two broad classes of things. First, it's used to initialize
/// and manage the SysTick timer that provides the kernel timer interrupt.
///
/// Second, it's read by debuggers, which often need to know the clock frequency
/// to properly initialize ITM, if you're using SWO.
///
/// This is called quite early in boot by the architecture-independent startup
/// code, and should only be called once.
///
/// # Safety
///
/// This is safe to call once, early in boot, before `start_first_task`. It
/// should not be called again.
pub unsafe fn set_clock_freq(tick_divisor: u32) {
    CLOCK_FREQ_KHZ.store(tick_divisor, Ordering::Relaxed);
}

pub fn reinitialize(task: &mut task::Task) {
    *task.save_mut() = SavedState::default();

    // Set the initial stack pointer
    task.save_mut().sp = task.descriptor().initial_stack;

    // RISC-V machines require 16-byte stack alignment. Make sure that's
    // still true. Note that this carries the risk of panic on task re-init if
    // the task table is corrupted -- this is deliberate.
    uassert!(task.save().sp & 0xF == 0);

    // Set the initial program counter
    task.save_mut().pc = task.descriptor().entry_point;
}

/// Log PMP configuration and address registers
pub fn log_pmp_registers() {
    klog!("[KERN]: ++++++ PMP Register Status ++++++");

    // Log configuration registers
    klog!(
        "[KERN]: pmpcfg0: 0x{:08x}, pmpcfg1: 0x{:08x}, pmpcfg2: 0x{:08x}",
        register::pmpcfg0::read().bits,
        register::pmpcfg1::read().bits,
        register::pmpcfg2::read().bits
    );

    // Log all address registers
    klog!(
        "[KERN]: pmpaddr0: 0x{:08x}, pmpaddr1: 0x{:08x}",
        register::pmpaddr0::read(),
        register::pmpaddr1::read()
    );
    klog!(
        "[KERN]: pmpaddr2: 0x{:08x}, pmpaddr3: 0x{:08x}",
        register::pmpaddr2::read(),
        register::pmpaddr3::read()
    );
    klog!(
        "[KERN]: pmpaddr4: 0x{:08x}, pmpaddr5: 0x{:08x}",
        register::pmpaddr4::read(),
        register::pmpaddr5::read()
    );
    klog!(
        "[KERN]: pmpaddr6: 0x{:08x}, pmpaddr7: 0x{:08x}",
        register::pmpaddr6::read(),
        register::pmpaddr7::read()
    );
    klog!(
        "[KERN]: pmpaddr8: 0x{:08x}, pmpaddr9: 0x{:08x}, pmpaddr10: 0x{:08x}",
        register::pmpaddr8::read(),
        register::pmpaddr9::read(),
        register::pmpaddr10::read()
    );

    klog!("[KERN]: ++++++ End PMP Register Status ++++++");
}

/// RISCV PMP precomputed region data.
///
/// This struct is `repr(C)` to preserve the order of its fields, which happens
/// to match the order of registers in the MPU. While we don't bit-copy the
/// struct directly, this does improve code generation in practice.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct RegionDescExt {
    /// PMP configuration byte (R/W/X/A).
    pmpcfg: u8,
    /// PMPADDR register value (NAPOT encoded).
    pmpaddr: u32,
}

/// Decode PMPADDR register back to (base, size)
fn decode_pmpaddr(pmpaddr: u32) -> (u32, u32) {
    // Count trailing ones in pmpaddr
    let n = pmpaddr.trailing_ones();

    // Region size = 8 * 2^(n)
    let region_size = 8 * 2_u32.pow(n);

    // Base address = (pmpaddr & !((1 << n) - 1)) << 2
    // (zero out trailing ones, then shift back by 2)
    let base_addr = (pmpaddr & !((1 << n) - 1)) << 2;

    (base_addr, region_size)
}

pub const fn compute_region_extension_data(
    base: u32,
    size: u32,
    attributes: RegionAttributes,
) -> RegionDescExt {
    // Must be power of two and base aligned to size, and minimum 32 bytes
    if size < 32 || (size & (size - 1)) != 0 || (base & (size - 1)) != 0 {
        panic!();
    }

    let r = attributes.contains(RegionAttributes::READ) as u8;
    let w = attributes.contains(RegionAttributes::WRITE) as u8;
    let x = attributes.contains(RegionAttributes::EXECUTE) as u8;

    // Proper NAPOT encoding: (copied from `tockos`)
    let pmpaddr: u32 = (base + (size - 1).overflowing_shr(1).0)
        .overflowing_shr(2)
        .0;

    // pmpcfg: bits [0]=X, [1]=W, [2]=R, [4:3]=A (0b11 -> NAPOT)
    // Lock → bit 7 Do not set lock bit bcz hart reset is necessary for next PMP register writes
    let pmpcfg = (x << 0) | (w << 1) | (r << 2) | (0b11 << 3);

    RegionDescExt { pmpcfg, pmpaddr }
}

/// Apply memory protection for a task
pub fn apply_memory_protection(task: &task::Task) {
    klog!(
        "[KERN]: ----- apply_memory_protection() for TASK [{}] -----",
        task.descriptor().index
    );

    unsafe {
        for (i, region) in task.region_table().iter().enumerate().take(8) {
            let pmpcfg = region.arch_data.pmpcfg as usize;
            let pmpaddr = region.arch_data.pmpaddr as usize;

            klog!(
                "[KERN]: Region {}: Preparing pmpaddr=0x{:08x}, pmpcfg=0x{:02x}",
                i,
                pmpaddr,
                pmpcfg
            );

            // --- Step 1: read containing CSR (pmpcfg0 or pmpcfg1) ---
            let cfg_val = if i < 4 {
                register::pmpcfg0::read().bits
            } else {
                register::pmpcfg1::read().bits
            };

            let shift = (i % 4) * 8;
            let slot_mask = 0xFF << shift;
            let slot_val = ((cfg_val >> shift) & 0xFF) as u8;

            // --- Step 2: check lock bit ---
            if (slot_val & (1 << 7)) != 0 {
                panic!("[KERN]: PMP region {} locked!", i);
            }

            // --- Step 3: verify HW backing by flipping R/W/X ---
            let flipped_cfg = cfg_val ^ (0x07 << shift);
            if i < 4 {
                register::pmpcfg0::write(flipped_cfg);
                if register::pmpcfg0::read().bits == cfg_val {
                    panic!("[KERN]: PMP region {} not backed by HW!", i);
                }
            } else {
                register::pmpcfg1::write(flipped_cfg);
                if register::pmpcfg1::read().bits == cfg_val {
                    panic!("[KERN]: PMP region {} not backed by HW!", i);
                }
            }

            // --- Step 4: disable region (clear A field bits [4:3]) ---
            let mut new_cfg = cfg_val & !(0x18 << shift);
            if i < 4 {
                register::pmpcfg0::write(new_cfg);
            } else {
                register::pmpcfg1::write(new_cfg);
            }

            // --- Step 5: write new address ---
            match i {
                0 => register::pmpaddr0::write(pmpaddr),
                1 => register::pmpaddr1::write(pmpaddr),
                2 => register::pmpaddr2::write(pmpaddr),
                3 => register::pmpaddr3::write(pmpaddr),
                4 => register::pmpaddr4::write(pmpaddr),
                5 => register::pmpaddr5::write(pmpaddr),
                6 => register::pmpaddr6::write(pmpaddr),
                7 => register::pmpaddr7::write(pmpaddr),
                _ => unreachable!(),
            }

            // --- Step 6: write final config into slot ---
            new_cfg = (new_cfg & !slot_mask) | (pmpcfg << shift);
            if i < 4 {
                register::pmpcfg0::write(new_cfg);
            } else {
                register::pmpcfg1::write(new_cfg);
            }

            // --- Optional: readback decode ---
            let readback = match i {
                0 => register::pmpaddr0::read(),
                1 => register::pmpaddr1::read(),
                2 => register::pmpaddr2::read(),
                3 => register::pmpaddr3::read(),
                4 => register::pmpaddr4::read(),
                5 => register::pmpaddr5::read(),
                6 => register::pmpaddr6::read(),
                7 => register::pmpaddr7::read(),
                _ => 0,
            } | 0b11;
            let (decoded_base, decoded_size) = decode_pmpaddr(readback as u32);

            klog!(
                "[KERN]: Region {} programmed -> base=0x{:08x}, size={}",
                i,
                decoded_base,
                decoded_size
            );
        }
    }

    klog!("[KERN]: ----- apply_memory_protection() DONE -----");
    log_pmp_registers();
}

pub fn start_first_task(tick_divisor: u32, task: &task::Task) -> ! {
    let mstatus = register::mstatus::read();
    klog!("[KERN]: Starting first task");
    // Configure MPP to switch us to User mode on exit from Machine
    // mode (when we call "mret" below).
    unsafe {
        register::mstatus::set_mpp(MPP::User);

        // Configure the timer
        // Write the initial task program counter.
        register::mepc::write(task.save().pc as *const usize as usize);

        // Reset mtime back to 0, set mtimecmp to chosen timer
        set_timer(tick_divisor - 1);

        // Machine timer interrupt enable
        register::mie::set_mtimer();

        // Global machine interrupt enable
        register::mstatus::set_mie();

        // Load first task pointer, set its initial stack pointer, and exit out
        // of machine mode, launching the task.
        CURRENT_TASK_PTR.store(task as *const _ as *mut _, Ordering::Relaxed);
        arch::asm!("
            lw sp, ({sp})
            # --- Workaround for RP2350-E7 ---
            # if returning to U-mode (mstatus.MPP == 0) set mstatus.MPIE (bit 7)
            addi    t0, x0, 128    # t0 = 1 << 7 (MPIE mask)
            csrrs   x0, mstatus, t0   # set mstatus.MPIE  (CSRRS with rd = x0 discards the read)
            mret",
            sp = in(reg) &task.save().sp,
            options(noreturn)
        );
    }
}

/// Records the address of `task` as the current user task.
///
/// # Safety
///
/// This records a pointer that aliases `task`. As long as you don't read that
/// pointer while you have access to `task`, and as long as the `task` being
/// stored is actually in the task table, you'll be okay.
pub unsafe fn set_current_task(task: &task::Task) {
    CURRENT_TASK_PTR.store(task as *const _ as *mut _, Ordering::Relaxed);
    crate::profiling::event_context_switch(task.descriptor().index as usize);
}

/// Reads the tick counter.
pub fn now() -> Timestamp {
    // Recall that we expect the systick interrupt cannot preempt kernel code,
    // so we're safe to read this in two nonatomic parts here.
    Timestamp::from([
        TICKS[0].load(Ordering::Relaxed),
        TICKS[1].load(Ordering::Relaxed),
    ])
}

/// Kernel global for tracking the current timestamp, measured in ticks.
///
/// This is a pair of `AtomicU32` because (1) we want the interior mutability of
/// the atomic types but (2) ARMv7-M doesn't have any 64-bit atomic operations.
/// We access this only from contexts where we can't be preempted, so, the fact
/// that it's split across two words is ok.
///
/// `TICKS[0]` is the least significant part, `TICKS[1]` the most significant.
static TICKS: [AtomicU32; 2] = {
    #[allow(clippy::declare_interior_mutable_const)]
    const ZERO: AtomicU32 = AtomicU32::new(0);
    [ZERO; 2]
};

pub fn disable_irq(
    _n: u32,
    _also_clear_pending: bool,
) -> Result<(), UsageError> {
    Ok(())
}

pub fn enable_irq(
    _n: u32,
    _also_clear_pending: bool,
) -> Result<(), UsageError> {
    Ok(())
}

/// Looks up an interrupt and returns a cross-platform
/// representation of that interrupt's status.
pub fn irq_status(_n: u32) -> Result<abi::IrqStatus, UsageError> {
    let status = abi::IrqStatus::empty();

    Ok(status)
}

pub fn pend_software_irq(
    InterruptNum(_n): InterruptNum,
) -> Result<(), UsageError> {
    // stub
    Ok(())
}

/// Rust entry point for fault.
///
/// # Safety
///
/// In brief: don't call this. This is an implementation factor of the fault
/// handler assembly code and should not be used for other purposes.
#[no_mangle]
unsafe extern "C" fn handle_fault(task: *mut task::Task, fault: FaultInfo) {
    with_task_table(|tasks| {
        let idx = (task as usize - tasks.as_ptr() as usize)
            / core::mem::size_of::<task::Task>();

        let next = match task::force_fault(tasks, idx, fault) {
            task::NextTask::Specific(i) => &tasks[i],
            task::NextTask::Other => task::select(idx, tasks),
            task::NextTask::Same => &tasks[idx],
        };

        if core::ptr::eq(next as *const _, task as *const _) {
            panic!("attempt to return to Task #{idx} after fault");
        }

        apply_memory_protection(next);
        // Safety: next comes from the task table and we don't use it again
        // until next kernel entry, so we meet set_current_task's requirements.
        unsafe {
            set_current_task(next);
        }
    });
    klog!(
        "[KERN]: Handled fault {:?} which had program_counter: {:#010x}",
        fault,
        unsafe { &*task }.save().pc
    );
}

pub fn reset() -> ! {
    klog!("[KERN]: Resetting CPU");
    loop {
        riscv::asm::wfi();
    }
}

impl AtomicExt for AtomicBool {
    type Primitive = bool;

    #[inline(always)]
    fn swap_polyfill(
        &self,
        value: Self::Primitive,
        ordering: Ordering,
    ) -> Self::Primitive {
        self.swap(value, ordering)
    }
}

// Provide our own interrupt vector to handle save/restore of the task on
// entry, overwriting the symbol set up by riscv-rt.  The (align(4)) is
// necessary as the bottom bits are used to determine direct or vectored traps.
//
// We may want to switch to a vectored interrupt table at some point to improve
// performance.
#[unsafe(naked)]
#[link_section = ".trap.rust"]
#[export_name = "_start_trap"]
pub unsafe extern "C" fn _start_trap() {
    arch::naked_asm!(
        "
        .align 4
        #
        # Store full task status on entry, setting up a0 to point at our
        # current task so that it's passed into our exception handler.
        #
        csrw mscratch, a0
        la a0, CURRENT_TASK_PTR
        lw a0, (a0)
        sw ra,   0*4(a0)
        sw sp,   1*4(a0)
        sw gp,   2*4(a0)
        sw tp,   3*4(a0)
        sw t0,   4*4(a0)
        sw t1,   5*4(a0)
        sw t2,   6*4(a0)
        sw s0,   7*4(a0)
        sw s1,   8*4(a0)
        #sw a0,  9*4(a0)
        sw a1,  10*4(a0)
        sw a2,  11*4(a0)
        sw a3,  12*4(a0)
        sw a4,  13*4(a0)
        sw a5,  14*4(a0)
        sw a6,  15*4(a0)
        sw a7,  16*4(a0)
        sw s2,  17*4(a0)
        sw s3,  18*4(a0)
        sw s4,  19*4(a0)
        sw s5,  20*4(a0)
        sw s6,  21*4(a0)
        sw s7,  22*4(a0)
        sw s8,  23*4(a0)
        sw s9,  24*4(a0)
        sw s10, 25*4(a0)
        sw s11, 26*4(a0)
        sw t3,  27*4(a0)
        sw t4,  28*4(a0)
        sw t5,  29*4(a0)
        sw t6,  30*4(a0)
        csrr a1, mepc
        sw a1,  31*4(a0)    # store mepc for resume
        csrr a1, mscratch
        sw a1, 9*4(a0)      # store a0 itself

        #
        # Jump to our main rust handler
        #
        jal trap_handler

        #
        # On the way out we may have switched to a different task, load
        # everything in and resume (using t6 as it's resored last).
        #
        la t6, CURRENT_TASK_PTR
        lw t6, (t6)

        lw t5,  31*4(t6)     # restore mepc
        csrw mepc, t5

        lw ra,   0*4(t6)
        lw sp,   1*4(t6)
        lw gp,   2*4(t6)
        lw tp,   3*4(t6)
        lw t0,   4*4(t6)
        lw t1,   5*4(t6)
        lw t2,   6*4(t6)
        lw s0,   7*4(t6)
        lw s1,   8*4(t6)
        lw a0,   9*4(t6)
        lw a1,  10*4(t6)
        lw a2,  11*4(t6)
        lw a3,  12*4(t6)
        lw a4,  13*4(t6)
        lw a5,  14*4(t6)
        lw a6,  15*4(t6)
        lw a7,  16*4(t6)
        lw s2,  17*4(t6)
        lw s3,  18*4(t6)
        lw s4,  19*4(t6)
        lw s5,  20*4(t6)
        lw s6,  21*4(t6)
        lw s7,  22*4(t6)
        lw s8,  23*4(t6)
        lw s9,  24*4(t6)
        lw s10, 25*4(t6)
        lw s11, 26*4(t6)
        lw t3,  27*4(t6)
        lw t4,  28*4(t6)
        lw t5,  29*4(t6)
        lw t6,  30*4(t6)

        mret
        "
    );
}

//
// The Rust side of our trap handler after the task's registers have been
// saved to SavedState.
//
#[no_mangle]
fn trap_handler(task: &mut task::Task) {
    let desc = task.descriptor();
    let index = desc.index as usize;
    let entry_point = desc.entry_point as usize;
    let initial_stack = desc.initial_stack as usize;
    let raw_trap: Trap<usize, usize> = mcause::read().cause();
    let standard_trap: Trap<Interrupt, Exception> =
        raw_trap.try_into().unwrap();
    klog!(
        "[KERN]: ****** trap_handler() MCAUSE: [0x{:08x}] ******",
        register::mcause::read().bits() as u32
    );

    match standard_trap {
        //
        // Interrupts.  Only our periodic MachineTimer interrupt via mtime is
        // supported at present.
        //
        Trap::Interrupt(MachineTimer) => {
            klog!("[KERN]: Trap::Interrupt(MachineTimer)");
            with_task_table(|tasks| safe_timer_handler(tasks))
        }
        //
        // System Calls.
        //
        Trap::Exception(UserEnvCall) => {
            klog!(
                "[KERN]: Trap::Exception(UserEnvCall) from TASK [{}]",
                task.descriptor().index
            );
            unsafe {
                // Advance program counter past ecall instruction.
                task.save_mut().pc = register::mepc::read() as u32 + 4;
                arch::asm!(
                    "
                    la a1, CURRENT_TASK_PTR
                    mv a0, a7               # arg0 = syscall number
                    lw a1, (a1)             # arg1 = task ptr
                    jal syscall_entry
                    ",
                );
            }
        }
        //
        // Exceptions.  Routed via the most appropriate FaultInfo.
        //
        Trap::Exception(IllegalInstruction) => unsafe {
            klog!("[KERN]: Trap::Exception(IllegalInstruction)");
            handle_fault(task, FaultInfo::IllegalInstruction);
        },
        Trap::Exception(LoadFault) => unsafe {
            klog!("[KERN]: Trap::Exception(LoadFault)");
            handle_fault(
                task,
                FaultInfo::MemoryAccess {
                    address: Some(register::mtval::read() as u32),
                    source: FaultSource::User,
                },
            );
        },
        Trap::Exception(StoreFault) => unsafe {
            klog!("[KERN]: Trap::Exception(StoreFault)");
            handle_fault(
                task,
                FaultInfo::MemoryAccess {
                    address: Some(register::mtval::read() as u32),
                    source: FaultSource::User,
                },
            );
        },
        Trap::Exception(InstructionFault) => unsafe {
            klog!("[KERN]: Trap::Exception(InstructionFault)");
            handle_fault(task, FaultInfo::IllegalText);
        },
        _ => {
            klog!(
                "[KERN]: Unimplemented cause 0x{:08x}",
                register::mcause::read().bits() as u32
            );
        }
    }
    klog!("[KERN]: ****** trap_handler() DONE ******");
}

// Timer handling.
//
// We currently only support single HART systems.  From reading elsewhere,
// additional harts have their own mtimecmp offset at 0x8 intervals from hart0.
//
// As per RP2350 Datasheet, section 3.1.11, the address of mtimecmp on
// our supported board is 0xD000_01B8, which also matches qemu ??
//
// On both RV32 and RV64 systems the mtime and mtimecmp memory-mapped registers
// are 64-bits wide.
//
const MTIMECMP: u64 = 0xD000_01B8;
const MTIME: u64 = 0xD000_01B0;

// Configure the timer.
//
// RISC-V Privileged Architecture Manual
// 3.2.1 Machine Timer Registers (mtime and mtimecmp)
//
// To keep things simple, especially on RV32 systems where we cannot atomically
// write to the mtime/mtimecmp memory-mapped registers as they are 64 bits
// wide, we only utilise the first 32-bits of each register, setting the
// high-order bits to 0 on startup, and restarting the low-order bits of mtime
// back to 0 on each interrupt.
//
#[no_mangle]
unsafe fn set_timer(tick_divisor: u32) {
    // Set high-order bits of mtime to zero.  We only call this function prior
    // to enabling interrupts so it should be safe.
    unsafe {
        arch::asm!("
            li {0}, {mtimecmp}  # load mtimecmp address
            li {1}, -1          # start with all low-order bits set
            sw {1}, 0({0})      # set low-order bits -1
            sw zero, 4({0})     # set high-order bits to 0
            sw {2}, 0({0})      # set low-order bits to tick_divisor
    
            li {0}, {mtime}     # load mtime address
            sw zero, 4({0})     # set high-order bits to 0
            sw zero, 0({0})     # set low-order bits back to 0
            ",
            out(reg) _,
            out(reg) _,
            in(reg) tick_divisor,
            mtime = const MTIME,
            mtimecmp = const MTIMECMP,
        );
    }
}

fn safe_timer_handler(tasks: &mut [task::Task]) {
    // Load the time before this tick event.
    let t0 = TICKS[0].load(Ordering::Relaxed);
    let t1 = TICKS[1].load(Ordering::Relaxed);

    // Advance the kernel's notion of time by adding 1. Laboriously.
    let (t0, t1) = if let Some(t0p) = t0.checked_add(1) {
        // Incrementing t0 did not roll over, no need to update t1.
        TICKS[0].store(t0p, Ordering::Relaxed);
        (t0p, t1)
    } else {
        // Incrementing t0 overflowed. We need to also increment t1. We use
        // normal checked addition for this, not wrapping, because this
        // should not be able to overflow under normal operation, and would
        // almost certainly indicate state corruption that we'd like to
        // discover.
        TICKS[0].store(0, Ordering::Relaxed);
        TICKS[1].store(t1 + 1, Ordering::Relaxed);
        (0, t1 + 1)
    };

    // Process any timers.
    let now = Timestamp::from([t0, t1]);
    let switch = task::process_timers(tasks, now);

    if switch != task::NextTask::Same {
        unsafe {
            with_task_table(|tasks| {
                let current = CURRENT_TASK_PTR.load(Ordering::Relaxed);
                uassert!(!current.is_null()); // irq before kernel started?
                let idx = (current as usize - tasks.as_ptr() as usize)
                    / core::mem::size_of::<task::Task>();

                let next = task::select(idx, tasks);
                apply_memory_protection(next);
                set_current_task(next);
            });
        }
    }

    unsafe {
        // Reset mtime back to 0.  In theory we could save an instruction on
        // RV32 here and only write the low-order bits, assuming that it has
        // been less than 12 seconds or so since our last interrupt(!), but
        // let's avoid any possibility of a nasty surprise.
        core::ptr::write_volatile(MTIME as *mut u64, 0);
    }
}
