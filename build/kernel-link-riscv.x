/* kernel-link-riscv.x — Hubris OS on RP2350 (RISC-V / riscv-rt) */

/* Memory layout defined externally (e.g. memory.x) */
INCLUDE memory.x

/* riscv-rt memory regions */
REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

/* Entry point and trap entry symbol from riscv-rt */
ENTRY(_start);
EXTERN(_start_trap);

/* Exception & interrupt handlers */
EXTERN(ExceptionHandler);
EXTERN(DefaultHandler);
EXTERN(__INTERRUPTS);

/* Default exception mapping */
PROVIDE(InstructionMisaligned   = DefaultHandler);
PROVIDE(InstructionFault        = DefaultHandler);
PROVIDE(IllegalInstruction      = DefaultHandler);
PROVIDE(Breakpoint              = DefaultHandler);
PROVIDE(LoadMisaligned          = DefaultHandler);
PROVIDE(LoadFault               = DefaultHandler);
PROVIDE(StoreMisaligned         = DefaultHandler);
PROVIDE(StoreFault              = DefaultHandler);
PROVIDE(UserEnvCall             = DefaultHandler);
PROVIDE(SupervisorEnvCall       = DefaultHandler);
PROVIDE(MachineEnvCall          = DefaultHandler);
PROVIDE(InstructionPageFault    = DefaultHandler);
PROVIDE(LoadPageFault           = DefaultHandler);
PROVIDE(StorePageFault          = DefaultHandler);

/* Kernel-specific default handler alias */
PROVIDE(DefaultHandler = DefaultHandler);

/* Default abort entry point. If no abort symbol is provided, then abort maps to _default_abort. */
EXTERN(_default_abort);
PROVIDE(abort = _default_abort);

/* Pre-init hook (optional override via riscv-rt's `pre_init!`) */
PROVIDE(_pre_init_trap = _default_abort);

/* Multi-processor hook function (for multi-core targets only). If no _mp_hook symbol
   is provided, then _mp_hook maps to _default_mp_hook, which leaves HART 0 running while
   the other HARTS stuck in a busy loop. Note that _default_mp_hook cannot be overwritten.
   We use PROVIDE to avoid compilation errors in single hart targets, not to allow users
   to overwrite the symbol. */
PROVIDE(_default_mp_hook = abort);
PROVIDE(_mp_hook = _default_mp_hook);

/* Default interrupt setup entry point. If not _setup_interrupts symbol is provided, then
   _setup_interrupts maps to _default_setup_interrupts, which in direct mode sets the value
   of the xtvec register to _start_trap and, in vectored mode, sets its value to
   _vector_table and enables vectored mode. */
EXTERN(_default_setup_interrupts);
PROVIDE(_setup_interrupts = _default_setup_interrupts);

/* Default exception handler. By default, the exception handler is abort.
   Users can override this alias by defining the symbol themselves */
PROVIDE(ExceptionHandler = abort);

/* Default interrupt handler. By default, the interrupt handler is abort.
   Users can override this alias by defining the symbol themselves */
PROVIDE(DefaultHandler = abort);

/* Default interrupt trap entry point. When vectored trap mode is enabled,
   the riscv-rt crate provides an implementation of this function, which saves caller saved
   registers, calls the the DefaultHandler ISR, restores caller saved registers and returns.
   Note, however, that this provided implementation cannot be overwritten. We use PROVIDE
   to avoid compilation errors in direct mode, not to allow users to overwrite the symbol. */
PROVIDE(_start_DefaultHandler_trap = _start_trap);

/* It is possible to define a special handler for each interrupt type.
   By default, all interrupts are handled by DefaultHandler. However, users can
   override these alias by defining the symbol themselves */
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));
PROVIDE(_max_hart_id = 1); /* 2 hazard3 harts in rp2350 */
PROVIDE(_hart_stack_size = SIZEOF(.stack) / (_max_hart_id + 1));
PROVIDE(_heap_size = 0);

/* Section layout */
SECTIONS {
  /* Optional header for bootloader */
  .header : {
    ASSERT(. == ALIGN(_HUBRIS_IMAGE_HEADER_ALIGN), "header alignment invalid");
    HEADER = .;
    . = . + _HUBRIS_IMAGE_HEADER_SIZE;
  } > FLASH

  /* .text code + init + trap handlers */
  .text : ALIGN(4) {
    _stext = .; __stext = .;
    *(.init); *(.init.rust); KEEP(*(.text.start));
    *(.text .text.*);
    *(.trap .trap.*);
    . = ALIGN(4);
    __etext = .;
  } > FLASH

  /* .rodata including Hubris ID marker */
  .rodata __etext : ALIGN(4) {
    __srodata = .;
    *(.srodata .srodata.*); *(.rodata .rodata.*);
    KEEP(*(.hubris_id));
    . = ALIGN(4);
    __erodata = .;
  } > FLASH

  /* Stack region */
  .stack (NOLOAD) : ALIGN(16) {
    _stack_base = .;
    . = ORIGIN(STACK) + LENGTH(STACK);
    _stack_start = .;
  } > STACK

  /* Data region in RAM, loaded from FLASH */
  .data : ALIGN(8) {
    . = ALIGN(8);
    __sdata = .;
    *(.sdata .sdata.*); *(.data .data.*);
    . = ALIGN(8);
  } > RAM AT> FLASH
  . = ALIGN(8);
  __edata = .;
  __sidata = LOADADDR(.data);

  /* BSS region, zero-initialized in RAM */
  .bss (NOLOAD) : ALIGN(8) {
    . = ALIGN(8);
    __sbss = .;
    *(.sbss .sbss.*); *(.bss .bss.*); *(COMMON);
    . = ALIGN(8);
  } > RAM
  . = ALIGN(8);
  __ebss = .;

  /* Uninitialized user data region */
  .uninit (NOLOAD) : ALIGN(8) {
    . = ALIGN(8);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(8);
    __euninit = .;
  } > RAM

  /* Define start of heap after uninit */
  PROVIDE(__sheap = __euninit);

  /* GOT/PLT placeholders (must be empty for riscv-rt) */
  .got (NOLOAD) : { KEEP(*(.got .got.*)); }
  .plt (NOLOAD) : { KEEP(*(.plt .plt.*)); }

  /* Discard debug/unneeded sections */
  /DISCARD/ : {
    *(.eh_frame); *(.eh_frame_hdr); *(.note.gnu.build-id);
  }
}

/* Optional device-tailored memory regions */
INCLUDE device.x

/* Alignment and format validation */
ASSERT(ORIGIN(FLASH) % 4 == 0, "Flash origin must be 4-byte aligned");
ASSERT(ORIGIN(RAM)  % 8 == 0, "RAM origin must be 8-byte aligned");
ASSERT(__sdata % 8 == 0 && __edata % 8 == 0, ".data alignment error");
ASSERT(__sidata % 4 == 0, ".sidata alignment error");
ASSERT(__sbss % 8 == 0 && __ebss % 8 == 0, ".bss alignment error");
ASSERT(__sheap % 8 == 0, "Heap start alignment error");
/*ASSERT(ADDR(.got) == ADDR(.plt) && SIZEOF(.got) == 0, "GOT/PLT not allowed");*/
ASSERT(_stack_start % 16 == 0, "Stack alignment error");
