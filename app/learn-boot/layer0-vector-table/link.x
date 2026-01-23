/* Layer 0 Linker Script for RP2350
 *
 * This is based on cortex-m-rt's link.x but adds RP2350-specific IMAGE_DEF support.
 * This follows the same pattern as Hubris's kernel-link.x.
 */

/* Include memory regions from memory.x */
INCLUDE memory.x

/* Entry point */
ENTRY(Reset);
EXTERN(__RESET_VECTOR);

/* Exception vector table - provided by cortex-m-rt */
EXTERN(__EXCEPTIONS);
EXTERN(DefaultHandler);

PROVIDE(NonMaskableInt = DefaultHandler);
EXTERN(HardFaultTrampoline);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SecureFault = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
PROVIDE(DebugMonitor = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);

PROVIDE(DefaultHandler = DefaultHandler_);
PROVIDE(HardFault = HardFault_);

/* Interrupt vectors from device crate */
EXTERN(__INTERRUPTS);

/* Pre-init hook */
PROVIDE(__pre_init = DefaultPreInit);

/* Stack starts at top of RAM */
PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

SECTIONS
{
    /* Vector table at start of flash */
    .vector_table ORIGIN(FLASH) :
    {
        __vector_table_start = .;
        /* Initial Stack Pointer */
        LONG(_stack_start);

        /* Reset vector */
        KEEP(*(.vector_table.reset_vector));
        __reset_vector = .;

        /* Exception handlers */
        KEEP(*(.vector_table.exceptions));
        __eexceptions = .;

        /* Device interrupts */
        KEEP(*(.vector_table.interrupts));
        __vector_table_end = .;
    } > FLASH

    /* RP2350 IMAGE_DEF block - must be within first 4KB */
    .image_def : ALIGN(4)
    {
        __image_def_start = .;
        KEEP(*(.image_def))
        __image_def_end = .;
    } > FLASH

    /* Calculate where .text should start */
    PROVIDE(_stext = ADDR(.image_def) + SIZEOF(.image_def));

    /* Code section */
    .text _stext :
    {
        __stext = .;
        *(.Reset);
        *(.HardFaultTrampoline);
        *(.HardFault.*);
        *(.text .text.*);
        . = ALIGN(4);
        __etext = .;
    } > FLASH

    /* Read-only data */
    .rodata : ALIGN(4)
    {
        __srodata = .;
        *(.rodata .rodata.*);
        . = ALIGN(4);
        __erodata = .;
    } > FLASH

    /* Initialized data - lives in RAM, loaded from FLASH */
    .data : ALIGN(4)
    {
        __sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        __edata = .;
    } > RAM AT>FLASH

    __sidata = LOADADDR(.data);

    /* Zero-initialized data */
    .bss (NOLOAD) : ALIGN(4)
    {
        __sbss = .;
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
        __ebss = .;
    } > RAM

    /* Uninitialized data */
    .uninit (NOLOAD) : ALIGN(4)
    {
        *(.uninit .uninit.*);
    } > RAM

    /* Heap starts after .uninit */
    PROVIDE(__sheap = .);

    /* Discard unwanted sections */
    /DISCARD/ :
    {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}

/* Assertions */
ASSERT(__reset_vector == ADDR(.vector_table) + 0x8, "Reset vector misplaced");
ASSERT(__eexceptions == ADDR(.vector_table) + 0x40, "Exception vectors misplaced");
ASSERT((__image_def_end - ORIGIN(FLASH)) <= 0x1000, "IMAGE_DEF must be within first 4KB");
