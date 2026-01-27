/* Memory layout for RP2350 / Pico 2
 *
 * This file defines the memory regions for cortex-m-rt.
 * cortex-m-rt uses this to set up the vector table and sections.
 *
 * RP2350 Memory Map:
 *   XIP Flash: 0x10000000 - 0x103FFFFF (4MB on Pico 2)
 *   SRAM:      0x20000000 - 0x20081FFF (520KB)
 */

MEMORY
{
    /* XIP Flash - code executes directly from here */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4M

    /* SRAM - stack, .data, .bss go here */
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}
