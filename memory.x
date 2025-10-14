/* Linker script for the STM32F303VCT6 with stack in CCM */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 256K

    /* .bss, .data and the heap go in this region */
    RAM   : ORIGIN = 0x20000000, LENGTH = 40K

    /* Core coupled (faster) RAM dedicated to hold the stack */
    CCRAM : ORIGIN = 0x10000000, LENGTH = 8K
}

_stack_start = ORIGIN(CCRAM) + LENGTH(CCRAM);
_stack_end = ORIGIN(CCRAM); /* Optional, add if used by the application */
