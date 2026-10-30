/*
  Linker Script for STM32F401CCU6
  Adapted from stm32f407.ld
  Author: Nguyen Hai Nam

  STM32F401CCU6 Memory:
    FLASH : 256KB  (0x08000000 - 0x0803FFFF)
    SRAM  :  64KB  (0x20000000 - 0x2000FFFF)

  FLASH Memory Layout:
  -----------------------------------------------------  <-------- 0x0800_0000
  |         .isr_vector (Interrupt Vector)            |
  -----------------------------------------------------
  |                   .text (Code)                    |
  -----------------------------------------------------
  |                   .rodata (Constant)              |
  -----------------------------------------------------
  |                   .preinit_array                  |
  -----------------------------------------------------
  |                   .init_array                     |
  -----------------------------------------------------
  |                   .fini_array                     |
  -----------------------------------------------------  <-------- _sidata
  |                  .data (Initialized Data)         |
  -----------------------------------------------------
  |                    Unused Flash                   |
  -----------------------------------------------------

  RAM Layout:
  ----------------------------------------------------- <-------- _sdata = 0x2000_0000
  |                   .data                           |
  ----------------------------------------------------- <-------- _edata = _sbss
  |                   .bss                            |
  ----------------------------------------------------- <-------- _ebss
  |                   Heap                            |
  -----------------------------------------------------
  |                   Stack  (grows downward)         |
  ----------------------------------------------------- <-------- _estack = 0x2001_0000

  NOTE:
  1. Difference between LMA and VMA
  - LMA (Load Memory Address): The physical memory address where code and data are
    loaded into the microcontroller during the firmware flashing process.
  - VMA (Virtual Memory Address): The memory address that the CPU actually uses to
    fetch instructions or read/write data while the program is executing.
  2. The Local Counter "." will always refer to the VMA.
  3. Linker Script works in a multi-pass mechanism (scans multiple times):
  - Pass 1: Scans all sections, calculates the total size, collects the .o files,
    and assigns LMA/VMA regions for each block.
  - Pass 2: Goes back to calculate and assign specific values to symbols
    (such as _sidata, _sdata).
*/

/* Entry Point of whole program */
ENTRY(Reset_Handler)

/* End of Stack — top of SRAM (64KB) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Minimum size reserved for Heap and Stack
 * The linker will report an error if RAM cannot accommodate both */
_Min_Heap_Size  = 0x800;   /* 2 KB  (reduced from 4KB to fit 64KB SRAM) */
_Min_Stack_Size = 0x800;   /* 2 KB  (reduced from 4KB to fit 64KB SRAM) */

/* Internal memory of STM32F401CCU6 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K   /* 256KB Flash */
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 64K    /*  64KB SRAM  */
}

SECTIONS
{
  /* Store the interrupt vector table at FLASH memory start */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))  /* KEEP(): Prevent GC with -Wl,--gc-sections */
    . = ALIGN(4);
  } >FLASH       /* VMA = LMA = FLASH */

  /* Store the firmware code in FLASH memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text.*)            /* Group all .text sub-sections */
    KEEP (*(.init))       /* Keep function in init.s  */
    KEEP (*(.fini))       /* Keep function in fini.s  */
    . = ALIGN(4);
  } >FLASH       /* VMA = LMA = FLASH */

  /* Store constant data / read-only symbols in FLASH memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } >FLASH       /* VMA = LMA = FLASH */

  /* Define memory partitions for C++ global/static constructors/destructors */
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* LMA of .data in FLASH (used by Reset_Handler to copy to SRAM) */
  _sidata = LOADADDR(.data);

  /* Store initialized global/static variables: LMA=FLASH, VMA=RAM */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;           /* VMA start of .data in SRAM */
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;           /* VMA end of .data in SRAM   */
  } >RAM AT> FLASH        /* VMA = RAM, LMA = FLASH */

  /* Zero-initialized global/static variables (not stored in FLASH) */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM                  /* VMA = LMA = RAM */

  /* Memory area for Heap (malloc) and Stack space check */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end    = . );   /* Standard alias — used in syscalls.c       */
    PROVIDE ( _end   = . );   /* Extra alias for some newlib versions       */
    PROVIDE ( __end__ = . );  /* Extra alias for rdimon semihosting         */
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}
