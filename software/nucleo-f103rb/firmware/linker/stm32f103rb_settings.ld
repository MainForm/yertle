/* Project-owned STM32F103RB layout: reserve independent calibration and mode Flash journals (2KiB each).
   Settings have no loadable section; normal application BIN never includes them. */
ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 124K
  CALIBRATION (r) : ORIGIN = 0x0801F000, LENGTH = 2K
  SETTINGS (r) : ORIGIN = 0x0801F800, LENGTH = 2K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;
__calibration_store_start__ = ORIGIN(CALIBRATION);
__calibration_store_end__ = ORIGIN(CALIBRATION) + LENGTH(CALIBRATION);
__mode_store_start__ = ORIGIN(SETTINGS);
__mode_store_end__ = ORIGIN(SETTINGS) + LENGTH(SETTINGS);
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text : {
    . = ALIGN(4); *(.text .text.*) *(.glue_7 .glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .;
  } > FLASH
  .rodata : { . = ALIGN(4); *(.rodata .rodata.*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .preinit_array : {
    PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array : {
    PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array)) PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array : {
    PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array)) PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH
  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4); _sdata = .; *(.data .data.*) *(.ramfunc .ramfunc.*) . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH
  .bss (NOLOAD) : {
    . = ALIGN(4); _sbss = .; __bss_start__ = .; *(.bss .bss.*) *(COMMON)
    . = ALIGN(4); _ebss = .; __bss_end__ = .;
  } > RAM
  ._user_heap_stack (NOLOAD) : {
    . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .);
    . += _Min_Heap_Size; . += _Min_Stack_Size; . = ALIGN(8);
  } > RAM
  .ARM.attributes 0 : { *(.ARM.attributes) }
  ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(CALIBRATION), "firmware overlaps persistent settings pages")
  ASSERT(_estack % 8 == 0, "MSP must be 8-byte aligned")
}
