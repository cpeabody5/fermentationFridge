ENTRY(Reset)

MEMORY {
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(ram) + LENGTH(ram);
PROVIDE(_stack_top = _estack);

SECTIONS {
  .vectors : {
    KEEP(*(.vectors))
  } > rom

  .text : {
    *(.text*)
  } > rom

  .rodata : {
    *(.rodata*)
  } > rom

  .data : AT (ADDR(.rodata) + SIZEOF(.rodata)) {
    _data_vma = .;
    *(.data*)
    _edata = .;
  } > ram

  _data_lma = LOADADDR(.data);
  PROVIDE(_data_lma = _data_lma);
  PROVIDE(_data_vma = _data_vma);
  PROVIDE(_edata    = _edata);

  .bss : {
    _bss_start = .;
    *(.bss*)
    *(COMMON)
    _bss_end = .;
  } > ram

  PROVIDE(_bss_start = _bss_start);
  PROVIDE(_bss_end   = _bss_end);
}