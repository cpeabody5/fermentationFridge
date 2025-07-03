ENTRY(Reset)

MEMORY
{
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(ram) + LENGTH(ram);
PROVIDE(_stack_top = _estack);

SECTIONS
{
  .vectors 0x08000000 :
  {
    KEEP(*(.vectors))
  } > rom

  .text :
  {
    *(.text*)
    *(.rodata*)
  } > rom

  .data :
  {
    *(.data*)
  } > ram

  .bss :
  {
    *(.bss*)
    *(COMMON)
  } > ram
}
