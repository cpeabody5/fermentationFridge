ENTRY(Reset)

MEMORY {
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(ram) + LENGTH(ram);
PROVIDE(_stack_top = _estack);

SECTIONS {
  .vectors 0x08000000 : {
  LONG(_stack_top)
  LONG(Reset)
  LONG(NMI_Handler)
  LONG(HardFault_Handler)
  LONG(0) /* Reserved vectors... */
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(sys_tick_handler)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(0)
  LONG(tim2_isr)
} > rom
  .text : {
  KEEP(*(.text.Reset))     /* keep the Reset handler */
  KEEP(*(.text.tim2_isr))  /* keep the ISR */
  *(.text*)                /* everything else */
  *(.rodata*)
} > rom

  .data : {
    *(.data*)
  } > ram

  .bss : {
    *(.bss*)
    *(COMMON)
  } > ram
}