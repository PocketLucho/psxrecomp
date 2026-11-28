ENTRY(_start)
SECTIONS { . = 0x80010000; .text : { *(.text.start) *(.text*) } .rodata : { *(.rodata*) } .data : { *(.data*) *(.sdata*) } .bss : { *(.sbss*) *(.bss*) *(COMMON) } }
