/* SL6801 startup. Code XIPs from flash; .data is copied to SRAM at boot. */
    .syntax unified
    .cpu cortex-m4
    .thumb

    .section .init.text,"ax",%progbits
    .global start
    .global reset_handler
    .type start, %function
    .type reset_handler, %function
start:
reset_handler:
    /* OPEN THE FLASH WINDOW FIRST.
     *
     * The XIP aperture decodes 0xc00000-0xe00000 (2MB) but the controller only
     * serves the first 128KB until 0x30000024 is written; its reset value is 0.
     * Found by poking each controller register from the host with write_mem and
     * re-reading 0xc40000.
     *
     * In BOOTLOADER mode the field behaves like an address mask:
     * 0x001fffff and 0x000fffff open the aperture while 0x00200000 does not.
     * But 0x001fffff does not work at boot - crt0 then dies in the .data copy,
     * which reads from ~0xc5e4d8 and so needs the window. The controller is
     * evidently in a different state here than under the ROM's USB bootloader,
     * so use the value proven in THIS context. 0xffffffff disturbs no
     * peripheral register (checked across the clock controller, SD, PWM and
     * LCDC), so the extra bits are harmless.
     *
     * This must come before ANY branch that lands past 0xc20000. crt0 sits at
     * 0xc0e1c0, inside the always-mapped 128KB, so it can safely do this.
     */
    ldr     r0, =0x30000024
    mov     r1, #0
    sub     r1, r1, #1              /* 0xffffffff - see note below */
    str     r1, [r0]
    dsb
    isb

    /* Munge the stack with 0xdeadbeef, as every other ARM crt0 does - see
       firmware/target/arm/imx233/crt0.S:155. The kernel uses this sentinel to
       detect stack overflow on the main thread. */
    ldr     r0, =stackbegin
    ldr     r1, =stackend
    ldr     r2, =0xdeadbeef
1:  cmp     r0, r1
    bhs     2f
    str     r2, [r0], #4
    b       1b
2:


    /* set up the stack */
    ldr     r0, =stackend
    mov     sp, r0

    /* copy .data from its flash image to SRAM: .text and .rodata execute in
       place from flash, so only initialised data needs moving. */
    ldr     r0, =_datacopy
    ldr     r1, =_datastart
    ldr     r2, =_dataend
4:  cmp     r1, r2
    bhs     5f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       4b
5:


    /* zero .bss */
    movs    r0, #0
    ldr     r2, =_bssbegin
    ldr     r3, =_bssend
2:  cmp     r2, r3
    it      lo
    strlo   r0, [r2], #4
    blo     2b


    /* The ROM jumps to us with interrupts masked (cpsid i) and VTOR still
       pointing at 0x800000, which is now our .bss. Fix both before main. */
    ldr     r0, =0xE000ED08         /* VTOR */
    ldr     r1, =__vectors_arm
    str     r1, [r0]
    dsb
    isb
    cpsie   i


    bl      main
3:  b       3b
    .size start, . - start
