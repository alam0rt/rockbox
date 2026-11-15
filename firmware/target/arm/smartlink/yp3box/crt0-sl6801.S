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
    /* --- get out of handler mode FIRST ---------------------------------
     *
     * The ROM's USB bootloader dispatches its RUNRAM command from inside the
     * USB interrupt handler: 0x627a loads the address and 0x61d4 does a bare
     * `bx`. Nothing unwinds the exception, so an image started with
     * smtlink `exec` runs in HANDLER MODE for ever, IPSR stuck at the USB
     * interrupt number.
     *
     * Measured, not deduced. An exec-booted image panicked with
     *
     *     Incorrect CPU mode in mrsw_write_acquire (0x39!=0x00)
     *
     * 0x39 is exception 57, IRQ 41, which the boot log independently names as
     * USB (`usb: ... irq41=00005695`). Rockbox asserts thread context on every
     * filesystem lock, so it boots and then dies the moment it touches a file.
     * It also means the USB ISR never returns, so the device stays enumerated
     * at the same devnum and looks to the host like it never branched at all.
     *
     * The ROM's own validated boot enters from main, in thread mode, and is
     * unaffected - which is why that path has always worked and this one never
     * has, despite 23_boot_rockbox.sh assuming it did.
     *
     * So synthesise an exception return. EXC_RETURN 0xFFFFFFF9 is "return to
     * Thread mode, use MSP", and the hardware pops R0-R3, R12, LR, PC, xPSR
     * from SP. Only PC and xPSR need to mean anything: the rest are dead
     * before crt0 sets up its own world. xPSR must have the T bit, and the
     * stacked PC has bit 0 clear because the hardware takes Thumb from xPSR.
     */
    mrs     r0, ipsr
    mov     r5, r0                  /* kept for bootctl: the frame pop below
                                       restores r0-r3/r12/lr/pc/xpsr only */
    cmp     r0, #0
    beq     10f                     /* already thread mode: the ROM boot path */

    ldr     r0, =stackend
    sub     r0, r0, #32             /* room for the 8-word exception frame */
    mov     sp, r0
    ldr     r1, =10f
    bic     r1, r1, #1
    str     r1, [r0, #24]           /* stacked PC   */
    mov     r1, #1
    lsl     r1, r1, #24             /* xPSR.T       */
    str     r1, [r0, #28]
    mov     r1, #0
    sub     r1, r1, #7              /* 0xfffffff9   */
    bx      r1
10:

    /* --- the rig hook, before ANYTHING else touches the machine ---------
     *
     * Runs while the boot ROM's world is still completely intact: early_init
     * has run, VTOR still points at the ROM's own vector table at 0x800000,
     * interrupts are enabled, and the USB module clock is on (ROM 0x4c952).
     * That is precisely the state ROM main is in at 0x4c978 when it calls the
     * download service for a held VOL key - which is why re-entering that
     * service from here is expected to work, and why this must come before
     * the flash window, the stack munge and the .data copy.
     *
     * yp3_bootctl lives at the head of .persist: never cleared by crt0, never
     * written by the .data copy, and above the 0x800000-0x806000 window the
     * boot ROM re-initialises, so it survives a reset. Offsets are fixed by
     * app.lds:
     *
     *   +0x00 magic   +0x04 request   +0x08 warm_flag  +0x0c warm_vec
     *   +0x10 resets  +0x14 boots     +0x18 rom_wdog   +0x1c spare
     */
    ldr     r0, =yp3_bootctl
    ldr     r1, [r0, #0]
    ldr     r2, =0x42430001             /* "BC" + version */
    cmp     r1, r2
    beq     11f
    /* Cold start: .persist holds whatever the last power-off left, so it has
       to be initialised before any field is believed. */
    movs    r1, #0
    str     r1, [r0, #4]
    str     r1, [r0, #8]
    str     r1, [r0, #12]
    str     r1, [r0, #16]
    str     r1, [r0, #20]
    str     r1, [r0, #24]
    str     r1, [r0, #28]
    str     r2, [r0, #0]
11:
    /* boots++. A software reset that comes back here with this incremented
       and the magic intact is the whole proof that .persist survives one -
       app.lds has always CLAIMED it does, and nothing had measured it. */
    ldr     r1, [r0, #20]
    adds    r1, #1
    str     r1, [r0, #20]

    /* The reset-path probe. ROM 0x68 reads 0x4000000c and, when it is not
       zero, does `bx *(0x30000600)` instead of running main - so a software
       reset that leaves that flag set never reaches this instruction at all.
       Arriving here IS the measurement; the value is recorded for the log.
       0x30000600 is only dereferenced when the flag is set, because nothing
       in the dumps says that register reads safely otherwise. */
    ldr     r1, =0x4000000c
    ldr     r1, [r1]
    str     r1, [r0, #8]
    movs    r2, #0
    cbz     r1, 12f
    ldr     r2, =0x30000600
    ldr     r2, [r2]
12: str     r2, [r0, #12]

    /* The IPSR we were entered with: 0 for the ROM's own validated boot,
       the USB exception number (0x39) for smtlink exec. Recorded, because
       the thread-mode drop above is otherwise only inferred from the absence
       of a panic. */
    str     r5, [r0, #28]

    /* A pending "go to the bootloader" request: hand the machine straight
       back to the ROM's USB download service, which is what the host talks
       to as 301a:2800. The request is cleared FIRST, so a service that does
       not come up costs one reset and not an unbreakable loop. The infinite
       call mirrors ROM 0x4c992, the bad-image path, exactly. */
    ldr     r1, [r0, #4]
    ldr     r2, =0x600710AD
    cmp     r1, r2
    bne     14f
    movs    r1, #0
    str     r1, [r0, #4]
    /* ...unless the host started us with exec. Then the ROM download service
       is already where we came from, and the host asked for THIS image. In rig
       mode the ROM never runs crt0 after a reset, so the request a rig reboot
       leaves behind survives to the next exec - and obeying it there diverts
       an image the host explicitly started back into a service this hook has
       never managed to bring up on USB. */
    cbnz    r5, 14f
    ldr     r4, =0x0004c8a1             /* ROM download service, Thumb */
13: blx     r4
    b       13b
14:

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


    /* The vector table has to live at 0x800000 - not in flash - because the
       boot ROM writes into it.
     *
       ROM irq_set_handler (0x2348) reads VTOR, and if it is not exactly
       0x800000 it FORCES it there before storing the handler at
       0x800000 + (irq + 16) * 4. Nothing asks its permission: the USB attach
       path alone registers IRQ 41 and 42 that way (ROM 0x553a/0x5548). Point
       VTOR at flash and the first ROM call that registers an interrupt moves
       the whole table out from under Rockbox - SysTick, the audio IRQ and
       PendSV included.

       0x800000 is where the ROM's own table already is: it is the reset VTOR,
       0x800000 is the ROM's initial stack pointer top too (vector entry 0),
       and every other piece of ROM state sits above 0x8001c0. Our linker
       keeps 0x800000-0x806000 out of the Rockbox image for exactly this
       reason. So copy our table over the ROM's, 112 words - 16 core vectors
       plus 96 device IRQs - and leave VTOR where the ROM put it. */
    ldr     r0, =0x00800000
    ldr     r1, =__vectors_arm
    add     r2, r0, #(112 * 4)
1:  ldr     r3, [r1], #4
    str     r3, [r0], #4
    cmp     r0, r2
    blo     1b

    ldr     r0, =0xE000ED08         /* VTOR */
    ldr     r1, =0x00800000
    str     r1, [r0]
    dsb
    isb
    cpsie   i


    bl      main
3:  b       3b
    .size start, . - start
