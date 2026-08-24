/* Device IRQ vectors. Unexpected device interrupts are masked and returned
 * from the shared handler until a driver claims each vector. */

/* There is deliberately no weak-alias default: the usual CMSIS idiom was
 * tried here and silently resolved references to the weak definition in this
 * object. */
    .syntax unified
    .thumb
    .section .vectors.platform,"ax",%progbits

    .global __vectors_platform
__vectors_platform:
    /* 0..59 unclaimed. IRQ 43 used to be "PCM DMA half/full transfer
       complete"; it is the USB mass-storage DMA and audio never touched it -
       see the correction at the head of docs/AUDIO.md. */
    .rept 60
    .word   yp3_spurious_irq
    .endr
    /* 60..67: the eight general DMA channels, 0x40001000 + ch*0x40. The
       vendor's own trampolines are SRAM 0x814c64..0x814c8e and its per-channel
       IRQ table is FIRM 0x00c488e8. PCM playback owns channel 2 (see
       PCM_DMA_CH in pcm-yp3box.c); the rest stay unclaimed until a driver
       takes them. */
    .word   yp3_spurious_irq    /* 60: DMA channel 0 */
    .word   yp3_spurious_irq    /* 61: DMA channel 1 */
    .word   yp3_dma2_irq        /* 62: DMA channel 2 - PCM playback */
    .word   yp3_spurious_irq    /* 63: DMA channel 3 */
    .word   yp3_spurious_irq    /* 64: DMA channel 4 */
    .word   yp3_spurious_irq    /* 65: DMA channel 5 */
    .word   yp3_spurious_irq    /* 66: DMA channel 6 */
    .word   yp3_spurious_irq    /* 67: DMA channel 7 */
    .rept 96 - 68
    .word   yp3_spurious_irq
    .endr
