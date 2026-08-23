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
    .rept 43
    .word   yp3_spurious_irq
    .endr
    .word   yp3_audio_irq       /* 43: PCM DMA half/full transfer complete */
    .rept 96 - 44
    .word   yp3_spurious_irq
    .endr
