/* Device IRQ vectors.
 *
 * These previously all pointed at Rockbox's UIE, which is `while(1)`. The moment
 * the LCDC was properly clocked it began raising interrupts, and the first one
 * hung the machine. Point them at our own handler instead: it records which IRQ
 * fired, masks it, and returns - so an unexpected interrupt yields evidence
 * rather than a lockup.
 *
 * A driver claims its vector by defining the handler symbol and this table
 * naming it. There is deliberately no weak-alias default: the usual CMSIS
 * idiom (.weak name / .thumb_set name, default) was tried here and SILENTLY
 * LOST - the assembler resolved the reference against the weak definition in
 * this same object, so vector 43 still pointed at yp3_spurious_irq with the
 * real handler linked in and unreferenced. The vector table read correctly in
 * nm and wrongly in the image. Name handlers directly; they are all compiled
 * unconditionally anyway.
 */
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
