/* Kernel tick via Cortex-M SysTick at the 192 MHz core clock. */
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "timer.h"
#include "yp3-buildstamp.h"
#include <stdint.h>

#ifndef YP3_PROFILE
#define YP3_PROFILE 0
#endif

#define SYST_CSR   (*(volatile uint32_t *)0xE000E010)
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018)

#define SYST_CSR_ENABLE     (1u << 0)
#define SYST_CSR_TICKINT    (1u << 1)
#define SYST_CSR_CLKSOURCE  (1u << 2)   /* processor clock */

void tick_start(unsigned int interval_in_ms)
{
    uint32_t reload = (CPU_FREQ / 1000u) * interval_in_ms - 1u;

    SYST_CSR = 0;                       /* stop while reprogramming */
    SYST_RVR = reload & 0x00ffffffu;    /* 24-bit reload */
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
}


void yp3_systick_body(void)
{
    /* Before the tick tasks, not after: the dead-man and the rig countdown are
     * the things that have to keep running when a tick task is what wedged. */
    yp3_rig_tick();
    call_tick_tasks();
}

#if YP3_PROFILE
/* A sampling profiler, because "the codec runs at 1% of real time" has been
 * blamed on XIP fetches while the codec itself already runs from SRAM. Each
 * tick records where the interrupted code was. Nothing is logged: the arrays
 * are .bss, which a reset does not clear, and tools/prof_read.py reads them
 * over USB after the rig reboot and maps buckets to symbols with nm.
 *
 * Regions: 0 core flash (.text, below the codec slots), 1 codec slots in
 * flash, 2 codecbuf (the running codec), 3 .icode/.data, 4 boot ROM,
 * 5 anything else, 6 other SRAM. The first profile put 35% in "SRAM" and
 * 0.1% in the codec, so .icode gets its own fine histogram: it is where the
 * idle loop, udelay and the SD drain live, and those mean opposite things. */
#define PROF_FLASH_BASE 0x00c0e000u
#define PROF_FLASH_END  0x00c70000u     /* codec slots start here */
#define PROF_FLASH_SHIFT 9              /* 512-byte buckets */
#define PROF_CODEC_SHIFT 8              /* 256-byte buckets */
#define PROF_CODEC_SPAN  0x00020000u    /* 128 KB of codecbuf is plenty */
#define PROF_ICODE_BASE 0x00806000u
#define PROF_ICODE_SPAN 0x00008000u
#define PROF_ICODE_SHIFT 5              /* 32-byte buckets */

extern unsigned char codecbuf[];
uint32_t yp3_prof_region[8];
uint16_t yp3_prof_icode[PROF_ICODE_SPAN >> PROF_ICODE_SHIFT];
/* Which thread was running, by slot (thread_self() & 0xff). With the LCD
 * busy-wait gone the codec STILL got 0.1% and the scheduler 23%, so the
 * question is no longer "where is the PC" but "whose turn is it". The host
 * names the slots from __threads[] and the ELF. */
uint16_t yp3_prof_thread[32];
uint32_t yp3_prof_total;
uint16_t yp3_prof_flash[(PROF_FLASH_END - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT];
uint16_t yp3_prof_codec[PROF_CODEC_SPAN >> PROF_CODEC_SHIFT];

void __attribute__((section(".icode"), used)) yp3_prof_sample(uint32_t pc)
{
    uint32_t cb = (uint32_t)codecbuf, i;

    yp3_prof_total++;
    i = thread_self() & 0xffu;
    if (i < 32 && yp3_prof_thread[i] != 0xffffu)
        yp3_prof_thread[i]++;
    if (pc >= PROF_FLASH_BASE && pc < PROF_FLASH_END) {
        yp3_prof_region[0]++;
        i = (pc - PROF_FLASH_BASE) >> PROF_FLASH_SHIFT;
        if (yp3_prof_flash[i] != 0xffffu)
            yp3_prof_flash[i]++;
    } else if (pc >= PROF_FLASH_END && pc < 0x00e00000u) {
        yp3_prof_region[1]++;
    } else if (pc >= cb && pc < cb + PROF_CODEC_SPAN) {
        yp3_prof_region[2]++;
        i = (pc - cb) >> PROF_CODEC_SHIFT;
        if (yp3_prof_codec[i] != 0xffffu)
            yp3_prof_codec[i]++;
    } else if (pc >= PROF_ICODE_BASE && pc < PROF_ICODE_BASE + PROF_ICODE_SPAN) {
        yp3_prof_region[3]++;
        i = (pc - PROF_ICODE_BASE) >> PROF_ICODE_SHIFT;
        if (yp3_prof_icode[i] != 0xffffu)
            yp3_prof_icode[i]++;
    } else if (pc >= 0x00800000u && pc < 0x00880000u) {
        yp3_prof_region[6]++;
    } else if (pc < 0x00060000u) {
        yp3_prof_region[4]++;
    } else {
        yp3_prof_region[5]++;
    }
}
#endif

/* The vector table (vectors-arm-micro.S, shared) names this. Naked so the
 * profiler can read the stacked PC out of the exception frame before any
 * prologue moves the stack: EXC_RETURN bit 2 says which stack it is on. */
void __attribute__((naked)) systick_handler(void)
{
#if YP3_PROFILE
    __asm__ volatile (
        "tst    lr, #4              \n"
        "ite    eq                  \n"
        "mrseq  r0, msp             \n"
        "mrsne  r0, psp             \n"
        "ldr    r0, [r0, #24]       \n"    /* stacked PC */
        "push   {r4, lr}            \n"
        "bl     yp3_prof_sample     \n"
        "pop    {r4, lr}            \n"
        "b      yp3_systick_body    \n");
#else
    __asm__ volatile ("b yp3_systick_body\n");
#endif
}

/* The general-purpose timer is not used by this target. */
bool timer_set(long cycles, bool start)
{
    (void)cycles;
    (void)start;
    return false;
}
bool timer_start(void) { return false; }
void timer_stop(void) { }

/* Any device interrupt we have not written a driver for is masked rather than
 * allowed to loop in the default handler. */
void yp3_spurious_irq(void)
{
    uint32_t ipsr;
    unsigned irq;
    __asm__ volatile ("mrs %0, ipsr" : "=r"(ipsr));
    irq = (ipsr & 0x1ff);
    if (irq >= 16) {
        irq -= 16;
        ((volatile uint32_t *)0xE000E180)[irq >> 5] =
            1u << (irq & 31);  /* NVIC ICER */
    }
}

/* Mask every device interrupt. SysTick is a core exception, not NVIC, so the
 * kernel tick is unaffected. */
void yp3_mask_all_irqs(void)
{
    volatile uint32_t *icer = (volatile uint32_t *)0xE000E180;
    volatile uint32_t *icpr = (volatile uint32_t *)0xE000E280;
    int i;
    for (i = 0; i < 8; i++) { icer[i] = 0xffffffffu; icpr[i] = 0xffffffffu; }
}
