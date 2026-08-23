/* Kernel tick via Cortex-M SysTick.
 * Core runs from the 192 MHz core PLL (measured with the ROM's get_clock_freq). */
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "timer.h"
#include <stdint.h>
#include "breadcrumb.h"

#define SYST_CSR   (*(volatile uint32_t *)0xE000E010)
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014)
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018)

#define SYST_CSR_ENABLE     (1u << 0)
#define SYST_CSR_TICKINT    (1u << 1)
#define SYST_CSR_CLKSOURCE  (1u << 2)   /* processor clock */

void tick_start(unsigned int interval_in_ms)
{
    BC(4);
    uint32_t reload = (CPU_FREQ / 1000u) * interval_in_ms - 1u;

    SYST_CSR = 0;                       /* stop while reprogramming */
    SYST_RVR = reload & 0x00ffffffu;    /* 24-bit reload */
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
}

/* PROBE: how much of the main thread's stack has ever been used.
 *
 * "stkov main" says the bottom word was reached; it does not say by how much,
 * and 0x1800 -> 0x4000 was a guess. new_thread_base_init fills the whole region
 * with DEADBEEF, so the first word from the bottom that is NOT DEADBEEF is the
 * high-water mark. Scanning the untouched part is cheap and it only runs every
 * 256 ticks. Delete this once the stack size is settled. */
static void stack_watermark(void)
{
    extern uintptr_t stackbegin[], stackend[];
    const uint32_t *p = (const uint32_t *)stackbegin;
    const uint32_t *e = (const uint32_t *)stackend;

    while (p < e && *p == 0xdeadbeefu)
        p++;
    BC_SLOT(171) = (uint32_t)((const char *)e - (const char *)p);   /* used */
    BC_SLOT(172) = (uint32_t)((const char *)e - (const char *)stackbegin);
}

void systick_handler(void)
{
    BC(5);   /* interrupts are live */
    if ((BC_SLOT(166) & 0xff) == 0)
        stack_watermark();
    /* PROBE, for lcd_irq_test(): count the ticks. A test that compares "with
     * interrupts" against "with interrupts masked" is worthless if the tick was
     * never firing in the first place - that failure mode has produced false
     * negatives in this port repeatedly - so the tick counts itself and the
     * display test reads the counter on both sides of its phases. */
    BC_SLOT(166) = BC_SLOT(166) + 1;
    call_tick_tasks();
}

/* General-purpose timer API - not needed for boot.  TODO. */
bool timer_set(long cycles, bool start) { (void)cycles; (void)start; return false; }
bool timer_start(void) { return false; }
void timer_stop(void) { }

/* Any device interrupt we have not written a driver for. Records the IRQ number,
 * masks it in the NVIC and returns, instead of hanging in UIE's while(1). */
void yp3_spurious_irq(void)
{
    uint32_t ipsr;
    unsigned irq;
    __asm__ volatile ("mrs %0, ipsr" : "=r"(ipsr));
    irq = (ipsr & 0x1ff);
    if (irq >= 16) {
        irq -= 16;
        BC_SLOT(21) = 0x19000000u | irq;            /* which IRQ fired */
        BC_SLOT(22) = BC_SLOT(22) + 1;              /* how many times */
        ((volatile uint32_t *)0xE000E180)[irq >> 5] = 1u << (irq & 31);  /* NVIC ICER */
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

/* Rockbox's hardfault_handler is a weak alias to UIE (while(1)), so a fault is
 * indistinguishable from a hang. Override it and record what happened. */
void hardfault_handler(void)
{
    uint32_t *sp;
    __asm__ volatile ("mrs %0, msp" : "=r"(sp));
    BC_SLOT(23) = 0xFA010000u;                          /* we faulted */
    BC_SLOT(24) = *(volatile uint32_t *)0xE000ED28;     /* CFSR */
    BC_SLOT(25) = sp[6];                                /* stacked PC */
    while (1) ;
}
