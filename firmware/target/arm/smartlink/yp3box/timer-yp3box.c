/* Kernel tick via Cortex-M SysTick at the 192 MHz core clock. */
#include "config.h"
#include "system.h"
#include "kernel.h"
#include "timer.h"
#include <stdint.h>

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


void systick_handler(void)
{
    call_tick_tasks();
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
