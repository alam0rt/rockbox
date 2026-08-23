/* SL6801 register map - all addresses measured on hardware */
#ifndef __SL6801_H__
#define __SL6801_H__
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG16(a) (*(volatile uint16_t *)(a))
#define REG8(a)  (*(volatile uint8_t  *)(a))

/* clock controller */
#define CLKCTRL_BASE    0x40080000
/* GPIO: 4 ports, 0x40 stride; read at +0x10 */
#define GPIO_BASE       0x40081000
#define GPIO_PORT(n)    (GPIO_BASE + (n) * 0x40)
#define GPIO_IN(n)      REG32(GPIO_PORT(n) + 0x10)
/* LCD controller */
#define LCDC_BASE       0x40095000
#define LCDC_STATUS     REG32(LCDC_BASE + 0x14)
#define LCDC_CTRL       REG32(LCDC_BASE + 0x20)
#define LCDC_CMD        REG32(LCDC_BASE + 0x24)
#define LCDC_DATALEN    REG32(LCDC_BASE + 0x28)
#define LCDC_DATA       REG32(LCDC_BASE + 0x30)
/* 0x34 is the READ FIFO (vendor 0x815932 returns base[0x34]); the DMA path
 * streams writes to it too (0x815a9a). 0x30 is the PIO write register. */
#define LCDC_DATA_RD    REG32(LCDC_BASE + 0x34)
/* 0x10: the vendor sets (x & ~0x30) | 8 before a DMA data phase. */
#define LCDC_CFG10      REG32(LCDC_BASE + 0x10)
#define LCDC_ST_BUSY    0x60000000
#define LCDC_ST_DONE    0x80000000
/* watchdog: ROM 0xb7b0 starts it, ROM 0xb7e4 stops it with WDOG_CTRL = 0 */
#define WDOG_BASE       0x40083000
#define WDOG_FEED       REG32(WDOG_BASE + 0x04)
#define WDOG_LOAD       REG32(WDOG_BASE + 0x14)
#define WDOG_CTRL       REG32(WDOG_BASE + 0x18)
/* audio */
#define AUDIO_CTRL      REG32(0x40040300)
#define AUDIO_ENABLE    REG32(0x40040304)
#define AUDIO_IRQSTAT   REG32(0x40040308)
#define AUDIO_DMA(ch)   (0x40040200 + (ch) * 0x10)
#define AUDIO_DMA_CTRL(ch) REG16(AUDIO_DMA(ch) + 0x04)
#define AUDIO_DMA_ADDR(ch) REG32(AUDIO_DMA(ch) + 0x08)
#define AUDIO_DMA_LEN(ch)  REG32(AUDIO_DMA(ch) + 0x0c)
#define IRQ_AUDIO       43
/* clock ids that more than one driver needs; the rest stay with their driver */
#define NORF_CLK        0x2b        /* SPI flash - we execute through it */
/* SDIO (ST-derived layout) */
#define SDIO_BASE       0x40003000
/* UART used by boot ROM printf, 1500000 baud */
#define UART_BASE       0x40092000
/* key ADC - the resistor ladder the vendor reads buttons from (/dev/kadc_ch1).
 * Base from the driver struct at SRAM 0x81e4b4, written by FIRM 0xd7dee4. */
#define ADC_BASE        0x40096000
#define ADC_REG(o)      REG32(ADC_BASE + (o))
#define ADC_DATA(ch)    ADC_REG(0x24 + 0x10 * (ch))   /* SRAM 0x8156d0 */
#define ADC_CHCFG(ch)   ADC_REG(0x20 + 0x10 * (ch))
/* TEN channels, not eight: adc_init's copy loop at FIRM 0xd7df4c ends on
 * cmp r2,#10 and lays cfg down at 0x20,0x30..0xb0, and the vendor registers
 * /dev/kadc_ch0 through /dev/kadc_ch9. The per-channel MODE nibble lives in
 * ADC(0x10) for channels 0-7 and in ADC(0x18) for 8 and 9 (0xd7dfc2). */
#define ADC_NCHAN       10

/* Boot ROM entry points. These live here rather than in one driver because
 * duplicated magic addresses drift: the LCD driver had its own copies and the
 * button driver needed the same ones. Thumb bit included. */
#define ROM_CLK_IS_ON   ((int  (*)(unsigned))0x2519u)
#define ROM_CLK_ENABLE  ((void (*)(unsigned))0x2565u)
#define ROM_CLK_SRC     ((void (*)(unsigned, unsigned))0x3119u)
#define ROM_CLK_DIV     ((void (*)(unsigned, unsigned))0x2d59u)
#define ROM_CLK_APPLY   ((void (*)(unsigned))0x27a1u)
/* 0x27a0 starts a clock, 0x2a7c stops it. FIRM settles this at
 * 0xd7e844/0xd7e88e, where codec enable starts 0x1f and 0x38. */
#define ROM_CLK_STOP    ((void (*)(unsigned))0x2a7du)
#define ROM_GPIO_CFG1   ((void (*)(unsigned))0x7adu)
#define ROM_GPIO_WRITE  ((void (*)(unsigned, unsigned))0x80fu)

/* A clock SOURCE number is itself a clock id, so selecting one can hand a
 * peripheral a dead clock. This is the vendor's wrapper (bootloader 0x8206c2,
 * ROM 0x8050f2) and the only correct way to change a source. */
#define ROM_CLK_SET_SRC(id, src) do {                       \
        if ((src) == 8 || (src) == 9) ROM_CLK_APPLY(src);   \
        ROM_CLK_SRC((id), (src));                           \
    } while (0)

#endif
