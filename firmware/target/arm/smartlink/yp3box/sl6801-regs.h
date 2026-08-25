/* SL6801 register map - all addresses measured on hardware */
#ifndef __SL6801_H__
#define __SL6801_H__
#include <stdint.h>
#include <stdbool.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG16(a) (*(volatile uint16_t *)(a))
#define REG8(a)  (*(volatile uint8_t  *)(a))

/* clock controller. The audio PLL lives here at +0x10/+0x14 (vendor set_pll,
 * SRAM HAL 0x80da34, literal 0x40080000). */
#define CLKCTRL_BASE    0x40080000
/* System/PMU block. NOT the clock controller, despite sitting next to it:
 * every vendor site that touches the PMU mailbox at +0x104..+0x110 (boot ROM
 * 0x3c64 write / 0x3ca4 read), the sleep sequencer at +0xb0/+0xb8/+0xd0 (FIRM
 * 0xcf7fae) or +0x60/+0xd8 (FIRM 0xcf7d10) loads 0x40085000. */
#define PMU_BASE        0x40085000
#define PMU_REG(o)      REG32(PMU_BASE + (o))
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
/* Audio. 0x40009000, module 0x25, clock 0x13 - NOT 0x40040300, which is the
 * USB mass-storage controller. See docs/AUDIO.md for how that was established
 * and what it cost; the short version is that FIRM 0xd66fe4 reads exactly like
 * audio_start and lives inside driver_usbd_msc_param_init. */
#define AUDIO_BASE      0x40009000
#define AUD(o)          REG32(AUDIO_BASE + (o))
#define AUDIO_MODULE    0x25u       /* FIRM 0xd7d5d0, audio_hw_init          */
#define AUDIO_CLK       0x13u       /* FIRM 0xd7d5d6                         */
#define AUDIO_MCLK      0x00u       /* the PLL output; set_pll then apply(0) */
#define ASRC_MODULE     0x20u       /* held only while zeroing 0x40009400    */
#define ASRC_CLK        0x2du
/* The playback (TX) group. +0x20c is the FIFO the DMA engine writes into.
 *
 * THIS MAP IS CORRECT AND WAS ONCE "FIXED" INTO BEING WRONG. The evidence, so
 * nobody re-derives it a third time:
 *
 *  - The vendor's playback DMA wrapper is SRAM 0x814bd4. It calls dma_start
 *    (SRAM 0x814c94) with the caller's buffer as SOURCE and **0x4000920c** as
 *    DESTINATION. The capture wrapper at 0x814bec is its mirror image:
 *    0x4000910c as source, the buffer as destination.
 *  - dma_start stores its destination at channel +0x04 and its source at
 *    +0x08, which probe/dma_work.bin confirms independently: the live LCD
 *    channel has source request id 9 in CONTROL[16:12] with CONTROL bit 17
 *    clear (source is a peripheral) and destination request id 0 with bit 5
 *    set (destination is memory) - and it is +0x08 that holds 0x40095030.
 *  - The playback stream control is base[0x200], not base[0x100]: FIRM
 *    0xd7b190 does `base[0x200] &= ~2` then `|= 1`, and its four channel
 *    volumes are the literals at 0xd7b3dc..0xd7b3e8 - 0x40009228, 0x258,
 *    0x288, 0x2b8. Stride 0x30 from 0x220, volume at +8, exactly as below.
 *  - base[0x100] is the same register file for CAPTURE. FIRM 0xd7b46c drives
 *    it with an identical stop/enable/rate sequence, which is why it reads
 *    like a transmitter enable and is not one.
 *
 * The two blocks are identical in shape, so a function that touches one looks
 * exactly like a function that touches the other. Only the DMA direction tells
 * them apart. Check the FIFO address before believing anything else. */
#define AUD_TX_CTRL     AUD(0x200)  /* [0] enable [1] run [11:8] rate code   */
#define AUD_TX_FMT      AUD(0x208)  /* [2] stereo, [7] always set            */
#define AUD_TX_FIFO     0x4000920cu
#define AUD_TX_CH(n)    (0x220u + (n) * 0x30u)      /* n = 0..3              */
#define AUD_TX_MUTE(n)  AUD(AUD_TX_CH(n) + 4)       /* bit 16                */
#define AUD_TX_VOL(n)   AUD(AUD_TX_CH(n) + 8)       /* [8:0], 0 = silent     */
/* The capture (RX) group, mirror image; +0x10c is its FIFO. */
#define AUD_RX_CTRL     AUD(0x100)
#define AUD_RX_FIFO     0x4000910cu
#define AUD_RX_CH(n)    (0x130u + (n) * 0x20u)      /* n = 0..2; 0x110 is a
                                                     * group register, not a
                                                     * channel               */
#define AUD_RX_MUTE(n)  AUD(AUD_RX_CH(n) + 4)       /* bit 16                */
#define AUD_RX_VOL(n)   AUD(AUD_RX_CH(n) + 8)       /* [8:0]                 */
#define AUD_VOL_MAX     0x1ffu

/* The general DMA engine: eight channels, module 0x21, IRQ 60 + channel.
 * Fully decoded in docs/DMA.md and confirmed against probe/dma_work.bin. */
#define DMA_BASE        0x40001000
#define DMA_MODULE      0x21u
#define DMA_CHANNELS    8
#define DMA_CH_BASE(ch) (DMA_BASE + (ch) * 0x40)
#define DMA_CTRL(ch)    REG32(DMA_CH_BASE(ch) + 0x00)
#define DMA_DST(ch)     REG32(DMA_CH_BASE(ch) + 0x04)
#define DMA_SRC(ch)     REG32(DMA_CH_BASE(ch) + 0x08)
#define DMA_LEN(ch)     REG32(DMA_CH_BASE(ch) + 0x0c)
#define DMA_CFG(ch)     REG32(DMA_CH_BASE(ch) + 0x20)
#define DMA_IRQST(ch)   REG32(DMA_CH_BASE(ch) + 0x24)
#define DMA_CTRL_GO     (1u << 30)
#define DMA_CTRL_CONT   (1u << 29)  /* cleared by dma_start, set by the loop
                                     * variant at SRAM 0x814cc0 [U]          */
#define DMA_LEN_MASK    0x3ffffu    /* 18 bits: 262143 bytes per transfer    */
#define DMA_IRQ(ch)     (60u + (ch))
#define DMA_IRQST_A     1u          /* handle callback A [U]                 */
#define DMA_IRQST_DONE  2u          /* handle callback B, transfer complete  */
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

/* PMU mailbox and status registers recovered from the vendor FIRM. */
#define PMU_REG_STATUS             0x2a
#define PMU_REG_BATTERY_VOLTAGE    0x2e
#define PMU_REG_VOLTAGE_CONFIG     0x47
#define PMU_STATUS_CHARGING        0x04
#define PMU_STATUS_POWER_INPUT     0x44

/* PMU interrupt block. Three status registers, each write-1-to-clear, each
 * with an enable register holding the INVERTED mask (FIRM 0xcf7b0c writes
 * ~mask, and FIRM 0xcf7c6c reads the status back and writes it out again):
 *
 *     status 0x30  enable 0x19        status 0x4a  enable 0x4b
 *     status 0x31  enable 0x1a
 *
 * The on/off key lives in 0x31. FIRM 0xcf7b6c turns its three bits into the
 * PMU event bits 1/2/4, and the key manager's callback at FIRM 0xcfe644 maps
 * those onto key ids: event 1 -> id 0x21 "enter" press, event 2 -> id 0x21
 * release, event 4 -> id 0x47 "power". "enter" is the centre Play/Pause key.
 */
#define PMU_REG_KEY_STATUS         0x31
#define PMU_REG_KEY_ENABLE         0x1a
#define PMU_KEY_PRESS              0x40   /* vendor "usk", event 1 */
#define PMU_KEY_LONG               0x20   /* vendor "lk",  event 4 */
#define PMU_KEY_RELEASE            0x10   /* vendor "sk",  event 2 */
#define PMU_KEY_ANY  (PMU_KEY_PRESS | PMU_KEY_LONG | PMU_KEY_RELEASE)

bool yp3_pmu_read(unsigned reg, uint8_t *value);
bool yp3_pmu_write(unsigned reg, uint8_t value);

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
