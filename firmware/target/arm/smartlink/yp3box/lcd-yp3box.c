/* GC9106 128x160 RGB565 over the SL6801 LCDC at 0x40095000.
 * Register map and init sequence extracted from the stock firmware and verified
 * on hardware by drawing to the panel. */
#include "config.h"
#include "lcd.h"
#include "system.h"
#include "sl6801-regs.h"
#include "lcd-target.h"
#include "kernel.h"
#include "yp3-buildstamp.h"
#ifdef ROCKBOX_HAS_LOGF
#define LOGF_ENABLE
#endif
#include "logf.h"

/* Bounded waits prevent a dead LCDC from hanging the whole boot. */
/* LCDC operations run from SRAM because XIP fetches can stall during setup. */
#define LCDC_SPIN 200000u

/* A full frame is 128x160x2 bytes; keep transfer primitives in SRAM. */
#define LCDICODE __attribute__((section(".icode"), noinline))


static void lcd_panel_init_seq(void);
static void lcd_set_window(int x, int y, int w, int h);

static void LCDICODE lcdc_wait_idle(void)
{
    uint32_t t = 0;
    while ((LCDC_STATUS & LCDC_ST_BUSY) && ++t < LCDC_SPIN) ;
}

static void LCDICODE lcdc_kick(uint32_t ctrl)
{
    uint32_t t = 0;
    LCDC_CTRL = ctrl;
    while (!(LCDC_STATUS & LCDC_ST_DONE) && ++t < LCDC_SPIN) ;
    LCDC_STATUS = LCDC_ST_DONE;
}

void LCDICODE lcd_write_cmd(unsigned cmd)
{
    LCDC_CMD = cmd;
    lcdc_wait_idle();
    lcdc_kick((LCDC_CTRL & ~0x30cu & ~0x2u) | 0x7u);
}

void LCDICODE lcd_write_data_n(unsigned data, unsigned n)
{
    LCDC_DATA = data;
    LCDC_DATALEN = n - 1;
    lcdc_wait_idle();
    lcdc_kick((LCDC_CTRL & ~0xeu) | 0xbu);
}

static void wd8(unsigned v) { lcd_write_data_n(v, 1); }


/* LCDC setup follows the vendor's register configuration. */
/* The controller and clock setup must run from SRAM because XIP fetches can
 * stall while the flash clock is reconfigured. */

#define LR(o)  (*(volatile uint32_t *)(LCDC_BASE + (o)))

/* Boot ROM clock helpers and the module IDs used by the LCDC. */
#define DMA_MODULE     0x21u
#define LCD_MODULE     0x5cu

/* Panel reset and backlight GPIO values from the vendor initialization. */
#define BL_PIN_ON_FROM_LCD 0x000188c0u
#define LCD_RESET_PIN  0x15000u

/* THE ACTUAL LCD PIN SET.
 *
 * Taken verbatim from the vendor's LCD hardware setup at FIRM 0xd66640, which
 * runs immediately before the panel bring-up (0xd665f8 calls 0xd66640 then
 * 0xd668a4). Each value is a packed gpio_config code passed to ROM 0x7ac.
 *
 * An earlier list here was wrong in every entry: it was assembled by collecting
 * gpio_config call sites across all of FIRM, which swept up pins belonging to
 * other peripherals (1.7, 1.9, 1.20, 1.21 are the FM radio, module 0x50) while
 * missing the real bus entirely - including four pins on PORT 3, which were
 * never considered because the reset pin happened to be on port 1.
 *
 *   mode 12 = LCD bus alternate function
 *   0x000150cb = pin 1.10, mode 1 (GPIO output), level high - the panel RESX
 */
static const unsigned lcd_pins[] = {
    0x00010600u,  /* 1.0  mode 12 */
    0x00010e00u,  /* 1.1  mode 12 */
    0x00011600u,  /* 1.2  mode 12 */
    0x00011e00u,  /* 1.3  mode 12 */
    0x00012600u,  /* 1.4  mode 12 */
    0x00012e00u,  /* 1.5  mode 12 */
    0x00019600u,  /* 1.18 mode 12 */
    0x00030e00u,  /* 3.1  mode 12 */
    0x00031600u,  /* 3.2  mode 12 */
    0x00031e00u,  /* 3.3  mode 12 */
    0x00035600u,  /* 3.10 mode 12 */
    0x000150cbu,  /* 1.10 mode 1, level high - panel reset */
};

/* Release port 3 pin 0 before configuring the LCD pins. */
#define LCD_PIN_RELEASE_3_0  0x00030780u

static void lcd_pins_config(void)
{
    unsigned i;
    ROM_GPIO_CFG1(LCD_PIN_RELEASE_3_0);
    for (i = 0; i < sizeof(lcd_pins)/sizeof(lcd_pins[0]); i++)
        ROM_GPIO_CFG1(lcd_pins[i]);
}

static void lcd_panel_reset(void)
{
    /* GC9106 requires a bounded reset pulse before accepting commands. */
    lcd_pins_config();
    ROM_GPIO_WRITE(LCD_RESET_PIN, 1); udelay(1000);     /* 1ms  */
    ROM_GPIO_WRITE(LCD_RESET_PIN, 0); udelay(10000);    /* 10ms */
    ROM_GPIO_WRITE(LCD_RESET_PIN, 1); udelay(50000);    /* 50ms */
}
#define LCD_CLOCK      0x3fu

/* Clock changes run from SRAM because changing the flash clock can stall an
 * instruction fetch from XIP flash. */
static void __attribute__((section(".icode"), noinline)) lcd_clock_bringup(void)
{
    volatile uint32_t d;

    /* Enable the LCD and DMA modules before touching their registers. */

    if (!ROM_CLK_IS_ON(LCD_MODULE))
        ROM_CLK_ENABLE(LCD_MODULE);
    for (d = 0; d < 200000u; d++) ;

    /* The DMA module needs its own clock, and GRAM writes go through DMA.
     * The vendor's dma_open opens with exactly this (FIRM 0xd7d834):
     *     movs r0, #33 ; bl 0x2518   clk_is_on(0x21)
     *     movs r0, #33 ; bl 0x2564   clk_enable(0x21)
     * Without it the channel registers still accept writes - a gated module
     * reads back zeros or holds values but never runs - which is why every DMA
     * transfer timed out with the channel correctly programmed.
     *
     * Runs here, from SRAM, because clk_enable spins on an acknowledge bit and
     * must not be executed from flash. */
    if (!ROM_CLK_IS_ON(DMA_MODULE))
        ROM_CLK_ENABLE(DMA_MODULE);
    for (d = 0; d < 200000u; d++) ;

    /* Apply the vendor pixel clock after its source has been enabled. */
#define YP3_VENDOR_PIXEL_CLOCK 1
#if YP3_VENDOR_PIXEL_CLOCK
    /* This was the wrapper's own logic written out by hand - start source 8,
     * then select it - which is correct, and is also precisely the shape that
     * gets copied to a site where the pre-start is forgotten. Use the wrapper:
     * identical behaviour, and one less trap. */
    ROM_CLK_SET_SRC(LCD_CLOCK, 8);
    ROM_CLK_DIV(LCD_CLOCK, 6);
#endif
    ROM_CLK_APPLY(LCD_CLOCK);
    for (d = 0; d < 200000u; d++) ;

}

static void lcdc_hw_init(void)
{
    volatile uint32_t d;

    lcd_clock_bringup();
    for (d = 0; d < 50000; d++) ;

    /* Apply the vendor's LCDC register configuration. */
    LR(0x00) = 0;
    LR(0x08) &= ~1u;
    for (d = 0; d < 10000; d++) ;

    /* Apply the clock configuration from SRAM before programming the LCDC. */
    for (d = 0; d < 20000; d++) ;

    /* Configure the LCDC pad and bus timing registers from the vendor setup. */
    *(volatile uint32_t *)0x40081404u |= 0xffd0u;
    LR(0x10) = 0x00080000u;
    LR(0x40) = 0xabf1ffffu;
    LR(0x44) = 0x58764029u;
    for (d = 0; d < 10000; d++) ;

}

/* init sequence lifted verbatim from gc9106_lcd_init in the stock firmware.
 *
 * Split out of lcd_init_device for the stage bisect that found the port 3 pin 0
 * fault; kept separate because it is the panel's sequence, not the
 * controller's. */
static void lcd_panel_init_seq(void)
{
    static const unsigned char gamma_p[14] = {
        0x31, 0x4c, 0x24, 0x58, 0xa8, 0x26, 0x28,
        0x00, 0x2c, 0x0c, 0x0c, 0x15, 0x15, 0x0f
    };
    static const unsigned char gamma_n[14] = {
        0x0e, 0x2d, 0x24, 0x3e, 0x99, 0x12, 0x13,
        0x00, 0x0a, 0x0d, 0x0d, 0x14, 0x13, 0x0f
    };
    int i;

    lcd_panel_reset();
    lcd_write_cmd(0xfe); lcd_write_cmd(0xfe); lcd_write_cmd(0xfe);
    lcd_write_cmd(0xef);
    lcd_write_cmd(0xaf); wd8(0x80);
    lcd_write_cmd(0xb3); wd8(0x03);
    lcd_write_cmd(0xb6); wd8(0x11);
    lcd_write_cmd(0xac); wd8(0x1a);
    lcd_write_cmd(0xa3); wd8(0x11);
    lcd_write_cmd(0x21);
    lcd_write_cmd(0x36); wd8(0xd0);
    lcd_write_cmd(0x3a); wd8(0x05);
    lcd_write_cmd(0xb4); wd8(0x21);
    lcd_write_cmd(0xf0);
    for (i = 0; i < 14; i++)
        wd8(gamma_p[i]);
    lcd_write_cmd(0xf1);
    for (i = 0; i < 14; i++)
        wd8(gamma_n[i]);
    lcd_write_cmd(0xfe);
    lcd_write_cmd(0xff);
    lcd_write_cmd(0x35); wd8(0x00);
    lcd_write_cmd(0x44); wd8(0x00);
    lcd_write_cmd(0x11);
    udelay(120000);
    lcd_write_cmd(0x29);
    ROM_GPIO_CFG1(BL_PIN_ON_FROM_LCD);
}

/* Blank the panel on the way to power-off (HAVE_LCD_SHUTDOWN).
 *
 * Without this the shutdown splash stays lit while the device powers down,
 * which is indistinguishable from the hang that power_off() used to be. The two
 * commands are the mirror of the bring-up sequence above: DISPON/SLPOUT in,
 * DISPOFF/SLPIN out. */
void lcd_shutdown(void)
{
    lcd_write_cmd(0x28);                       /* DISPOFF */
    lcd_write_cmd(0x10);                       /* SLPIN */
}

void lcd_init_device(void)
{
    lcdc_hw_init();
    lcd_panel_init_seq();
}







/* Column +2, row +1: the GRAM is 132x162 (datasheet: 48,114 bytes for a
 * 128RGBx160 display) and the visible area is centred in it. Vendor 0x812cb0
 * adds exactly these offsets before CASET/RASET; we were writing to GRAM
 * columns 0..127 / rows 0..159, which is off the visible region by (2,1). */
#define LCD_COL_OFFSET  2
#define LCD_ROW_OFFSET  1

static void lcd_set_window(int x, int y, int w, int h)
{
    int x0 = x + LCD_COL_OFFSET, x1 = x + w - 1 + LCD_COL_OFFSET;
    int y0 = y + LCD_ROW_OFFSET, y1 = y + h - 1 + LCD_ROW_OFFSET;
    lcd_write_cmd(0x2a);
    wd8(x0 >> 8); wd8(x0 & 0xff); wd8(x1 >> 8); wd8(x1 & 0xff);
    lcd_write_cmd(0x2b);
    wd8(y0 >> 8); wd8(y0 & 0xff); wd8(y1 >> 8); wd8(y1 & 0xff);
}

/* --- pixels by DMA -----------------------------------------------------------
 *
 * The PIO path below sends ONE PIXEL per LCDC transaction and busy-waits each
 * one: 20,480 kick/wait pairs a frame. The sampling profiler (YP3_PROFILE,
 * 2026-09-29) put ~40% of all CPU time in lcdc_wait_idle/lcdc_kick and 0.1% in
 * the codec, which is why playback starves.
 *
 * The vendor sets the window by PIO and streams the pixels by DMA - its
 * transfer routine is SRAM 0x815aba(dev, cmd, cmdlen, buf, nbytes):
 *
 *   base[0x24] = cmd;  base[0x28] = nbytes - 1      nbytes <= 4096
 *   wait !(status & 0x60000000)
 *   base[0x10] = (base[0x10] & ~0x30) | 8           FIFO fed by DMA
 *   dma_start(tx, src = buf, dst = base + 0x34, nbytes)
 *   base[0x20] = (base[0x20] & ~0x30e) | ((cmdlen - 1) << 8) | 1
 *
 * The channel is the vendor's LCD TX channel as captured live in
 * probe/dma_work.bin and decoded in docs/DMA.md: channel 0, control
 * 0x001a0289 (16-bit memory source, 32-bit destination, request line 9),
 * length config 0x13000000, config 0x00080002.
 *
 * This path was abandoned on 2026-08-21 ("the DMA channel does not transfer"),
 * before the DMA engine was decoded for audio and before the panel itself
 * worked. It has not been tried on a working display before this.
 *
 * Pixels are byte-swapped into SRAM bounce buffers, exactly as the PIO path
 * swaps each pixel: DMA sources must be SRAM anyway (CLAUDE.md), and the swap
 * of chunk k+1 overlaps the transfer of chunk k.
 *
 * While a chunk is in flight the CPU YIELDS, which is the point: the codec
 * runs while the panel is fed. Only from thread context with interrupts on and
 * the kernel ticking; anywhere else it spins, bounded, as the PIO path does.
 * A chunk that has not finished in 100 ms is logged, the channel stopped, and
 * the rest of the boot falls back to PIO - a bad DMA path costs a log line,
 * not a dead display.
 *
 * MEASURED 2026-09-29, and it is OFF by default: the panel kept showing the
 * last PIO frame. The first-chunk dump says the channel accepts GO and clears
 * it, but its length register never leaves 0x1000, and the LCDC raises DONE
 * within ~2000 loop iterations of the kick - far too soon for 4 KB on an
 * 8-bit bus. That is the 2026-08-21 result again: the LCDC finishes an empty
 * transaction because it never raises request line 9. The channel descriptor
 * matches the vendor's (FIRM 0xd7e044) field for field and the LCDC's
 * +0x10/+0x40/+0x44 match stock, so the missing enable is somewhere in the
 * vendor's LCD open path, not in anything this file already writes. */
#ifndef YP3_LCD_DMA
#define YP3_LCD_DMA 0
#endif
#define LCD_DMA_CH          0
#define LCD_DMA_CTRL        0x001a0289u
#define LCD_DMA_LENCFG      0x13000000u
#define LCD_DMA_CFG         0x00080002u
#define LCDC_FIFO           (LCDC_BASE + 0x34)
#define LCD_CHUNK_PIX       2048                /* 4096 bytes, the vendor cap */

#if YP3_LCD_DMA
static uint16_t lcd_bounce[2][LCD_CHUNK_PIX] __attribute__((aligned(4)));
#else
static uint16_t lcd_bounce[2][1];   /* 8 KB of SRAM is not spent on a path that is off */
#endif
static bool lcd_dma_ok = YP3_LCD_DMA;
static bool lcd_dma_ready;
static unsigned long lcd_dma_frames, lcd_pio_frames;
static long lcd_dma_t0, lcd_pio_t0;

static void lcd_dma_open(void)
{
    DMA_CTRL(LCD_DMA_CH) &= ~DMA_CTRL_GO;
    DMA_CTRL(LCD_DMA_CH)  = LCD_DMA_CTRL;
    DMA_LEN(LCD_DMA_CH)   = LCD_DMA_LENCFG;
    DMA_IRQST(LCD_DMA_CH) = DMA_IRQST(LCD_DMA_CH);
    DMA_CFG(LCD_DMA_CH)   = (DMA_CFG(LCD_DMA_CH) & 0x3fffu) | LCD_DMA_CFG;
    lcd_dma_ready = true;
}

/* Thread context, interrupts on, kernel ticking: the only place yield() is
 * safe. lcd_update is also reached from panicf and early boot. */
static bool lcd_may_yield(void)
{
    uint32_t ipsr, primask;

    __asm__ volatile ("mrs %0, ipsr" : "=r"(ipsr));
    __asm__ volatile ("mrs %0, primask" : "=r"(primask));
    return ipsr == 0 && primask == 0 && current_tick > HZ;
}

/* Swap n pixels of the rect starting at pixel index `first` into dst. */
static void LCDICODE lcd_fill_bounce(uint16_t *dst, int x, int y, int w,
                                     int first, int n)
{
    int r = first / w, c = first % w;
    const fb_data *p = FBADDR(x + c, y + r);

    while (n--) {
        unsigned v = *p++;
        *dst++ = (uint16_t)(((v & 0xff) << 8) | (v >> 8));
        if (++c == w) {
            c = 0;
            r++;
            p = FBADDR(x, y + r);
        }
    }
}

static bool lcd_dma_wait(bool yield_ok)
{
    long deadline = current_tick + HZ / 10;
    uint32_t spins = 0;

    while (!(LCDC_STATUS & LCDC_ST_DONE)) {
        if (yield_ok) {
            if (TIME_AFTER(current_tick, deadline))
                return false;
            yield();
        } else if (++spins > 4u * LCDC_SPIN) {
            return false;
        }
    }
    LCDC_STATUS = LCDC_ST_DONE;
    DMA_IRQST(LCD_DMA_CH) = DMA_IRQST(LCD_DMA_CH);
    return true;
}

/* One-shot record of the first chunk: does the channel MOVE bytes? On
 * 2026-08-21 the length register never decremented and DONE was the LCDC
 * finishing an empty transaction; today the panel again shows a stale image
 * while every chunk reports DONE. dma_get_remaining (SRAM 0x814d1c) is
 * ch[0x0c] & 0x3ffff, so a moving channel counts down. */
static bool lcd_dma_dumped;
static uint32_t lcd_dd[12];

static bool lcd_update_rect_dma(int x, int y, int w, int h)
{
    int total = w * h, sent = 0, k = 0;
    bool yield_ok = lcd_may_yield();

    if (!lcd_dma_ready)
        lcd_dma_open();

    lcd_set_window(x, y, w, h);
    lcd_fill_bounce(lcd_bounce[0], x, y, w, 0,
                    total < LCD_CHUNK_PIX ? total : LCD_CHUNK_PIX);

    while (sent < total) {
        int n = total - sent;
        unsigned nbytes;

        if (n > LCD_CHUNK_PIX)
            n = LCD_CHUNK_PIX;
        nbytes = (unsigned)n * 2u;

        LCDC_CMD = sent ? 0x3c : 0x2c;          /* RAMWR, then continue */
        LCDC_DATALEN = nbytes - 1u;
        lcdc_wait_idle();
        LCDC_CFG10 = (LCDC_CFG10 & ~0x30u) | 8u;

        DMA_DST(LCD_DMA_CH) = LCDC_FIFO;
        DMA_SRC(LCD_DMA_CH) = (uint32_t)lcd_bounce[k];
        DMA_LEN(LCD_DMA_CH) = (DMA_LEN(LCD_DMA_CH) & ~DMA_LEN_MASK) | nbytes;
        DMA_CTRL(LCD_DMA_CH) = (DMA_CTRL(LCD_DMA_CH) & ~DMA_CTRL_CONT)
                             | DMA_CTRL_GO;
        if (!lcd_dma_dumped) {
            lcd_dd[0] = DMA_LEN(LCD_DMA_CH);
            lcd_dd[1] = DMA_CTRL(LCD_DMA_CH);
            lcd_dd[2] = DMA_IRQST(LCD_DMA_CH);
            lcd_dd[3] = LCDC_STATUS;
            lcd_dd[4] = LCDC_CFG10;
        }
        LCDC_CTRL = (LCDC_CTRL & ~0x30cu & ~0x2u) | 1u;     /* cmdlen 1 */
        if (!lcd_dma_dumped) {
            volatile uint32_t d;
            for (d = 0; d < 2000u; d++) ;
            lcd_dd[5] = DMA_LEN(LCD_DMA_CH);        /* shortly after the kick */
            lcd_dd[6] = LCDC_DATALEN;
            lcd_dd[7] = LCDC_STATUS;
        }

        /* Prepare the next chunk while this one is on the wire. */
        if (sent + n < total) {
            int m = total - sent - n;
            lcd_fill_bounce(lcd_bounce[k ^ 1], x, y, w, sent + n,
                            m < LCD_CHUNK_PIX ? m : LCD_CHUNK_PIX);
        }

        if (!lcd_dma_wait(yield_ok)) {
            DMA_CTRL(LCD_DMA_CH) &= ~(DMA_CTRL_GO | DMA_CTRL_CONT);
            logf("lcd: DMA chunk timed out at %d/%d px, st=%08lx dma ctrl=%08lx "
                 "len=%08lx irq=%08lx - PIO from now on", sent, total,
                 (unsigned long)LCDC_STATUS, (unsigned long)DMA_CTRL(LCD_DMA_CH),
                 (unsigned long)DMA_LEN(LCD_DMA_CH),
                 (unsigned long)DMA_IRQST(LCD_DMA_CH));
            lcd_dma_ok = false;
            return false;
        }
        if (!lcd_dma_dumped) {
            lcd_dd[8]  = DMA_LEN(LCD_DMA_CH);
            lcd_dd[9]  = DMA_CTRL(LCD_DMA_CH);
            lcd_dd[10] = LCDC_CTRL;
            lcd_dd[11] = LCDC_DATALEN;
            lcd_dma_dumped = true;
            logf("lcd dma: armed len=%08lx ctrl=%08lx irq=%08lx lst=%08lx cfg10=%08lx",
                 (unsigned long)lcd_dd[0], (unsigned long)lcd_dd[1],
                 (unsigned long)lcd_dd[2], (unsigned long)lcd_dd[3],
                 (unsigned long)lcd_dd[4]);
            logf("lcd dma: kicked len=%08lx dlen=%08lx lst=%08lx",
                 (unsigned long)lcd_dd[5], (unsigned long)lcd_dd[6],
                 (unsigned long)lcd_dd[7]);
            logf("lcd dma: done len=%08lx ctrl=%08lx lctrl=%08lx dlen=%08lx",
                 (unsigned long)lcd_dd[8], (unsigned long)lcd_dd[9],
                 (unsigned long)lcd_dd[10], (unsigned long)lcd_dd[11]);
        }
        sent += n;
        k ^= 1;
    }
    return true;
}

/* PIO redraws YIELD every 8 rows. A full frame takes ~69 ms of busy-waited
 * per-pixel transfers, and the codec thread shares priority 16 with main:
 * the 2026-09-29 beacon showed one MP3 frame decoded per ~1.5 s while main
 * held the CPU through redraws. Yielding mid-frame lets another priority-16
 * thread run while the panel window is half written - and the scroll and
 * backlight threads are priority 16 and draw - so redraws are serialised:
 * a second one waits (yielding) for the first to finish. */
static volatile bool lcd_xfer_busy;

static void LCDICODE lcd_update_rect_pio(int x, int y, int width, int height,
                                         bool yield_ok)
{
    int r, c;

    lcd_set_window(x, y, width, height);
    lcd_write_cmd(0x2c);
    for (r = 0; r < height; r++) {
        if (yield_ok && r && !(r & 7))
            yield();
        const fb_data *p = FBADDR(x, y + r);
        for (c = 0; c < width; c++) {
            unsigned v = *p++;
            /* One 2-byte transfer of the byte-swapped colour. This is exactly
             * what the vendor's fill at 0x812dc4 does: it byte-swaps
             * (r4 = (c << 8) | (c >> 8)) and calls the count == 2 data path
             * once per pixel. */
            lcd_write_data_n(((v & 0xff) << 8) | (v >> 8), 2);
        }
    }
}

void lcd_update_rect(int x, int y, int width, int height)
{
    bool full;

    if (x + width > LCD_WIDTH) width = LCD_WIDTH - x;
    if (y + height > LCD_HEIGHT) height = LCD_HEIGHT - y;
    if (width <= 0 || height <= 0) return;

    /* Full-frame timings, one number per path: the first 16 of each. */
    full = (width == LCD_WIDTH && height == LCD_HEIGHT);

    if (lcd_dma_ok && lcd_update_rect_dma(x, y, width, height)) {
        if (full && lcd_dma_frames < 16 && lcd_dma_frames++ == 0)
            lcd_dma_t0 = current_tick;
        else if (full && lcd_dma_frames == 16)
            logf("lcd: DMA, 15 full frames in %ld ticks",
                 current_tick - lcd_dma_t0), lcd_dma_frames++;
        return;
    }
    {
        bool yield_ok = lcd_may_yield();

        if (yield_ok)
            while (lcd_xfer_busy)
                yield();
        lcd_xfer_busy = true;
        lcd_update_rect_pio(x, y, width, height, yield_ok);
        lcd_xfer_busy = false;
    }
    if (full && lcd_pio_frames < 16 && lcd_pio_frames++ == 0)
        lcd_pio_t0 = current_tick;
    else if (full && lcd_pio_frames == 16)
        logf("lcd: PIO, 15 full frames in %ld ticks",
             current_tick - lcd_pio_t0), lcd_pio_frames++;
}

void lcd_update(void)
{
    lcd_update_rect(0, 0, LCD_WIDTH, LCD_HEIGHT);
}
