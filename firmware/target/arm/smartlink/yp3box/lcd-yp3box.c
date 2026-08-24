/* GC9106 128x160 RGB565 over the SL6801 LCDC at 0x40095000.
 * Register map and init sequence extracted from the stock firmware and verified
 * on hardware by drawing to the panel. */
#include "config.h"
#include "lcd.h"
#include "system.h"
#include "sl6801-regs.h"
#include "lcd-target.h"

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
    lcd_write_cmd(0x2c);
}

void LCDICODE lcd_update_rect(int x, int y, int width, int height)
{
    int r, c;

    if (x + width > LCD_WIDTH) width = LCD_WIDTH - x;
    if (y + height > LCD_HEIGHT) height = LCD_HEIGHT - y;
    if (width <= 0 || height <= 0) return;

    lcd_set_window(x, y, width, height);
    for (r = 0; r < height; r++) {
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

void lcd_update(void)
{
    lcd_update_rect(0, 0, LCD_WIDTH, LCD_HEIGHT);
}
