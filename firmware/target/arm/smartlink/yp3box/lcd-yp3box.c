/* GC9106 128x160 RGB565 over the SL6801 LCDC at 0x40095000.
 * Register map and init sequence extracted from the stock firmware and verified
 * on hardware by drawing to the panel. */
#include "config.h"
#include "lcd.h"
#include "system.h"
#include "sl6801.h"
#include "lcd-target.h"
#include "breadcrumb.h"

/* BOUNDED waits. Under ROM-only the LCDC reads CTRL=0, i.e. it is powered but not
 * configured the way the vendor firmware leaves it, and an unbounded spin on the
 * DONE bit hangs the whole boot. Count timeouts so we can see it from the outside. */
#define LCDC_SPIN 200000u

/* The per-transfer primitives run from SRAM. A full frame is 128*160*2 = 40960
 * transfers; fetching each one's instructions from SPI flash makes both the
 * bring-up fill and Rockbox's own lcd_update crawl. */
#define LCDICODE __attribute__((section(".icode"), noinline))

/* DIAGNOSTIC, see lcd_pattern_test() below. Set to 1 to paint a row-coded test
 * pattern instead of booting. */
#define YP3_LCD_PATTERN_TEST 0

/* DIAGNOSTIC: does ANY command reach the panel? See lcd_cmd_test().
 * Set to 1 to loop on the panel instead of booting. */
#define YP3_LCD_CMD_TEST 0

/* DIAGNOSTIC: write GRAM through the FIFO at 0x34, as stock does. See
 * lcd_fifo_test(). Superseded by the DMA test - the FIFO needs the DMA engine
 * to feed it, which is why hand-pushing words timed out. */
#define YP3_LCD_FIFO_TEST 0
/* DIAGNOSTIC: fill the panel red by DMA, then hold. Must be defined BEFORE the
 * call site in lcd_init_device, or the #if there silently evaluates to 0. */
#define YP3_LCD_DMA_TEST 0

/* DIAGNOSTIC: does ANY command still reach the panel? Blink the display off and
 * on, forever. See lcd_blink_test(). */
#define YP3_LCD_BLINK_TEST 0

/* DIAGNOSTIC: replay the sequence that is KNOWN to have drawn pixels on this
 * panel, as closely as our boot path allows. See lcd_minimal_test(). */
#define YP3_LCD_MINIMAL_TEST 0

/* DIAGNOSTIC: does a single PARAMETER byte land? Rotate the display 180 degrees
 * and back, using the stock image as the target. See lcd_param_test(). */
#define YP3_LCD_PARAM_TEST 0

/* DIAGNOSTIC: with the vendor pixel clock in place, fill red then rotate.
 * See lcd_pixclk_test(). */
#define YP3_LCD_PIXCLK_TEST 0

/* DIAGNOSTIC: is D/C stuck LOW, so that every data byte executes as a command?
 * See lcd_dc_test(). */
#define YP3_LCD_DC_TEST 0

/* DIAGNOSTIC: send a parameter in the SAME transaction as its command, the way
 * vendor 0x8159f0 does. See lcd_combined_test(). */
#define YP3_LCD_COMBINED_TEST 0

/* DIAGNOSTIC: does masking interrupts around a fill make pixels land?
 * ANSWERED - no. Ticks 164 -> 968 across the live phase, PRIMASK read back 1 in
 * the masked phase, both fills 21 transfers with zero timeouts and an identical
 * 1539-iteration kick spin, and the panel did not change either way. Interrupts
 * and SysTick preemption are eliminated. */
#define YP3_LCD_IRQ_TEST 0
#if YP3_LCD_IRQ_TEST
static void lcd_irq_test(void);
#endif

static void lcd_panel_init_seq(void);
#if YP3_LCD_COMBINED_TEST
static void lcd_combined_test(void);
#endif
#if YP3_LCD_DC_TEST
static void lcd_dc_test(void);
#endif
#if YP3_LCD_PIXCLK_TEST
static void lcd_pixclk_test(void);
#endif
#if YP3_LCD_PARAM_TEST
static void lcd_param_test(void);
#endif
#if YP3_LCD_MINIMAL_TEST
static void lcd_minimal_test(void);
#endif
#if YP3_LCD_BLINK_TEST
static void lcd_blink_test(void);
#endif
/* All defined below lcd_init_device; the display tests use them. Declared
 * unconditionally - they used to sit inside #if YP3_LCD_DMA_TEST, so turning
 * that test off took the declarations with it. */
static void lcd_set_window(int x, int y, int w, int h);
/* The vendor pixel path (0x815aba). LCDICODE on the declaration too, for the
 * same reason as below. */
static void LCDICODE lcd_dma_chunk(unsigned cmd, const void *src, unsigned nbytes);
#if YP3_LCD_DMA_TEST
/* LCDICODE on the declaration too: gcc rejects a section attribute that appears
 * only on the definition once the symbol has already been declared. */
static void LCDICODE lcd_dma_test(void);
#endif
#if YP3_LCD_FIFO_TEST
static void lcd_fifo_test(void);
#endif
#if YP3_LCD_CMD_TEST
static void lcd_cmd_test(void);
#endif
#if YP3_LCD_PATTERN_TEST
static void LCDICODE lcd_pattern_test(void);
#endif
volatile uint32_t lcd_timeouts, lcd_cmds_done;
/* PROBE: how long the last lcdc_kick actually spun before DONE came back.
 *
 * "Transfer completed" has never been worth anything on its own. A 4096-byte
 * transfer at a 24 MHz pixel clock cannot finish in the same handful of loop
 * iterations a parameterless command takes; if it does, the controller shifted
 * nothing out and the DONE bit is just an ack. Comparing the two counts settles
 * that without needing to trust the DMA registers. */
volatile uint32_t lcd_last_spin;

static void LCDICODE lcdc_wait_idle(void)
{
    uint32_t t = 0;
    while ((LCDC_STATUS & LCDC_ST_BUSY) && ++t < LCDC_SPIN) ;
    if (t >= LCDC_SPIN) lcd_timeouts++;
}

static void LCDICODE lcdc_kick(uint32_t ctrl)
{
    uint32_t t = 0;
    LCDC_CTRL = ctrl;
    while (!(LCDC_STATUS & LCDC_ST_DONE) && ++t < LCDC_SPIN) ;
    lcd_last_spin = t;
    if (t >= LCDC_SPIN) lcd_timeouts++; else lcd_cmds_done++;
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

/* Read from the panel - vendor 0x815932.
 *
 *   base[0x28] = n - 1
 *   wait !busy
 *   CTRL = (CTRL & ~0xe) | 9        <- bits 0 and 3, not 0xb
 *   wait done; STATUS = done
 *   return base[0x34]               <- the READ FIFO, not 0x30
 *
 * This ends the guessing about whether parameters land. Instead of setting
 * MADCTL and squinting at the panel, ask the panel what its MADCTL is. If it
 * reads back 0xd0 we set it; if it reads back the reset default, no parameter
 * has ever landed and the eye test was right.
 */
static unsigned LCDICODE lcd_read_n(unsigned cmd, unsigned n)
{
    lcd_write_cmd(cmd);
    LCDC_DATALEN = n - 1;
    lcdc_wait_idle();
    lcdc_kick((LCDC_CTRL & ~0xeu) | 0x9u);
    return LCDC_DATA_RD;
}

/* Peek at the transmit shift register WITHOUT sending a command first.
 * Reading 0x34 directly returns 0 - it only latches during a read transaction,
 * which is why the first attempt at this probe came back empty. */
static unsigned LCDICODE lcd_wire_peek(void)
{
    LCDC_DATALEN = 0;
    lcdc_wait_idle();
    lcdc_kick((LCDC_CTRL & ~0xeu) | 0x9u);
    return LCDC_DATA_RD;
}

/* Command WITH parameters, as one transaction - vendor 0x8159f0.
 *
 * Our standalone-data path (lcd_write_data_n, vendor 0x815902) completes and
 * emits nothing: proven on hardware by sending MADCTL 0xd0 / 0x10 alternately
 * and watching a display that never flipped, while parameterless commands in
 * the same loop toggled the panel every time.
 *
 * The vendor never sends a parameter that way. It pushes the data words into
 * the FIFO at 0x30 FIRST, then writes the command and the length, then kicks
 * CTRL with bits 8-9 = command length - 1 and the low bits = 3 - not 7 (command
 * only) and not 0xb (data only). Data is part of the command transaction, which
 * is presumably why a lone data kick goes nowhere.
 */
static void LCDICODE lcd_cmd_data(unsigned cmd, const unsigned char *d, unsigned n)
{
    unsigned i, words = (n + 3) >> 2;

    for (i = 0; i < words; i++) {
        unsigned w = 0, b;
        for (b = 0; b < 4; b++)
            if (i * 4 + b < n)
                w |= (unsigned)d[i * 4 + b] << (b * 8);
        LCDC_DATA = w;
    }
    LCDC_CMD = cmd;
    LCDC_DATALEN = n - 1;
    lcdc_wait_idle();
    /* command length is 1, so bits 8-9 stay 0 */
    lcdc_kick((LCDC_CTRL & ~0x30cu & ~0x2u) | 0x3u);
}

/* Controller + clock bring-up.
 *
 * Captured by diffing the register state with the stock firmware running (LCD
 * working) against ROM-only (LCD dead). Under ROM-only just 3 of 64 LCDC
 * registers are non-zero, so the vendor sets up considerably more than the panel
 * command sequence - which is why every transfer timed out.
 *
 * This replays the captured values rather than reverse engineering the vendor's
 * config-struct driven init at bootloader 0x827c68.
 */
#define CLK(o) (*(volatile uint32_t *)(0x40080000u + (o)))
#define LR(o)  (*(volatile uint32_t *)(LCDC_BASE + (o)))

/* Is the LCDC actually clocked? If the block is gated off, writes are dropped
 * and read back as zero - which is exactly what we observed. */
static int lcdc_writable(void)
{
    volatile uint32_t d;
    LR(0x28) = 0x0000039fu;   /* 0x28 is the transfer-length register: restore below */
    for (d = 0; d < 1000; d++) ;
    { int ok = (LR(0x28) == 0x0000039fu); LR(0x28) = 0; return ok; }
}

/* Boot ROM module clock API, recovered from the vendor bootloader's LCD init at
 * 0x827e88:
 *
 *     r0 = 0x5c ; bl 0x2518          ; is module 0x5c clocked?
 *     if (!r0) { r0 = 0x5c ; bl 0x2564 }   ; if not, enable it
 *
 * The same pattern appears for DMA (module 0x21) and in ROM main (0x46), so
 * 0x2518 = clk_is_enabled(id), 0x2564 = clk_enable(id). The LCD is module 0x5c.
 * This is why writing clock-controller registers by hand never worked: the ROM
 * owns that block and does more than set a single gate bit.
 */
#define DMA_MODULE     0x21u   /* FIRM 0xd7d834 */
#define LCD_MODULE     0x5cu

/* The vendor LCD init (bootloader 0x827e88) also configures a CLOCK, id 0x3f,
 * through three ROM helpers - the same trio the flash init uses for its clock:
 *     0x3118(id, src)   set source
 *     0x2d58(id, div)   set divider
 *     0x27a0(id)        apply / enable
 * Enabling the module (0x5c) without giving the controller a pixel clock is why
 * every transfer times out: the block is powered and writable but not running. */

/* Panel hardware reset, from the vendor's pre-init at FIRM 0xd66790:
 *     gpio_write(0x15000,1); delay(10);
 *     gpio_write(0x15000,0); delay(10);      RESX low
 *     gpio_write(0x15000,1); delay(50);      RESX high
 * Pin 0x15000 decodes as port 1 pin 10. It is configured at FIRM 0xd7a630 with
 * gpio_config(0x15000, 0x780).
 *
 * The GC9106 ignores every command until RESX is pulsed, which is why the LCDC
 * happily reported 20549 successful transfers with nothing on screen: the
 * controller was transmitting correctly to a panel that was not listening.
 * The vendor bootloader normally does this, but our boot path skips it - the ROM
 * jumps straight to us. */
/* Backlight pin, driven from here too: lcd_init_device runs before the backlight
 * thread exists, so this cannot be raced by backlight_update_state(). */
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

/* PORT 3 PIN 0 MUST BE RELEASED, OR NO PIXEL EVER REACHES THE PANEL.
 *
 * This is the bug. Found on hardware, not by reasoning: forcing the system
 * that was drawing red into our firmware's GPIO state one group
 * at a time, and pin 3.0 in mode 2 - alone, with every LCD pin still at mode 12
 * - turned it into our exact failure. Every transfer completes, zero timeouts,
 * and nothing appears.
 *
 * We never wrote those bits. The boot ROM does: it samples 3.0 as a boot strap
 * at 0x4c96a (gpio_read(0x00030000)) and then calls 0x4c0fc, a one-line function
 * that does gpio_config(0x0003011b) - port 3 pin 0, mode 2, pull 1, alt index
 * 0xb, which the table at ROM 0x9c4 expands to alt fields 2 and 2. That is
 * exactly the state our dumps showed, and it is reached only on the normal-boot
 * path. In card-reader mode the ROM takes a different branch and leaves 3.0
 * alone - which is why SIX payloads carrying progressively more of our code all
 * drew red, why the fault was present at the earliest point in our boot the LCD
 * can run at all, and why every pin-mux comparison came back clean: 3.0 is not
 * an LCD pin, so nobody was looking at it.
 *
 * Mode 15 with pull 0 and no alt function is what stock runs with - and stock
 * executes XIP from the same flash while doing so, so releasing this pad cannot
 * be what feeds our instruction fetch.
 */
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
    /* Real timings via udelay (which now runs from SRAM) rather than raw loop
     * counts, which mean nothing when the loop itself is fetched from flash.
     * GC9106 wants RESX low for at least 10us and roughly 120ms before commands;
     * these are generous but bounded. */
    lcd_pins_config();
    /* PROBE, slots 147..150: does RESX actually move?
     *
     * The panel is still displaying a COHERENT image left in GRAM by the stock
     * firmware, boot after boot. A hardware reset does not leave that intact,
     * so the most likely reading is that this pulse never reaches the pin - in
     * which case the panel has been running under stock's configuration all
     * along, including whatever window stock last set, and a frame written into
     * a small stale window would wrap inside it invisibly. That single fact
     * would explain the whole "commands land, pixels never appear" asymmetry
     * without needing anything exotic.
     *
     * Pin 1.10 is configured mode 1 (GPIO output) by lcd_pins[], and an output's
     * level reads back on the port input register, so the firmware can check its
     * own work: sample bit 10 of GPIO_IN(1) at each phase of the pulse. Three
     * identical readings mean the pin is not being driven. Slot 150 takes the
     * port-1 mode word covering pins 8-15 so a mis-muxed pad shows up too.
     *
     * Mode is 4 bits per pin, 8 pins per word: pin 10 lives in the word at
     * 0x40081040 + 4, nibble 2. */
#define RESX_LEVEL()  ((GPIO_IN(1) >> 10) & 1u)
    ROM_GPIO_WRITE(LCD_RESET_PIN, 1); udelay(1000);     /* 1ms  */
    BC_SLOT(147) = 0xE5000000u | RESX_LEVEL();
    ROM_GPIO_WRITE(LCD_RESET_PIN, 0); udelay(10000);    /* 10ms */
    BC_SLOT(148) = 0xE5000000u | RESX_LEVEL();
    ROM_GPIO_WRITE(LCD_RESET_PIN, 1); udelay(50000);    /* 50ms */
    BC_SLOT(149) = 0xE5000000u | RESX_LEVEL();
    BC_SLOT(150) = *(volatile uint32_t *)(0x40081040u + 4);
}
#define LCD_CLOCK      0x3fu

/* Does a real transfer actually complete? Issue a harmless NOP command. */
static int lcd_transfer_works(void)
{
    uint32_t t = 0;
    LCDC_CMD = 0x00;
    while ((LCDC_STATUS & LCDC_ST_BUSY) && ++t < 20000u) ;
    LCDC_CTRL = (LCDC_CTRL & ~0x30cu & ~0x2u) | 0x7u;
    t = 0;
    while (!(LCDC_STATUS & LCDC_ST_DONE) && ++t < 20000u) ;
    if (LCDC_STATUS & LCDC_ST_DONE) { LCDC_STATUS = LCDC_ST_DONE; return 1; }
    return 0;
}


/* ROM clock-frequency query, for diagnostics. */

/* MUST run from SRAM - see app.lds.
 *
 * ROM clk_enable (0x23bc for ids >= 64) is:
 *     [0x40080078] |= bit;  [0x40080068] |= bit;
 *     while (([0x40080068] & bit) == 0) ;
 *
 * It spins waiting for the clock to acknowledge. Executed from flash that is
 * exactly the hazard: if bringing the module up perturbs the flash controller,
 * the CPU cannot fetch the next instruction of the very loop it is running, and
 * stops forever. The SRAM build never hit this because all its code was in RAM.
 *
 * The ROM entry points themselves live in mask ROM at 0x0..0x5c000, so calling
 * them from here is fine; only OUR caller had to move.
 */
static void __attribute__((section(".icode"), noinline)) lcd_clock_bringup(void)
{
    volatile uint32_t d;

    /* The previous run proved this function COMPLETES and leaves the module on
     * at 24MHz - yet the marker immediately after the call, which lives in
     * flash, never runs. So the clock operation kills FLASH ACCESS, and every
     * earlier reading of "stalls at the LCDC register" was a dead instruction
     * fetch being misattributed.
     *
     * Probe flash from here (SRAM) after each step to find which one does it.
     * 0x00c0e000 is the vector table; a plausible non-zero word means flash is
     * still serving reads. */
    BC_SLOT(61) = 0x1C000001u;

    if (!ROM_CLK_IS_ON(LCD_MODULE))
        ROM_CLK_ENABLE(LCD_MODULE);
    for (d = 0; d < 200000u; d++) ;
    BC_SLOT(61) = 0x1C000002u;                         /* survived module enable */

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
    BC_SLOT(61) = 0x1C00000Au;                         /* DMA module clocked */

    /* The vendor's own pixel clock: clk_set_source(0x3f, 8),
     * clk_set_divider(0x3f, 6), clk_apply - FIRM 0xd7dfe0, cfg[0]=8 cfg[1]=6
     * from the struct at 0xd66640.
     *
     * This was tried before and STOPPED the LCDC: transfers went from 20549 to
     * 0 with STATUS frozen, because source 8 is itself a clock that the vendor
     * brings up in its full clock-tree init - which we did not run, so the LCDC
     * got a dead source.
     *
     * We run that init now (yp3_clock_init, bootloader 0x8206e4), and it applies
     * sources 8 and 9 explicitly. So the reason this failed no longer holds, and
     * the pixel rate is the one variable the panel might care about that we have
     * never actually matched.
     *
     * TRIED, AND IT STILL KILLS THE LCDC: 0 transfers completed, 58 timed out,
     * and the panel went pure white - which is what this panel looks like when
     * it is reset and then receives nothing at all. Source 8 measured 96MHz
     * (not the 192MHz the ROM leaves), giving a 16MHz pixel clock instead of the
     * vendor's 32MHz, so our clock init is also not reproducing the vendor's
     * tree faithfully: source 8 was 192MHz BEFORE yp3_clock_init and 96MHz
     * after, so we halve it somewhere.
     *
     * Useful negative result all the same: a white panel is the signature of
     * "no command reached the panel", which retroactively confirms that in the
     * working state the noise WAS the panel actively displaying GRAM after a
     * DISPON that landed. */
/* The vendor's pixel clock, source 8 divided by 6 - FIRM 0xd7dfe0, cfg[0]=8
 * cfg[1]=6 from the struct at 0xd66640.
 *
 * TRIED ONCE AND REVERTED, and the reason it failed no longer holds. That
 * attempt ran before the clock tree was brought up: source 8 was being handed to
 * the LCDC while it was still dead, the LCDC froze, and the panel went white.
 * yp3_clock_init now applies sources 8 and 9 exactly as bootloader 0x8206e4
 * does, and the firmware measures source 8 at 96 MHz before the LCD is touched.
 * 96/6 = 16 MHz.
 *
 * WHY THIS IS NOW THE PRIME SUSPECT. Commands reach the panel - proven twice,
 * by the blink test and by INVON restoring the stock image to normal - and no
 * data byte ever does. Every other explanation is exhausted: the LCDC registers
 * match the captured working state exactly, the twelve pins match FIRM 0xd66640 entry
 * for entry, the PIO and DMA primitives match vendor 0x8158d8 and 0x815902
 * instruction for instruction, and the panel bring-up is exonerated because
 * doing strictly less than it still draws nothing.
 *
 * What does NOT match is the rate those transfers are clocked at. 0x40 and 0x44
 * are sixteen 4-bit bus-timing fields, counted in PIXEL CLOCK cycles, and the
 * vendor's values pack fields of 0xf next to fields of 0, 1 and 2. We run them
 * at the 24 MHz the boot ROM happened to leave, 1.5x the vendor's 16 MHz, so
 * every strobe is 1.5x short. A field of 0xf still latches comfortably at that
 * rate; a field of 1 or 2 does not. That is exactly an asymmetry between two
 * phases of the same bus - which is the shape of the bug.
 *
 * The earlier all-0xf experiment is not evidence against this: it changed the
 * command fields too, and it ran at the same wrong clock.
 */
#define YP3_VENDOR_PIXEL_CLOCK 1
#if YP3_VENDOR_PIXEL_CLOCK
    /* APPLY SOURCE 8 BEFORE SELECTING IT. This is why both previous attempts at
     * the vendor pixel clock froze the LCDC and left the panel pure white.
     *
     * yp3_clock_init calls rom_2a7c(8) and rom_2a7c(9), transcribed faithfully
     * from bootloader 0x8206e4 - and 0x2a7c is clk_STOP, not clk_apply. FIRM
     * settles which is which: the codec enable path at 0xd7e844 starts clocks
     * 0x1f and 0x38 through 0x8051ec -> 0x27a0, and the disable path at
     * 0xd7e88e stops 0x38 through 0x8051f0 -> 0x2a7c.
     *
     * So the vendor stops sources 8 and 9 during its tree init, exactly as we
     * do - but it never selects one with the raw setter afterwards. Bootloader
     * 0x827e9c routes the LCD's source through 0x8206c2:
     *
     *     if (src == 8 || src == 9) clk_apply(src);
     *     clk_set_source(id, src);
     *
     * We called 0x3119 directly and handed the controller a stopped clock. The
     * rule is already written down in CLAUDE.md - "a source number is itself a
     * clock id... selecting a source can hand a peripheral a dead clock if that
     * source was never brought up" - and measuring source 8 at 96 MHz did not
     * contradict it, because get_clock_freq computes a rate from the dividers
     * whether or not the gate is open.
     */
    ROM_CLK_APPLY(8);                        /* vendor 0x8206c2 */
    BC_SLOT(151) = ROM_CLK_FREQ(8);          /* source 8 after applying it */
    ROM_CLK_SRC(LCD_CLOCK, 8);
    ROM_CLK_DIV(LCD_CLOCK, 6);
#endif
    ROM_CLK_APPLY(LCD_CLOCK);
    for (d = 0; d < 200000u; d++) ;
    BC_SLOT(61) = 0x1C000003u;                         /* survived clock apply */
    BC_SLOT(127) = ROM_CLK_FREQ(LCD_CLOCK);            /* the rate we ended up with */

    BC_SLOT(61) = 0x1C000004u;
}

static void lcdc_hw_init(void)
{
    volatile uint32_t d;

    /* UNIQUE step numbers. There used to be two "= 2" markers here, so a stall
     * anywhere between the bring-up and lcdc_writable() all reported as step 2
     * and I attributed it to lcdc_writable() without evidence. */
    BC_SLOT(60) = 1;
    lcd_clock_bringup();
    BC_SLOT(60) = 2;                 /* bring-up returned */
    for (d = 0; d < 50000; d++) ;
    BC_SLOT(60) = 3;                 /* settle delay done */
    BC_SLOT(18) = 0xC10C0000u | (ROM_CLK_IS_ON(LCD_MODULE) ? 1 : 0);
    BC_SLOT(60) = 4;                 /* second ROM_CLK_IS_ON returned */
    BC_SLOT(20) = 0xC0DE0000u | (lcdc_writable() ? 1 : 0);
    BC_SLOT(60) = 5;                 /* first LCDC register access survived */

    /* The vendor's LCDC register init is FIRM 0xd7e12c, and with the config
     * struct built at 0xd66640 (cfg[2] == 0, cfg[3..6] == 0) it reduces to
     * exactly two writes. Everything this function used to write to 0x10, 0x28,
     * 0x40, 0x44 and 0x20 was invented during blind probing - and 0x28 is the
     * transfer LENGTH register while 0x20 is CTRL, so those writes were actively
     * corrupting the data path. */
    BC_SLOT(60) = 6;
    LR(0x00) = 0;
    LR(0x08) &= ~1u;
    for (d = 0; d < 10000; d++) ;

    /* Pixel clock.
     *
     * The vendor does clk_set_source(0x3f, 8); clk_set_divider(0x3f, 6) - see
     * FIRM 0xd7dfe0 with cfg[0]=8, cfg[1]=6 from the struct at 0xd66640. Doing
     * the same here STOPS the LCDC: transfers went from 20549 completed to 0
     * completed, with STATUS frozen at 0x00010000 (neither BUSY nor DONE ever
     * moving) while CTRL still accepted writes.
     *
     * The reason is that a "source" number is itself a clock id. Source 8 is a
     * clock the vendor brings up in its full clock-tree init, which Rockbox
     * never runs, so selecting it hands the LCDC a dead source. Replaying that
     * whole init is what hung the device earlier.
     *
     * So we keep the source and divider the boot ROM already left running and
     * only apply - the configuration that demonstrably completes transfers. The
     * pixel rate is then whatever the ROM chose, which is fine for command
     * traffic; revisit only if the panel needs a specific rate for pixel data.
     */
    /* clock apply moved into lcd_clock_bringup(), which runs from SRAM */
    for (d = 0; d < 20000; d++) ;

    /* Bus timing and pad enable - FIRM 0xd66640 calls 0xd7e024 right after the
     * clock setup, and this was never ported.
     *
     * 0xd7e1c2 ORs a 16-bit mask into 0x40081404, and 0xd7e228 packs sixteen
     * 4-bit fields into LCDC 0x40 and 0x44. The values below are not guesses:
     * the config struct that 0xd66640 builds on its stack was replayed write for
     * write - including the later stores that overwrite part of the 0x0f block
     * at sp+22, sp+24 and sp+28 - and the packing routines applied to it. That
     * yields bytes 0f 0f 0f 0f 01 0f 0b 0a 09 02 00 04 06 07 08 05 and hence:
     *
     * Note 0x40 and 0x44 were previously written here with exactly these values
     * and then REMOVED as "invented during probing". The values were correct;
     * only the writes to 0x10, 0x28 and 0x20 were wrong (0x28 is the transfer
     * length register and 0x20 is CTRL, both set per transfer). Removing all
     * five together threw out two required writes along with three harmful ones.
     */
    BC_SLOT(60) = 7;
    *(volatile uint32_t *)0x40081404u |= 0xffd0u;   /* 0xd7e1c2 */
    /* 0x10 is the only LCDC register that differs between the captured working
     * state (stock firmware displaying) and ROM-only, and that we do not
     * replay: the working capture has 0x00080000, ROM-only has 0. 0x20 and 0x28
     * also differ but are per-transfer residue.
     *
     * Capture-derived, not read out of vendor code - the bootloader's own
     * lcdc_hw_init is not present in the available SRAM capture (that region reads as 0xa5a5
     * filler), so this change is not confirmed by disassembly. It was deleted
     * once before as "invented during probing"; the working-state capture says
     * otherwise. */
    LR(0x10) = 0x00080000u;
    /* 0x40/0x44 are the bus TIMING registers: sixteen 4-bit fields. The vendor's
     * values pack fields of 0xf (maximum) next to fields of 0, 1 and 2
     * (minimum). If the field governing the data phase is near zero while the
     * command phase is 0xf, commands would latch at the panel and data would
     * not - which is precisely the asymmetry we have been unable to explain, and
     * the one mechanism left that distinguishes the two.
     *
     * The vendor's values are also DERIVED, not read directly: a previous
     * session replayed the config struct that 0xd66640 builds on its stack and
     * applied the packing routine at 0xd7e228 to it. The packing checks out
     * (nibbles f,f,f,f,1,f,b,a -> 0xabf1ffff), but the struct contents are one
     * inference away from the disassembly.
     *
     * TEST: all fields maximum, i.e. the slowest possible bus. This is a known
     * valid state - it is exactly what the ROM leaves (both timing registers read
     * 0xffffffff). If parameters start landing, the fault is timing and we narrow
     * down field by field from here.
     *
     * TRIED: the panel went pure white, i.e. even commands stopped landing. So
     * these fields do gate the bus, but the vendor values are not wrong - the
     * firmware's own readback matches the captured working state exactly at both 0x40
     * and 0x44. Timing is ruled out as the command/data asymmetry. */
#define YP3_LCD_SLOW_BUS 0
#if YP3_LCD_SLOW_BUS
    LR(0x40) = 0xffffffffu;
    LR(0x44) = 0xffffffffu;
#else
    LR(0x40) = 0xabf1ffffu;                         /* 0xd7e228 */
    LR(0x44) = 0x58764029u;
#endif
    for (d = 0; d < 10000; d++) ;

    BC_SLOT(60) = 8;
    BC_SLOT(41) = ROM_CLK_FREQ(LCD_CLOCK);   /* actual pixel clock, in Hz */
    BC_SLOT(42) = ROM_CLK_FREQ(8);           /* is source 8 alive at all? */
    BC_SLOT(60) = 9;
    BC_SLOT(19) = 0x1CDC0000u;
}

/* init sequence lifted verbatim from gc9106_lcd_init in the stock firmware.
 *
 * Split out of lcd_init_device for the stage bisect that found the port 3 pin 0
 * fault; kept separate because it is the panel's sequence, not the
 * controller's. */
static void lcd_panel_init_seq(void)
{
    static const unsigned char gamma_p[14] =
        { 0x31,0x4c,0x24,0x58,0xa8,0x26,0x28,0x00,0x2c,0x0c,0x0c,0x15,0x15,0x0f };
    static const unsigned char gamma_n[14] =
        { 0x0e,0x2d,0x24,0x3e,0x99,0x12,0x13,0x00,0x0a,0x0d,0x0d,0x14,0x13,0x0f };
    int i;

    BC_SLOT(15) = 0xA0;
    lcd_panel_reset();
    BC_SLOT(15) = 0xA8;
    /* Unlock the extended command set: 0xfe THREE times, then 0xef.
     *
     * We sent 0xfe twice. github.com/nachus001/GC9106_driver sends it three
     * times, and that is the one difference in that reference we can act on
     * while parameters are still not landing - the unlock is commands only, so
     * unlike its 0xe6/0xe7 VREG writes or its MADCTL value it does not depend on
     * the very thing that is broken.
     *
     * Worth doing regardless of whether it is the fault. If the panel never
     * entered the extended set, then 0xaf, 0xb3, 0xb6, 0xac, 0xa3 and 0xb4 have
     * all been ignored outright, which changes what our init has actually been
     * doing to the panel. It cannot explain a dead data phase on its own -
     * CASET, RASET, MADCTL and COLMOD are standard MIPI commands outside the
     * extended set - so this is a correction, not a theory. */
    lcd_write_cmd(0xfe); lcd_write_cmd(0xfe); lcd_write_cmd(0xfe);
    lcd_write_cmd(0xef);                                            /* unlock */
    BC_SLOT(15) = 0xA1;
    lcd_write_cmd(0xaf); wd8(0x80);
    lcd_write_cmd(0xb3); wd8(0x03);
    lcd_write_cmd(0xb6); wd8(0x11);
    lcd_write_cmd(0xac); wd8(0x1a);
    lcd_write_cmd(0xa3); wd8(0x11);
    lcd_write_cmd(0x21);                       /* INVON */
    lcd_write_cmd(0x36); wd8(0xd0);            /* MADCTL */
    lcd_write_cmd(0x3a); wd8(0x05);            /* COLMOD = 16bpp */
    BC_SLOT(15) = 0xA2;
    lcd_write_cmd(0xb4); wd8(0x21);
    lcd_write_cmd(0xf0); for (i = 0; i < 14; i++) wd8(gamma_p[i]);
    lcd_write_cmd(0xf1); for (i = 0; i < 14; i++) wd8(gamma_n[i]);
    BC_SLOT(15) = 0xA3;
    lcd_write_cmd(0xfe);
    lcd_write_cmd(0xff);
    lcd_write_cmd(0x35); wd8(0x00);
    lcd_write_cmd(0x44); wd8(0x00);
    lcd_write_cmd(0x11);                       /* SLPOUT */
    BC_SLOT(15) = 0xA4;
    BC_SLOT(16) = lcd_cmds_done; BC_SLOT(17) = lcd_timeouts;
    udelay(120000);
    BC_SLOT(15) = 0xA5;
    lcd_write_cmd(0x29);                       /* DISPON */
    BC_SLOT(15) = 0xA6;

    /* The bring-up test fill has been removed. It cost 40960 transfers - seconds
     * when executed from flash - and show_logo_boot() runs at init line 54
     * anyway, so Rockbox draws its own logo through lcd_update(). If that appears
     * the whole path works; if it does not, the fill would not have told us more.
     */
    ROM_GPIO_CFG1(BL_PIN_ON_FROM_LCD);
    BC_SLOT(15) = 0xA7;
    BC_SLOT(16) = lcd_cmds_done;
    BC_SLOT(17) = lcd_timeouts;
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
    BC(6);
    BC_SLOT(11) = LCDC_STATUS;          /* before hw init */
    BC_SLOT(12) = LCDC_CTRL;
    lcdc_hw_init();
    BC_SLOT(13) = LCDC_STATUS;          /* after hw init */
    BC_SLOT(14) = LCDC_CTRL;

    lcd_panel_init_seq();
    BC(8);                                     /* LCD init finished */
#if YP3_LCD_PATTERN_TEST
    lcd_pattern_test();
#endif
#if YP3_LCD_CMD_TEST
    lcd_cmd_test();
#endif
#if YP3_LCD_FIFO_TEST
    lcd_fifo_test();
#endif
#if YP3_LCD_DMA_TEST
    lcd_dma_test();
#endif
#if YP3_LCD_BLINK_TEST
    lcd_blink_test();
#endif
#if YP3_LCD_MINIMAL_TEST
    lcd_minimal_test();
#endif
#if YP3_LCD_PARAM_TEST
    lcd_param_test();
#endif
#if YP3_LCD_PIXCLK_TEST
    lcd_pixclk_test();
#endif
#if YP3_LCD_DC_TEST
    lcd_dc_test();
#endif
#if YP3_LCD_COMBINED_TEST
    lcd_combined_test();
#endif
#if YP3_LCD_IRQ_TEST
    lcd_irq_test();
#endif
}

#if YP3_LCD_IRQ_TEST
/* The one thing no payload has ever modelled: Rockbox's own runtime.
 *
 * Six payloads run under stock firmware - register protocol, pin config, RESX
 * pulse, init sequence, LCD clock bring-up, system clock tree, crt0's flash
 * window mask, lcdc_hw_init - all drew red. Every line of the display driver is
 * the display driver is therefore correct, and the hardware is fine. What
 * differs between a payload that draws and our firmware that does not is the
 * environment the same code runs in:
 *
 *   - interrupts and SysTick, firing throughout our transfers
 *   - the memory layout: .data/.bss at 0x806000+, the stack, pluginbuf
 *   - XIP - our callers fetch instructions from SPI flash; payloads run from SRAM
 *
 * Interrupts are the only one testable without restructuring the build, and they
 * have a plausible mechanism. An LCDC transaction is not atomic: we write CMD,
 * DATA and DATALEN, then kick CTRL, then spin on the DONE bit. A SysTick
 * exception landing between the DATA write and the kick, or between the kick and
 * the completion, runs the scheduler - and if anything in that path touches the
 * LCDC, or simply lets the controller's transaction time out and drop CS, the
 * parameter is lost while the opcode still executes. That is the exact asymmetry
 * this port has been chasing: commands land, parameters do not.
 *
 * The probe alternates two identical full-screen DMA fills that differ only in
 * whether interrupts are masked:
 *
 *   phase A   PRIMASK set, SysTick counter stopped, fill RED,  hold 4 s
 *   phase B   interrupts restored,                  fill BLUE, hold 4 s
 *
 * and what the user reports is one of three pictures:
 *
 *   alternating red/blue   both fills land - interrupts are not the fault, and
 *                          something in the *caller* path is (this is the fill
 *                          the driver has never been able to make land, so this
 *                          outcome is itself a large result)
 *   steady red             only the masked fill lands - INTERRUPTS ARE THE
 *                          FAULT; the fix is to mask around every transaction
 *   stock's image, static  neither lands - interrupts are eliminated, and what
 *                          is left is the memory layout and the XIP context
 *
 * Slot 166 makes the null result meaningful: systick_handler counts its own
 * entries, so if the tick is not actually firing during phase B then "no
 * difference" says nothing at all and this test has to be re-run with the tick
 * proven alive. Every previous negative in this port that could not distinguish
 * "the mechanism is absent" from "the mechanism did nothing" turned out to be a
 * false negative.
 *
 * The fills go through lcd_dma_chunk, the vendor's own pixel path (0x815aba),
 * with the window set by PIO exactly as vendor 0x812cb0 does.
 */
#define SYST_CSR_REG   (*(volatile uint32_t *)0xE000E010)

static void LCDICODE lcd_irq_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

/* One full frame: window by PIO, then RAMWR plus nine write-memory-continues.
 * buf MUST be the .bss array below - a const or literal source is flash, and
 * pointing the DMA engine at the aperture the CPU fetches from wedges the bus. */
static void LCDICODE lcd_irq_fill(unsigned short *buf, unsigned short swapped)
{
    unsigned i;

    for (i = 0; i < 2048; i++)
        buf[i] = swapped;

    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_dma_chunk(0x2c, buf, 4096);              /* RAMWR */
    for (i = 0; i < 9; i++)
        lcd_dma_chunk(0x3c, buf, 4096);          /* write-memory-continue */
}

static void lcd_irq_test(void)
{
    static unsigned short px[2048];              /* .bss = SRAM. NEVER const. */
    unsigned n = 0;

    BC_SLOT(46) = (uint32_t)px;                  /* 0x8xxxxx = SRAM, ok */
    BC_SLOT(116) = 0x19E00000u;
    BC_SLOT(160) = SYST_CSR_REG;                 /* SysTick as the kernel left it */
    BC_SLOT(161) = BC_SLOT(166);                 /* ticks seen before we start */

    while (1) {
        /* ---- phase A: interrupts masked ---- */
        disable_irq();
        SYST_CSR_REG = 0;                        /* stop the counter as well, so
                                                  * nothing is pending to flood
                                                  * the moment we unmask */
        {
            uint32_t primask;
            __asm__ volatile ("mrs %0, primask" : "=r"(primask));
            BC_SLOT(162) = primask;              /* 1 = masking really took */
        }
        BC_SLOT(116) = 0x19E00001u;
        lcd_irq_fill(px, 0x00f8);                /* 0xf800 red, byte-swapped */
        BC_SLOT(163) = lcd_cmds_done;
        BC_SLOT(164) = lcd_timeouts;
        BC_SLOT(165) = lcd_last_spin;            /* spin for a masked 4 KB DMA */
        lcd_irq_delay(4000);

        /* ---- phase B: interrupts live ---- */
        SYST_CSR_REG = BC_SLOT(160);             /* restore the kernel tick */
        enable_irq();
        BC_SLOT(116) = 0x19E00002u;
        lcd_irq_fill(px, 0x1f00);                /* 0x001f blue, byte-swapped */
        BC_SLOT(167) = lcd_cmds_done;
        BC_SLOT(168) = lcd_timeouts;
        BC_SLOT(169) = lcd_last_spin;            /* spin for an unmasked 4 KB DMA */
        BC_SLOT(170) = BC_SLOT(166);             /* ticks taken by now - if this
                                                  * does not move, phase B was
                                                  * not actually different and
                                                  * the result means nothing */
        lcd_irq_delay(4000);

        BC_SLOT(117) = ++n;
        BC_SLOT(118) = lcd_timeouts;
        BC_SLOT(119) = lcd_cmds_done;
    }
}
#endif

#if YP3_LCD_COMBINED_TEST
/* Send the parameter in the SAME transaction as the command.
 *
 * Every parameter this port has ever sent went out as two separate LCDC
 * transactions: lcd_write_cmd kicks CTRL with |7, then lcd_write_data_n kicks it
 * again with |0xb. Between those two kicks the controller finishes a
 * transaction, and if it drops CS in the gap, a controller that resets its
 * parameter pointer on a CS edge discards the argument. The opcode still
 * executes - which is precisely the asymmetry: commands land, their parameters
 * evaporate.
 *
 * The vendor has a second primitive for exactly this shape, at 0x8159f0, and it
 * is ONE transaction:
 *
 *     for each word of data:  base[0x30] = word     data FIRST
 *     base[0x24] = cmd
 *     base[0x28] = datalen - 1
 *     wait !busy
 *     CTRL = (CTRL & ~0x30c & ~2) | ((cmdlen - 1) << 8) | 3
 *
 * Note the kick is |3, not |7 and not |0xb - both the command bit and the data
 * bit set, in a single transaction, with the command length in bits 8-9.
 *
 * lcd_cmd_data() has been sitting in this file implementing that faithfully and
 * has never been called; the compiler has been warning about it. The docs say
 * MADCTL was "sent three different ways" and none flipped the display, but every
 * one of those attempts was judged against a panel showing random GRAM noise,
 * where a flip is invisible. That verdict is no more reliable than the two other
 * noise-target conclusions this session has already overturned.
 *
 * So: MADCTL 0x00 against 0xc0, two seconds apart, through the combined path,
 * against a coherent image.
 *
 *     image rotates -> parameters must share a transaction with their command,
 *                      and lcd_set_window / the whole data path can be rebuilt
 *                      on lcd_cmd_data
 *     image static  -> the combined path fails too, and the fault is not in how
 *                      the transaction is framed
 */
static void LCDICODE lcd_combined_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

static void lcd_combined_test(void)
{
    static const unsigned char madctl_normal[1] = { 0x00 };
    static const unsigned char madctl_flip[1]   = { 0xc0 };
    unsigned n = 0;

    BC_SLOT(116) = 0xC0DA0000u;
    while (1) {
        lcd_cmd_data(0x36, madctl_normal, 1);
        lcd_combined_delay(2000);
        lcd_cmd_data(0x36, madctl_flip, 1);
        lcd_combined_delay(2000);
        BC_SLOT(117) = ++n;
        BC_SLOT(118) = lcd_timeouts;
        BC_SLOT(119) = lcd_cmds_done;
    }
}
#endif

#if YP3_LCD_DC_TEST
/* Is D/C stuck LOW, so that every data byte is executed as a COMMAND?
 *
 * This is the last mechanism that fits, and it fits everything:
 *
 *   commands land                D/C low is correct for them
 *   no parameter ever lands      MADCTL 0xc0 executes as command 0xc0, which
 *                                does nothing visible - hence no rotation
 *   no window is ever set        CASET's four bytes execute as commands
 *   pixels never reach GRAM      a pixel stream executes as a command stream
 *   a black bar once appeared    a pixel stream contains 0x12 (partial mode on)
 *                                and 0x30 (partial area), which this port saw
 *   the LCDC takes real time     it is transmitting perfectly; the bytes are
 *                                simply being interpreted as opcodes
 *
 * It was "ruled out" in round 4 of the old command test - MADCTL parameters of
 * 0x28 and 0x29 supposedly failed to blank the panel. That test ran against a
 * screen of random GRAM noise, on the old clock tree, in a build that predates
 * everything since. Two other conclusions drawn against that same noise target
 * have now been shown untestable: MADCTL "never flipped the display" and PTLAR
 * "never moved a band" could not have shown a positive result either way.
 *
 * The target is different now. The panel holds a coherent image and DISPOFF
 * blanks it visibly - the blink test proved exactly that. So send DISPOFF and
 * DISPON as DATA bytes, with no command in front of them:
 *
 *     screen blinks    D/C is stuck low. Data bytes are being executed as
 *                      commands, and that single fault explains every
 *                      observation this port has collected.
 *     screen static    D/C is genuinely fine, and the data phase is putting
 *                      bytes on a wire the panel is not reading.
 *
 * A bare data byte with no command in front of it is harmless either way: if
 * D/C works it is a parameter to nothing.
 */
static void LCDICODE lcd_dc_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

static void lcd_dc_test(void)
{
    unsigned n = 0;

    BC_SLOT(116) = 0x0DC00000u;
    while (1) {
        lcd_write_data_n(0x28, 1);      /* DISPOFF, sent as DATA */
        lcd_dc_delay(2000);
        lcd_write_data_n(0x29, 1);      /* DISPON, sent as DATA  */
        lcd_dc_delay(2000);
        BC_SLOT(117) = ++n;
        BC_SLOT(118) = lcd_timeouts;
        BC_SLOT(119) = lcd_cmds_done;
    }
}
#endif

#if YP3_LCD_PIXCLK_TEST
/* Two answers from one flash, now that the bus runs at the vendor's rate.
 *
 * First a full-screen red fill through the PIO path, then MADCTL rotation
 * forever. Watch for both:
 *
 *   screen turns red     the data phase works - the timings were being clocked
 *                        too fast and pixel bytes were never latching
 *   then rotates 180     parameters land too, so the whole data phase is fixed
 *   red but no rotation  bulk data latches and single parameters still do not
 *   neither              the pixel clock was not the fault; slots 118/119 say
 *                        whether the LCDC survived the change at all
 *
 * Slot 127 carries the pixel rate the firmware measures after the change, so a
 * dead LCDC can be told from a wrong rate.
 */
static void LCDICODE lcd_pixclk_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

static void lcd_pixclk_test(void)
{
    unsigned n = 0;
    int i;

    BC_SLOT(116) = 0x91C10001u;
    lcd_write_cmd(0x2a);
    wd8(0); wd8(0); wd8(0); wd8(LCD_WIDTH - 1);
    lcd_write_cmd(0x2b);
    wd8(0); wd8(0); wd8(0); wd8(LCD_HEIGHT - 1);
    lcd_write_cmd(0x2c);
    for (i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++)
        lcd_write_data_n(0x00f8, 2);          /* red, pre-swapped */
    BC_SLOT(116) = 0x91C10002u;
    BC_SLOT(118) = lcd_timeouts;
    BC_SLOT(119) = lcd_cmds_done;

    while (1) {
        lcd_write_cmd(0x36); wd8(0x00);
        lcd_pixclk_delay(2000);
        lcd_write_cmd(0x36); wd8(0xc0);
        lcd_pixclk_delay(2000);
        BC_SLOT(117) = ++n;
    }
}
#endif

#if YP3_LCD_PARAM_TEST
/* Does a single PARAMETER byte reach the panel?
 *
 * This has been "answered" twice before and both answers are suspect, because
 * both were run against a panel showing random GRAM noise. MADCTL was alternated
 * 0xd0 / 0x10 and "never flipped the display" - but flipping noise looks exactly
 * like noise. PTLAR was given two row ranges and "never moved a band" - against
 * noise, again.
 *
 * The target has changed. The panel now holds a coherent image the stock
 * firmware left in GRAM, and it survives our reset. A 180 degree rotation of a
 * real picture is impossible to mistake, and impossible to confuse with a
 * stalled loop.
 *
 * MADCTL (0x36) takes ONE byte. MY|MX = 0xc0 rotates the display; 0x00 is
 * normal. Nothing else in this test needs a parameter, a window, GRAM or DMA.
 *
 *     image rotates   -> parameters DO land, and the fault is specific to GRAM
 *                        writes rather than to the data phase as a whole
 *     image static    -> parameters really do not land, now established against
 *                        a target where the previous two attempts could not
 *                        have shown it either way
 *
 * Slot 117 counts cycles so a static image is still distinguishable from a
 * stalled loop - the flaw in round 3 of the old command test.
 */
static void LCDICODE lcd_param_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

static void lcd_param_test(void)
{
    unsigned n = 0;

    BC_SLOT(116) = 0x9A2A0000u;
    while (1) {
        lcd_write_cmd(0x36); wd8(0x00);     /* MADCTL normal      */
        lcd_param_delay(2000);
        lcd_write_cmd(0x36); wd8(0xc0);     /* MADCTL MY|MX = 180 */
        lcd_param_delay(2000);
        BC_SLOT(117) = ++n;
        BC_SLOT(118) = lcd_timeouts;
        BC_SLOT(119) = lcd_cmds_done;
    }
}
#endif

#if YP3_LCD_MINIMAL_TEST
/* Replay the one sequence that has ever put pixels on this panel.
 *
 * The blink test settled that commands DO still reach the panel on the current
 * machine, so the fault is in the data phase. Pixels HAVE been drawn on this
 * hardware once: a card-reader experiment called the vendor's own helpers while
 * the stock firmware was running.
 *
 * Look at what that sequence did NOT do:
 *
 *     no panel reset          no unlock (0xfe/0xfe/0xef)
 *     no gamma                no COLMOD, no MADCTL
 *     no +2 / +1 window offsets - plain 0..W-1, 0..H-1
 *
 *     wc(0x29)                                    DISPON
 *     wc(0x2a) wd(0,1) wd(0,1) wd(0,1) wd(W-1,1)  CASET
 *     wc(0x2b) wd(0,1) wd(0,1) wd(0,1) wd(H-1,1)  RASET
 *     wc(0x2c)                                    RAMWR
 *     for (i = 0; i < W*H; i++) wd(swap16(colour), 2)
 *
 * Our init does all of the omitted things, and every one of them depends on a
 * parameter landing. If parameters do not land, our init is really a stream of
 * bare opcodes with their arguments silently dropped, which could leave the
 * panel somewhere the vendor never puts it. The working sequence sidestepped
 * that by not configuring the panel at all: it inherited a configured panel.
 *
 * We cannot inherit that any more - RESX now demonstrably pulses, so the panel
 * is genuinely reset every boot - but we CAN stop doing the things the working
 * sequence did not do and find out whether our init is the thing breaking it.
 * If a red screen appears here, the fault is in our panel bring-up rather than
 * in the bus, and everything about "data never lands" was a consequence.
 *
 * Colour is pre-swapped: 0xF800 renders blue because the panel receives 0x00F8,
 * so red goes out as 0x00f8.
 */
static void lcd_minimal_test(void)
{
    int i;

    BC_SLOT(116) = 0x71110001u;
    lcd_write_cmd(0x29);                              /* DISPON */

    lcd_write_cmd(0x2a);                              /* CASET, no offset */
    wd8(0); wd8(0); wd8(0); wd8(LCD_WIDTH - 1);
    lcd_write_cmd(0x2b);                              /* RASET, no offset */
    wd8(0); wd8(0); wd8(0); wd8(LCD_HEIGHT - 1);
    lcd_write_cmd(0x2c);                              /* RAMWR */
    BC_SLOT(116) = 0x71110002u;

    for (i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        lcd_write_data_n(0x00f8, 2);                  /* red, pre-swapped */
        if ((i & 0x3ff) == 0)
            BC_SLOT(117) = i;
    }

    BC_SLOT(116) = 0x7111000Du;
    BC_SLOT(118) = lcd_timeouts;
    BC_SLOT(119) = lcd_cmds_done;
    while (1) ;
}
#endif

#if YP3_LCD_BLINK_TEST
/* Is the command path still alive AT ALL?
 *
 * Everything this port has reasoned about the display for several rounds rests
 * on one inherited finding: "commands land, parameters and pixels do not". That
 * was established in an earlier session under a different machine - flash at
 * 4 MHz, the old clock tree, a different init path - and has never been
 * re-checked since. It has been treated as fact and used to rule things in and
 * out ever since, including by probes that could only have been meaningful if
 * it were true.
 *
 * If NOTHING reaches the panel now, it would sit there displaying the image
 * stock left in GRAM - which is precisely what we see, and fits every single
 * observation as well as the asymmetry does. GRAM survives the reset pulse, so
 * a live panel that is simply not listening looks identical to a panel being
 * written in the wrong place.
 *
 * DISPOFF and DISPON take no parameters, need no window, need no GRAM and no
 * DMA. Blink them a second apart, forever:
 *
 *     screen blinks         commands land, and the fault really is in the data
 *                           path, as assumed
 *     screen sits unchanged nothing has been reaching the panel at all, and
 *                           every "commands work" conclusion since is void
 *
 * Slot 117 counts cycles so a static screen can still be told from a stalled
 * loop, which is the mistake round 3 of the old command test made. */
static void LCDICODE lcd_blink_delay(unsigned ms)
{
    while (ms--)
        udelay(1000);
}

static void lcd_blink_test(void)
{
    unsigned n = 0;

    BC_SLOT(116) = 0xB11B0000u;
    while (1) {
        lcd_write_cmd(0x28);            /* DISPOFF */
        lcd_blink_delay(1000);
        lcd_write_cmd(0x29);            /* DISPON  */
        lcd_blink_delay(1000);
        BC_SLOT(117) = ++n;
        BC_SLOT(118) = lcd_timeouts;
        BC_SLOT(119) = lcd_cmds_done;
    }
}
#endif

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

/* DIAGNOSTIC: row-coded full-window fill, then halt.
 *
 * Six full 128x160 frames (20480 pixel writes each) were reported successful by
 * the transfer counter, yet only ~59 rows of the panel ever showed content and
 * the rest stayed at power-on noise. That is the controller accepting pixels and
 * discarding them, which means the addressing window is not what we think.
 *
 * This paints a pattern whose row index is readable off a photograph:
 *   rows 0-3     solid red        (marks framebuffer row 0)
 *   rows 156-159 solid blue       (marks framebuffer row 159)
 *   every 32nd   white line       (count them: 4 means the full height landed)
 *   otherwise    green gradient   (dark at the top, bright at the bottom)
 *
 * Then it halts, so the pattern stays on screen instead of being overwritten by
 * Rockbox's own frames. Set to 0 to boot normally. DELETE once this is answered.
 */
#if YP3_LCD_PATTERN_TEST
static void LCDICODE lcd_pattern_test(void)
{
    int r, c;
    BC_SLOT(77) = 0xF11E0000u;
    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
    for (r = 0; r < LCD_HEIGHT; r++) {
        unsigned v;
        if (r < 4)                  v = 0xf800;             /* red: row 0 */
        else if (r >= LCD_HEIGHT-4) v = 0x001f;             /* blue: last row */
        else if ((r & 31) == 0)     v = 0xffff;             /* white every 32 */
        else                        v = ((r * 63 / (LCD_HEIGHT-1)) & 0x3f) << 5;
        v = ((v & 0xff) << 8) | (v >> 8);   /* same primitive as lcd_update_rect */
        for (c = 0; c < LCD_WIDTH; c++)
            lcd_write_data_n(v, 2);
        BC_SLOT(78) = r + 1;                /* rows completed */
        BC_SLOT(79) = lcd_timeouts;
        BC_SLOT(80) = lcd_cmds_done;
    }
    BC_SLOT(77) = 0xF11EDEADu;
    while (1) ;                             /* hold the pattern on screen */
}
#endif

/* Snapshot the whole LCDC register block into breadcrumbs.
 *
 * An external diff cannot see our runtime state: the pinhole reset
 * re-initialises these blocks, so the firmware has to read itself back.
 *
 * Slots 81..104 are comparable against the working stock capture. */
#if YP3_LCD_CMD_TEST
/* Is the command path actually reaching the panel?
 *
 * This has been assumed all along and never tested. The reasoning was: the panel
 * shows GRAM noise, therefore SLPOUT and DISPON took effect, therefore commands
 * land. That is not sound - the noise appeared when we got the BACKLIGHT working
 * (GPIO 1.17), which merely revealed whatever the panel was already displaying.
 * If our RESX pulse misses and no command ever lands, the panel would simply
 * still be on from the stock firmware, showing stale GRAM, and look exactly like
 * what we see.
 *
 * These four commands take no parameters and have unmistakable visual effects,
 * so they test the command path alone, with no data phase and nothing to
 * photograph precisely - just watch the panel for ~8 seconds:
 *
 *   DISPOFF -> panel blanks
 *   DISPON  -> noise returns
 *   INVON   -> noise inverts (colours flip)
 *   INVOFF  -> noise returns to normal
 *
 * If the panel NEVER changes, no command is reaching it and the display bug is
 * pins/reset/bus, not protocol - which would explain why a byte-for-byte correct
 * LCDC and 124k successful transfers produce nothing.
 */
static void LCDICODE lcd_delay_ms(int ms)
{
    while (ms-- > 0)
        udelay(1000);
}

static void lcd_cmd_test(void)
{
    int phase = 0;

    /* ROUND 2. Round 1 proved the command path works: DISPOFF blanked the
     * panel, DISPON brought the noise back, and INVON flipped the black bar to
     * white - which also proves that bar is real GRAM content.
     *
     * So commands land. The open question is whether any DATA byte lands,
     * including command parameters, which go out through the same standalone
     * data path (vendor 0x815902) as our pixels. If parameters never land then
     * gamma, MADCTL, COLMOD, CASET and RASET have ALL been ignored since the
     * beginning, and the window was never set - which fits every observation.
     *
     * MADCTL (0x36) takes one parameter and has an unmistakable effect: MY|MX
     * flip the display, so the black bar moves between bottom and top and the
     * noise visibly mirrors.
     *
     *   phase 0: MADCTL 0xd0   bar where it is now
     *   phase 1: MADCTL 0x10   MY and MX cleared -> bar should JUMP to the other end
     *   phase 2: INVON         heartbeat - proves the loop is still running
     *   phase 3: INVOFF
     *
     * If 0 and 1 look identical while 2 and 3 visibly toggle, the command path
     * works and the data path does not, for parameters as well as pixels.
     */
    /* ANSWERED and removed: the panel readback (0x34 only echoes our own TX
     * byte), the four-variant wire probe (all four transmit, none change the
     * panel), and the attempt to call the vendor's own fill at 0x812dc4.
     *
     * That last one was INVALID and the result must not be believed: our
     * .data/.bss run from 0x806000 to 0x880000, so the HAL code at 0x812dc4,
     * its command table at 0x81d9cc and the device struct at 0x81e550 are all
     * overwritten by our own image. The struct read as zeros because it IS our
     * zeroed .bss, and the call hung because it branched into our data.
     * The captured SRAM image is a static artifact; none of that code is
     * callable at runtime under Rockbox.
     */

    while (1) {
        BC_SLOT(105) = 0xC3D70000u | (phase & 0xffff);
        switch (phase & 3) {
        /* ROUND 3: is D/C stuck, so that the panel reads our parameters as
         * commands?
         *
         * Everything now points that way. The bytes ARE driven onto the bus
         * (0x34 comes back holding the last byte we sent, for commands and
         * parameters alike), the twelve pin codes match the vendor's exactly,
         * the LCDC registers match the stock working capture - and yet
         * parameters have no effect while commands always work.
         *
         * So send a parameter whose value, read as a command, is unmistakable.
         * MADCTL takes one byte; 0x28 is DISPOFF and 0x29 is DISPON.
         *
         *   phase 0: MADCTL parameter 0x28 -> panel BLANKS if D/C is stuck low
         *   phase 1: MADCTL parameter 0x29 -> panel comes BACK
         *   phase 2/3: INVON/INVOFF heartbeat
         *
         * If the panel blinks off and on, every data byte we have ever sent was
         * executed as a command - which is why GRAM was never written, and why
         * a stream of pixel bytes eventually painted a black bar: it contains
         * 0x12 (partial mode on) and 0x30 (partial area). If it does NOT blink
         * while the heartbeat still toggles, D/C is fine and the panel is
         * genuinely ignoring well-formed parameters. */
        /* ROUND 4: the heartbeat is REMOVED on purpose.
         *
         * Round 3 was ambiguous - "it blinked" could have been the parameter
         * byte executing as DISPOFF/DISPON, or just the INVON/INVOFF heartbeat
         * inverting the noise. With inversion gone, the ONLY thing that can
         * change the panel is the parameter byte:
         *
         *   panel goes dark and comes back  -> D/C stuck, data runs as commands
         *   panel completely steady         -> D/C fine, parameters are being
         *                                      received and ignored
         *
         * Slot 105 counts phases, so a steady panel can still be told apart
         * from a stalled loop. */
        /* ROUND 5: partial mode - parameters that CHOOSE where the effect
         * lands.
         *
         * D/C is fine (round 4: the panel never blanked, so parameter bytes are
         * not being executed as commands) and the vendor sends parameters
         * exactly as we do - send_cmd(0xaf), send_data(0x80, 1) - so protocol,
         * pins and registers all match.
         *
         * PTLAR (0x30) takes four bytes defining a row range, and PTLON (0x12)
         * blanks everything outside it. That makes a parameter's VALUE visible:
         * the surviving band should sit where we put it.
         *
         *   phase 0: NORON            - if the black bar vanishes, the bar was
         *                               partial mode all along, triggered by
         *                               pixel bytes read as commands
         *   phase 1: rows 0..39   + PTLON  -> band at the TOP
         *   phase 2: rows 120..159 + PTLON -> band at the BOTTOM
         *   phase 3: NORON            - back to full display
         *
         * Band moves between top and bottom => parameters land, and the display
         * bug is confined to the RAMWR pixel stream.
         * Nothing ever changes => no parameter has ever reached the panel,
         * despite being correctly transmitted.
         */
        case 0:
        case 3:
            lcd_write_cmd(0x13);                  /* NORON */
            break;
        case 1:
            lcd_write_cmd(0x30);                  /* PTLAR 0..39 */
            wd8(0); wd8(0); wd8(0); wd8(39);
            lcd_write_cmd(0x12);                  /* PTLON */
            break;
        case 2:
            lcd_write_cmd(0x30);                  /* PTLAR 120..159 */
            wd8(0); wd8(120); wd8(0); wd8(159);
            lcd_write_cmd(0x12);                  /* PTLON */
            break;
        }
        BC_SLOT(106) = lcd_cmds_done;
        BC_SLOT(107) = lcd_timeouts;
        lcd_delay_ms(700);   /* ~2s real: everything runs ~3x slow at 12MHz AHB */
        phase++;
    }
}
#endif

/* Snapshot the GPIO mode registers for the two LCD ports, plus the pad register.
 *
 * The last unchecked part of the path. Every LCDC register now matches the
 * stock capture, the pin CODES match the vendor's config calls field for field -
 * but we have never compared the resulting GPIO hardware STATE against stock,
 * which remained an unchecked part of the bring-up.
 *
 * Mode is 4 bits per pin, 8 pins per word, base 0x40081000 + 0x40*port. */
/* DIAGNOSTIC: write GRAM through the FIFO at 0x34, the way stock actually does.
 *
 * Everything observable now matches stock - LCDC registers, all thirteen pin
 * modes, the pad register at 0x40081404 (0xffd0 in both) - and parameters still
 * never land. But we have never driven the path stock actually uses.
 *
 * A stock mid-transfer capture showed 0x28 = 0x39f: a 928-byte
 * transfer. The PIO path is capped at 4 bytes (the vendor checks n-1 > 3 and
 * bails), so stock writes GRAM exclusively through the FIFO at base+0x34, fed by
 * DMA (0x815aba -> 0x815a9a). The count==2 fill at 0x812dc4 that we copied may
 * be dead code the vendor never calls.
 *
 * Vendor 0x815aba, with the DMA replaced by PIO pushes into the same FIFO:
 *   base[0x24] = cmd            RAMWR
 *   base[0x28] = nbytes - 1
 *   wait !busy
 *   base[0x10] = (base[0x10] & ~0x30) | 8      <- only the DMA path does this
 *   <data into base[0x34]>
 *   CTRL = (CTRL & ~0x30c & ~2) | 1            <- 1, not 7 and not 0xb
 *
 * No window is set first, deliberately: CASET/RASET are parameters and those do
 * not land, so the panel keeps its reset default of the full screen. If this
 * works the panel fills from its origin and we get a red band. Any red at all
 * means GRAM is finally being written.
 */
/* GRAM writes by DMA - the path stock actually uses.
 *
 * The LCD device struct in the live stock firmware names the channel outright:
 *   0x81e550 +0x00 TX handle 0x81e430 -> regs 0x40001000 = DMA channel 0
 *            +0x0c RX handle 0x81e440 -> regs 0x40001040 = channel 1
 *            +0x18 LCDC base 0x40095000
 *
 * Channel registers (vendor 0x814c94), 0x40001000 + ch*0x40:
 *   +0x00 control      bit30 = start, bit29 cleared on start
 *   +0x04 destination  0x40095034, the LCD FIFO (NOT 0x30, the PIO register)
 *   +0x08 source
 *   +0x0c length       low 18 bits; upper bits are configuration
 *   +0x20 channel config - written by the vendor's dma_open (FIRM 0xd7d970,
 *         "str r3, [r2, #32]"), reads 0x00080002 on BOTH channels in the live
 *         capture. Omitting it is why the first DMA attempt still timed out:
 *         the channel was addressed correctly but never configured.
 *
 * The control and length-register configuration bits are captured from stock
 * with channel 0 parked: control 0x001a0289 with both bit29 and bit30 clear,
 * length register 0x13000000 with the count zero. We never call the vendor's
 * dma_open, so we write those configuration words ourselves rather than
 * read-modify-writing what open() would have left.
 *
 * Transfer sequence is vendor 0x815aba:
 *   base[0x24] = cmd; base[0x28] = nbytes-1; wait !busy
 *   base[0x10] = (base[0x10] & ~0x30) | 8
 *   start the DMA
 *   CTRL = (CTRL & ~0x30c & ~2) | 1      (cmdlen 1 -> bits 8-9 clear)
 */
#define LCD_DMA_CH_BASE  0x40001000u
#define LCD_DMA_CTRL_CFG 0x001a0289u
#define LCD_DMA_LEN_CFG  0x13000000u
#define LCD_DMA_CH_CFG   0x00080002u
#define LCD_FIFO_ADDR    (LCDC_BASE + 0x34u)

/* PROBE, slots 136..141: is the DMA ENGINE actually moving bytes?
 *
 * Every "N transfers completed" this port has ever reported counts the LCDC's
 * DONE bit, which says the controller finished a transaction - not that the DMA
 * fed it anything. A full frame now completes with zero timeouts and the panel
 * does not change, so that distinction is the whole question.
 *
 * The vendor's own dma_get_remaining at ROM 0x814d1c is `ch[0x0c] & 0x3ffff`,
 * so the length register counts DOWN. After a completed 4096-byte transfer it
 * should read 0. If it still reads 4096, the channel never ran and the LCDC has
 * been completing empty transactions all along. Source is captured too: the
 * vendor's engine advances it. */
static unsigned lcd_dma_probed;

static void LCDICODE lcd_dma_chunk(unsigned cmd, const void *src, unsigned nbytes)
{
    volatile uint32_t *ch = (volatile uint32_t *)LCD_DMA_CH_BASE;

    LCDC_CMD = cmd;
    LCDC_DATALEN = nbytes - 1;
    lcdc_wait_idle();
    LCDC_CFG10 = (LCDC_CFG10 & ~0x30u) | 8u;

    ch[9] = ch[9];                               /* +0x24 W1C status - dma_open
                                                  * clears this at FIRM 0xd7d944
                                                  * ("ldr r3,[r2,#36]; str r3,
                                                  * [r2,#36]") and we never did.
                                                  * A latched status bit is one
                                                  * way an armed channel refuses
                                                  * to start. */
    ch[8] = LCD_DMA_CH_CFG;                      /* +0x20 channel config */
    ch[1] = LCD_FIFO_ADDR;                       /* +0x04 destination */
    ch[2] = (uint32_t)src;                       /* +0x08 source      */
    ch[3] = LCD_DMA_LEN_CFG | (nbytes & 0x3ffffu);
    ch[0] = (LCD_DMA_CTRL_CFG & ~0x20000000u) | 0x40000000u;   /* start */

    if (!lcd_dma_probed) {
        BC_SLOT(136) = ch[3];            /* length WHILE the DMA should be running */
        BC_SLOT(137) = LCDC_STATUS;      /* is the FIFO filling? */
    }

    lcdc_kick((LCDC_CTRL & ~0x30cu & ~0x2u) | 0x1u);

    if (!lcd_dma_probed) {
        lcd_dma_probed = 1;
        BC_SLOT(145) = lcd_last_spin;    /* spin for a 4096-byte DMA transfer */
        BC_SLOT(138) = ch[3];            /* remaining after - 0 means it drained */
        BC_SLOT(139) = ch[0];            /* control: did start/done bits move? */
        BC_SLOT(140) = ch[2];            /* source: did it advance past src? */
        BC_SLOT(141) = LCDC_STATUS;
    }
}

#if YP3_LCD_DMA_TEST
/* Fill the panel red by DMA, then hold.
 *
 * THE DMA SOURCE MUST LIVE IN SRAM.
 *
 * Round 1 filled from red[] in .bss and completed ten chunks with no timeouts.
 * Round 2 added COLMOD/CASET/RASET as DMA transfers, and the compiler put those
 * byte arrays in .rodata - which for an XIP build is SPI NOR flash at
 * 0xc0e000+. The dump from that run has slots 116..119 all zero while BC(8) and
 * panel stage 0xa7 are both set: it hung inside the FIRST lcd_dma_chunk, before
 * the store that follows the call. lcd_dma_chunk contains only bounded loops
 * and cannot hang on its own, so this was a bus stall - pointing the DMA engine
 * at the flash aperture the CPU is fetching instructions from wedges the flash
 * controller, and the next flash-resident instruction never retires. The port
 * has now been bitten by flash contention four times; slot 46 records the
 * source address so the next dump answers this by inspection.
 *
 * Parameters go back through the PIO path, which is what the vendor actually
 * does. Its window setter at 0x812cb0 sends CASET and RASET one byte at a time
 * through 0x815788 -> 0x815902 - the standalone data path wd8() copies
 * instruction for instruction - and only the pixel stream ever goes by DMA
 * (0x815aba). The vendor path shows the opposite of the earlier assumption:
 * parameters go through PIO; only the pixel stream uses DMA.
 */
static void LCDICODE lcd_dma_test(void)
{
    /* .bss, so SRAM. NEVER const and never a literal array: either puts it in
     * .rodata, i.e. flash, which is the hang described above. */
    static unsigned short red[2048];   /* 4096 bytes, the vendor's per-transfer cap */
    unsigned i;

    BC_SLOT(116) = 0xD1A00000u;
    for (i = 0; i < 2048; i++)
        red[i] = 0x00f8;               /* 0xf800 byte-swapped, as everywhere else */
    BC_SLOT(46) = (uint32_t)red;       /* 0x8xxxxx = SRAM, ok. 0xcxxxxx = flash = hang */

    /* Full screen, +2/+1 offsets, by PIO exactly as vendor 0x812cb0 does. It
     * ends with RAMWR, which the first chunk sends again - harmless, it just
     * re-homes the GRAM pointer at the window origin. */
    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
    BC_SLOT(116) = 0xD1A00001u;

    lcd_dma_chunk(0x2c, red, sizeof red);        /* RAMWR */
    BC_SLOT(116) = 0xD1A00002u;
    BC_SLOT(117) = lcd_cmds_done;
    BC_SLOT(118) = lcd_timeouts;
    BC_SLOT(47) = 1;

    for (i = 0; i < 9; i++) {                    /* 10 * 4096 = 40960 = one frame */
        lcd_dma_chunk(0x3c, red, sizeof red);    /* write-memory-continue */
        BC_SLOT(47) = i + 2;
    }

    BC_SLOT(116) = 0xD1A0000Du;
    BC_SLOT(119) = lcd_cmds_done;

    /* PHASE 2: the six-bit test.
     *
     * Commands land and parameters do not, and that has never been explained by
     * protocol, pins, registers, clocks or timing - all of which now match the
     * vendor. But look at WHICH bytes have ever worked. Every command proven to
     * reach the panel is <= 0x3f: DISPOFF 0x28, DISPON 0x29, INVON 0x21,
     * INVOFF 0x20, NORON 0x13, SLPOUT 0x11. Every parameter that silently did
     * nothing had a high bit set: MADCTL 0xd0, PTLAR row 159 = 0x9f, CASET
     * x1 = 129 = 0x81, and the panel unlock 0xfe/0xfe/0xef.
     *
     * If the top two data lines are not reaching the panel, all of that follows
     * at once. MADCTL 0xd0 arrives as 0x10 - identical to the 0x10 it was being
     * compared against, so the display could not flip. CASET x1 = 0x81 arrives
     * as 0x01, giving x0 = 2 > x1 = 1: an inverted window, into which a whole
     * frame writes invisibly. That is exactly what we see.
     *
     * So paint a block whose window bounds AND pixel bytes are ALL <= 0x3f.
     * Nothing in this phase needs bit 6 or bit 7 on the bus.
     *
     *   window  x 2..33, y 1..32      params 0x00,0x02,0x00,0x21 / 0x00,0x01,0x00,0x20
     *   colour  0x3f3f                both bytes <= 0x3f
     *
     * A 32x32 block of blue-green in the top-left corner means the bus is
     * dropping the top two bits, and every unexplained result
     * collapses into that one fault. Nothing at all means
     * the truncation theory is dead and the DMA readback in slots 136..141 is
     * the thread to pull instead. */
    lcd_write_cmd(0x2a);
    wd8(0x00); wd8(0x02); wd8(0x00); wd8(0x21);      /* x 2..33  */
    lcd_write_cmd(0x2b);
    wd8(0x00); wd8(0x01); wd8(0x00); wd8(0x20);      /* y 1..32  */
    lcd_write_cmd(0x2c);                             /* RAMWR    */
    BC_SLOT(144) = lcd_last_spin;        /* spin for a parameterless command */

    /* PIO, one pixel per transfer - NOT the DMA path.
     *
     * The first attempt at this test sent the block through lcd_dma_chunk,
     * which was a mistake: it gated a question about the BUS on the very path
     * that is itself in doubt. A dead DMA engine would have produced "no block"
     * regardless of how many data bits reach the panel, and the test would have
     * proved nothing.
     *
     * lcd_write_data_n(v, 2) is vendor 0x815902 with count 2, which is exactly
     * what the vendor's own fill at 0x812dc4 does once per pixel - it byte-swaps
     * (r4 = (c << 8) | (c >> 8)) and calls the count == 2 data path in a loop.
     * So this is a real vendor GRAM path, independent of the DMA engine.
     *
     * 1024 pixels; at 32 MHz flash that is nothing. */
    for (i = 0; i < 1024; i++)
        lcd_write_data_n(0x3f3f, 2);     /* 32 x 32, both bytes <= 0x3f */

    /* PROBE, slots 151..159: what does the wire actually carry?
     *
     * The LCDC diff came back matching the stock working capture everywhere
     * except +0x34, the data FIFO, which stock leaves at 0 and ours holds at
     * 0x3c3c3c3c - one byte replicated across all four lanes. The last byte we
     * had put on the wire was 0x3f, from lcd_write_data_n(0x3f3f, 2). And 0x3f
     * with bits 0 and 1 cleared is 0x3c.
     *
     * If the data path really is dropping the low two bits, everything follows:
     * CASET x1 = 129 = 0x81 arrives as 0x80, RASET y1 = 160 = 0xa0 survives,
     * MADCTL 0xd0 survives but 0x10 does too - and, more to the point, a pixel
     * stream would still be written, just wrong. So this alone does not explain
     * a blank panel, which is exactly why it has to be MEASURED rather than
     * believed.
     *
     * Send eight data bytes chosen to exercise every bit position and read the
     * FIFO straight back after each. The pattern in the readback settles what
     * 0x34 reflects at the same time: a replicated byte means it is the
     * transmit shift register, anything else means it is something we have been
     * misreading. Slot 159 does the same for a COMMAND byte, 0x13 (NORON, safe
     * and with bits 0 and 1 set), because commands demonstrably reach the panel
     * and data does not - so if the two readbacks differ in the same way the
     * asymmetry is finally located in hardware rather than inferred.
     *
     * Values: ff 55 aa 0f f0 03 3f 01. */
    {
        static const unsigned char probe[8] =
            { 0xff, 0x55, 0xaa, 0x0f, 0xf0, 0x03, 0x3f, 0x01 };
        unsigned k;
        for (k = 0; k < 8; k++) {
            lcd_write_data_n(probe[k], 1);
            BC_SLOT(151 + k) = LCDC_DATA_RD;
        }
        lcd_write_cmd(0x13);                 /* NORON - bits 0 and 1 set */
        BC_SLOT(159) = LCDC_DATA_RD;
    }

    BC_SLOT(116) = 0xD1A0006Bu;          /* six-bit block sent, by PIO */
    /* Our LCDC state, for a register-by-register diff against the stock capture
     * made while stock was displaying. This build does not reach lcd_update_rect,
     * so the comparison has not been checked against THIS configuration. */
    lcd_snapshot_regs();
    BC_SLOT(142) = lcd_cmds_done;
    BC_SLOT(143) = lcd_timeouts;
    BC_SLOT(146) = lcd_last_spin;        /* spin for a 2-byte PIO pixel write */

    while (1) ;                                  /* hold the result on screen */
}
#endif

#if YP3_LCD_FIFO_TEST
static void LCDICODE lcd_fifo_chunk(unsigned cmd, const unsigned *words,
                                    unsigned nwords)
{
    unsigned i, nbytes = nwords * 4;

    LCDC_CMD = cmd;
    LCDC_DATALEN = nbytes - 1;
    lcdc_wait_idle();
    LCDC_CFG10 = (LCDC_CFG10 & ~0x30u) | 8u;
    for (i = 0; i < nwords; i++)
        LCDC_DATA_RD = words[i];
    lcdc_kick((LCDC_CTRL & ~0x30cu & ~0x2u) | 0x1u);
}

static void lcd_fifo_test(void)
{
    static unsigned buf[8];
    unsigned i, chunk;

    /* 0xf800 red, byte-swapped like every other pixel we send */
    for (i = 0; i < 8; i++)
        buf[i] = 0x00f800f8u;

    BC_SLOT(116) = 0xF1F00001u;
    /* First chunk carries RAMWR; the rest continue via 0x3c so the GRAM address
     * keeps advancing instead of resetting to the window origin. */
    lcd_fifo_chunk(0x2c, buf, 8);
    BC_SLOT(116) = 0xF1F00002u;
    BC_SLOT(117) = lcd_cmds_done;
    BC_SLOT(118) = lcd_timeouts;

    for (chunk = 0; chunk < 1280; chunk++)   /* 1280 * 16 px = a full frame */
        lcd_fifo_chunk(0x3c, buf, 8);

    BC_SLOT(116) = 0xF1F0000Du;
    BC_SLOT(119) = lcd_cmds_done;   /* 120..124 are the clock probe - do not reuse */
    while (1) ;                              /* hold the result on screen */
}
#endif



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
