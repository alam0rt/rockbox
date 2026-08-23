/* Boot progress markers, in a reserved NOLOAD region (see app.lds).
 *
 * SINGLE SLOT MAP - 128 slots exist (512 bytes). Overlapping assignments
 * silently corrupted results once already, so keep this table authoritative:
 *
 *   0       most recent stage marker
 *   1..10   boot stages (BC(n))
 *   11,12   LCDC status/ctrl before init
 *   13,14   LCDC status/ctrl after init
 *   15      LCD panel sequence stage (0xA0..0xA7)
 *   16,17   LCD transfers completed / timed out
 *   18      clock-gate checkpoint
 *   19      LCDC register checkpoint
 *   20      LCDC writable bisect
 *   21,22   spurious IRQ number / count
 *   23,24,25 hard fault flag / CFSR / stacked PC
 *   26      backlight init entered
 *   27      backlight result (module id or status)
 *   28      backlight scan progress
 *   29,30   PWM3 +0x00 / +0x04
 *   31      pixel clock
 *   32..40  SD: stage, bring-up, STA, CLKCR, card version, OCR/HC, RCA,
 *           block count, success
 *   41      LCD pixel clock measured in lcdc_hw_init
 *   42      source 8 rate measured in lcdc_hw_init
 *   43      ROM timer delta   44,45  watchdog CTRL / LOAD as the ROM left it
 *   46      PROBE DMA test: source address - MUST be 0x8xxxxx (SRAM). A
 *           0xcxxxxx source is the XIP flash aperture and hangs the bus.
 *   47      PROBE DMA test: chunks completed
 *   50..52  storage_init / disk_mount_all / init() complete
 *   53..55  main() entered / init() entered / system_init returned
 *   56      post-mount init phase
 *   57..61  SD capacity and CSD readback
 *   62      audio_init step
 *   63      playback_init step
 *   64      audio thread: running / pcm_postinit returned
 *   65      codec thread: running
 *   66      buffering_init: thread created / returned
 *   67      settings_apply_skins step
 *   68      skin_get_gwps (skin<<8 | screen), |0xff once returned
 *   69      skin_load step
 *   70      skin_data_load step
 *   71,72   plugin_get_buffer result: address / size
 *   73      PROBE lcd_update_rect call count
 *   74      PROBE transfers after the last update
 *   75      PROBE FBADDR(0,0) at the first update
 *   76      main(): 1 = first screen update done, 2 = list/tree_init done
 *   77      PROBE pattern test: started / finished
 *   78      PROBE pattern test: rows completed
 *   79,80   PROBE pattern test: timeouts / transfers
 *   81..104 PROBE LCDC 0x00..0x5c read back after the first frame
 *   105..115 PROBE GPIO state: port1 mode 0-3, port3 mode 0-3,
 *            pad regs 0x40081400/404/408
 *   116..119 PROBE display test: state / progress / timeouts / transfers.
 *            Shared by the DMA, FIFO and blink tests - exactly one of them is
 *            ever compiled in, so they cannot collide.
 *   120..124 PROBE clock rates: cpu pll / core pll / ahb / norf / HOSC
 *   125      build id - compare against the ELF; a mismatch means the dump
 *            came from a stale image
 *   126      clock init progress (vendor bootloader 0x8206e4)
 *   127      LCD pixel clock after bring-up
 *   128..132 clock census after yp3_clock_init: ids 3, 4, 5, 0x29, 0x2a
 *   151      source 8 rate after clk_apply(8), before it is selected as the
 *            LCD pixel clock source
 *   147..149 PROBE RESX level read back at each phase of the reset pulse
 *            (0xE50000xx); 150 = port-1 mode word for pins 8-15
 *   144..146 PROBE lcdc_kick spin count: parameterless command / 4096-byte
 *            DMA transfer / 2-byte PIO pixel write. A DMA transfer that
 *            completes in a command's worth of iterations shifted nothing.
 *   142,143  PROBE six-bit block: transfers / timeouts after the PIO fill
 *   136,137  PROBE DMA: length register and LCDC STATUS with the first
 *            transfer running
 *   138..141 PROBE DMA after the first transfer: length remaining (0 = it
 *            drained), channel control, source, LCDC STATUS
 *   133..135 flash clock: norf before / a word read back from 0xc0e000 at the
 *            new rate / norf after
 *
 *   166      SysTick tick counter, incremented by systick_handler itself
 *   171,172  PROBE main-thread stack: bytes ever used / total size,
 *            sampled from systick_handler every 256 ticks by scanning
 *            for the DEADBEEF fill. Delete once the size is settled.
 *
 *   200..252 PANIC record, written by panic_log_target/panic_log_frame in
 *            system-yp3box.c before panicf touches the LCD:
 *            200 magic 'PAN1', 201 pc, 202 sp, 203 frame count,
 *            204..219 the first 16 unwound frames, 220..251 the message,
 *            252 panic count (>1 = a panic panicked; the record is the last)
 *
 *
 * Slots 0..63 are ALL claimed. The region is 320 slots (1280 bytes) precisely
 * because reusing one costs a whole hardware cycle to a false reading - it has
 * happened three times. Take from the free range and extend this table.
 */
#ifndef __BREADCRUMB_H__
#define __BREADCRUMB_H__
#include <stdint.h>
extern uint32_t __bc_start[];
/* volatile: these stores are never read back by the program - the host reads the
 * region over USB after a reset - so the compiler is free to sink, reorder or
 * delete them. It did: the pattern test's final marker was eliminated because an
 * infinite loop followed it, and the dump reported "DID NOT FINISH" for a loop
 * that had completed all 160 rows. */
#define BC_SLOT(i) (((volatile uint32_t *)__bc_start)[(i)])
#define BC(n) do { \
        BC_SLOT(0) = 0xBCBCBC00u | (n); \
        BC_SLOT(n) = 0xBCBCBC00u | (n); \
    } while (0)
#endif
