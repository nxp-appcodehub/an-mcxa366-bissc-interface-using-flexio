/*
 * Copyright 2025-2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define DEMO_ENCODER_PROTOCOL "BiSS-C"

/* ADK-4088-MA16/17B1BLP-BH40 (IDEncoder, https://www.idencoder.cn):
 * 16-bit multi-turn and 17-bit single-turn position. */
#define DEMO_ENCODER_NAME     "ADK-4088-MA16/17B1BLP-BH40"
#define DEMO_ENCODER_BIT_RATE 10000000U
#define DEMO_ENCODER_MT_LEN   16U
#define DEMO_ENCODER_ST_LEN   17U

/* STR starts the fixed receive tail; one trailing sample is removed before parsing.
 * FLEXIO_BISSC_Init derives the window itself from MT/ST and this value, so the
 * frame length is stated in exactly one place. */
#define DEMO_BISSC_RX_TRAILING_BITS 1U

#define DEMO_BISSC_FLEXIO_BASE      (FLEXIO0)
#define DEMO_BISSC_CLOCK_FREQUENCY  CLOCK_GetFlexioClkFreq()
#define DEMO_BISSC_FLEXIO_IRQ       FLEXIO_IRQn
#define DEMO_BISSC_SL_PIN           17U
#define DEMO_BISSC_MA_PIN           18U

/* Debug output of the MA gate timer (TIMER2), for probing the frame window.
 * FLEXIO0_D30 = P2_22 = package pin 57. Off by default; set to 1U to drive the
 * pin. The pinmux for P2_22 is configured either way, so only this macro needs
 * to change to get the probe back. */
#define DEMO_BISSC_GATE_PIN_OUTPUT  0U
#define DEMO_BISSC_GATE_PIN         30U
#define DEMO_BISSC_SHIFTER_START    0U
#define DEMO_BISSC_TIMER_START      0U
#define DEMO_BISSC_USE_HW_TRIGGER   0U
#define DEMO_BISSC_USE_INTERRUPT    1U

/* Frame wait budget in real microseconds, not loop iterations.
 *
 * One 44-bit window at 10 MHz takes 4.4 us, plus about 4 MA clocks of LAT and
 * ACK ahead of STR, so a healthy frame completes in well under 10 us. The upper
 * bound matters more than the lower one: the MA gate timer runs single 16-bit
 * with TIMCMP = 0xFFFF, which wraps after 65536 FlexIO clocks, about 364 us at
 * 180 MHz. While MA is left running by a missing STR, a gate wrap drops the RX
 * timer's trigger-high condition and then raises it again, so an unrelated SL
 * edge can open a receive window and produce a completed-but-garbage frame. A
 * budget comfortably below one gate period keeps a missing STR reported as a
 * timeout, as expected by the re-init recovery path. */
#define DEMO_BISSC_WAIT_TIMEOUT_US   100U

/* Soak mode for long-run validation.
 *
 * The default 5 Hz frame rate keeps the printed values readable by eye, but it
 * limits the number of frames a long run can cover, because the console rather
 * than the encoder sets the pace. Soak mode runs frames back to back and reports
 * statistics instead of every frame, so a large frame count can be validated.
 * Failed frames are still printed in full, because the counters do not record
 * the field values of a bad frame. Set to 1U for a long run. */
#define DEMO_BISSC_SOAK_MODE 0U

#if (DEMO_BISSC_SOAK_MODE != 0U)
#define DEMO_BISSC_FRAME_INTERVAL_US 200U
#define DEMO_BISSC_PRINT_EVERY_FRAME 0U
#define DEMO_BISSC_STATS_PERIOD      2000U
#else
#define DEMO_BISSC_FRAME_INTERVAL_US 200000U
#define DEMO_BISSC_PRINT_EVERY_FRAME 1U
#define DEMO_BISSC_STATS_PERIOD      20U
#endif

/* Offline check of the CRC6 parameters and the field layout. Enabled by default:
 * it costs one startup line and nothing at run time, and it catches a broken
 * CRC6 or a shifted field boundary before the encoder gets blamed for it. */
#define DEMO_BISSC_RUN_PARSER_SELF_TEST 1U
#define DEMO_BISSC_SELF_TEST_MT     0x1234U
#define DEMO_BISSC_SELF_TEST_ST     0x1AAAAU
#define DEMO_BISSC_SELF_TEST_ERR    1U
#define DEMO_BISSC_SELF_TEST_WARN   0U
#define DEMO_BISSC_SELF_TEST_CDS    1U
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
