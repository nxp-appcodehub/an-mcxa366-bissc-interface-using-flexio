/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app.h"
#include "board.h"
#include "clock_config.h"
#include "flexio_bissc.h"
#include "fsl_debug_console.h"
#include "mcuxsdk_version.h"
#include "pin_mux.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

static FLEXIO_BISSC_Type g_bisscHandle = {
    .flexio           = DEMO_BISSC_FLEXIO_BASE,
    .slPinIdx         = DEMO_BISSC_SL_PIN,
    .maPinIdx         = DEMO_BISSC_MA_PIN,
    .gatePinIdx       = DEMO_BISSC_GATE_PIN,
    .gatePinOutput    = (DEMO_BISSC_GATE_PIN_OUTPUT != 0U),
    .shifterStartIdx  = DEMO_BISSC_SHIFTER_START,
    .timerStartIdx    = DEMO_BISSC_TIMER_START,
    .hwTrigger        = (DEMO_BISSC_USE_HW_TRIGGER != 0U),
    .baudRateBps      = DEMO_ENCODER_BIT_RATE,
    .mtLen            = DEMO_ENCODER_MT_LEN,
    .stLen            = DEMO_ENCODER_ST_LEN,
    .rxTrailingBitLen = DEMO_BISSC_RX_TRAILING_BITS,
};

/* CDS accumulator. Eight valid frames form one byte, MSB first. */
static uint8_t s_cdsByte;
static uint8_t s_cdsBitCount;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static bool    DEMO_InitBissc(FLEXIO_BISSC_Type *base);
static bool    DEMO_WaitBisscFrame(FLEXIO_BISSC_Type *base);
static int64_t DEMO_CombinedPosition(const FLEXIO_BISSC_Type *base);
static void    DEMO_PrintPosition(int64_t position);
static void    DEMO_PrintHeader(FLEXIO_BISSC_Type *base);
static void    DEMO_PrintFrame(FLEXIO_BISSC_Type *base);
static void    DEMO_PrintTimeoutDiag(FLEXIO_BISSC_Type *base);
static void    DEMO_MaybePrintStats(FLEXIO_BISSC_Type *base, uint32_t attempts);
static void    DEMO_CollectCds(FLEXIO_BISSC_Type *base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static bool DEMO_InitBissc(FLEXIO_BISSC_Type *base)
{
    bool status;

    /* FLEXIO_BISSC_Init resets and reconfigures the whole FlexIO instance, so the
     * interrupt must stay masked until the new configuration is in place. */
#if (DEMO_BISSC_USE_INTERRUPT != 0U)
    (void)DisableIRQ(DEMO_BISSC_FLEXIO_IRQ);
#endif

    status = (FLEXIO_BISSC_Init(base, DEMO_BISSC_CLOCK_FREQUENCY) == 0);

#if (DEMO_BISSC_USE_INTERRUPT != 0U)
    if (status)
    {
        FLEXIO_BISSC_EnableRxInterrupt(base, true);
        NVIC_ClearPendingIRQ(DEMO_BISSC_FLEXIO_IRQ);
        (void)EnableIRQ(DEMO_BISSC_FLEXIO_IRQ);
    }
#endif

    return status;
}

static bool DEMO_WaitBisscFrame(FLEXIO_BISSC_Type *base)
{
#if (DEMO_BISSC_USE_INTERRUPT != 0U)
    uint32_t stepsLeft = (DEMO_BISSC_WAIT_TIMEOUT_US + FLEXIO_BISSC_POLL_STEP_US - 1U) / FLEXIO_BISSC_POLL_STEP_US;

    while (!base->rxdBufferReady)
    {
        if (stepsLeft == 0U)
        {
            base->timeoutCount++;
            return false;
        }
        SDK_DelayAtLeastUs(FLEXIO_BISSC_POLL_STEP_US, SystemCoreClock);
        stepsLeft--;
    }

    /* Pair the barrier in the ISR: it publishes rxdBuffer before the flag, this
     * one keeps the compiler from loading rxdBuffer before the flag was seen set.
     *
     * The flag is deliberately left set. It doubles as the ownership marker for
     * rxdBuffer, so holding it across the parse makes the ISR drop a new
     * window instead of overwriting the one being parsed. It is released at the
     * top of the next loop iteration. */
    __DMB();

    return true;
#else
    return FLEXIO_BISSC_ReadBlocking(base, DEMO_BISSC_WAIT_TIMEOUT_US);
#endif
}

static int64_t DEMO_CombinedPosition(const FLEXIO_BISSC_Type *base)
{
    /* MT is a signed multi-turn counter, so sign extend it before combining.
     * position = MT * 2^stLen + ST is the check that proves the MT/ST
     * boundary: CRC6 covers the payload as a whole, so any regrouping of the same
     * bits passes it, while the combined position has to stay monotonic across an
     * ST rollover in both rotation directions. */
    int64_t mtSigned = (int64_t)base->mt;

    if ((base->mtLen != 0U) && (base->mtLen < 32U) && (((base->mt >> (base->mtLen - 1U)) & 1U) != 0U))
    {
        mtSigned -= (int64_t)1 << base->mtLen;
    }

    return (mtSigned * ((int64_t)1 << base->stLen)) + (int64_t)base->st;
}

static void DEMO_PrintPosition(int64_t position)
{
    /* The lite debug console is built with PRINTF_ADVANCED_ENABLE = 0 and has no
     * %lld, so format the combined position by hand. Truncating it to 32 bits is
     * not an option: for a 16-bit MT and a 17-bit ST the value needs 33 bits, and
     * it would wrap exactly at the rollover this print exists to inspect. */
    char     text[24];
    uint32_t idx       = (uint32_t)sizeof(text) - 1U;
    bool     negative  = (position < 0);
    uint64_t magnitude = negative ? (0ULL - (uint64_t)position) : (uint64_t)position;

    text[idx] = '\0';

    do
    {
        idx--;
        text[idx] = (char)('0' + (uint32_t)(magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U);

    if (negative)
    {
        idx--;
        text[idx] = '-';
    }

    PRINTF("%s", &text[idx]);
}

static void DEMO_PrintHeader(FLEXIO_BISSC_Type *base)
{
    PRINTF("MCUX SDK version: %s\r\n", MCUXSDK_VERSION_FULL_STR);
    PRINTF("\r\nFlexIO %s %s demo on MCXA366\r\n", DEMO_ENCODER_PROTOCOL, DEMO_ENCODER_NAME);
    PRINTF("FlexIO clock: %u Hz\r\n", DEMO_BISSC_CLOCK_FREQUENCY);
    PRINTF("Bit rate: %u bps, MT: %u bits, ST: %u bits\r\n", base->baudRateBps, base->mtLen, base->stLen);
    PRINTF("MA: FLEXIO0_D%u, SL: FLEXIO0_D%u\r\n", base->maPinIdx, base->slPinIdx);
    if (base->gatePinOutput)
    {
        PRINTF("MA gate debug output: FLEXIO0_D%u (high for the whole MA burst)\r\n", base->gatePinIdx);
    }
    PRINTF("MA gating: continuous until STR, then stopped by the RX window; SL pull: off\r\n");
    PRINTF("RX start: STR rising edge, RX edge: rising, RX window: %u bits (%u trailing), %u shifters\r\n",
           base->rxWindowBitLen, base->rxTrailingBitLen, base->rxShifterNum);
    PRINTF("Frame wait timeout: %u us, frame interval: %u us\r\n", DEMO_BISSC_WAIT_TIMEOUT_US,
           DEMO_BISSC_FRAME_INTERVAL_US);
#if (DEMO_BISSC_SOAK_MODE != 0U)
    PRINTF("Soak mode: per-frame print off, statistics every %u attempts\r\n\r\n", DEMO_BISSC_STATS_PERIOD);
#else
    PRINTF("Statistics every %u attempts\r\n\r\n", DEMO_BISSC_STATS_PERIOD);
#endif
}

static void DEMO_PrintFrame(FLEXIO_BISSC_Type *base)
{
    PRINTF("MT=%u, ST=%u, POS=", base->mt, base->st);
    DEMO_PrintPosition(DEMO_CombinedPosition(base));
    PRINTF(", CDS=%u, START=%u, ERR=%u, WARN=%u, CRC=%s\r\n", base->cdsBit, base->startBit, base->errorBit,
           base->warnBit, base->crcMatch ? "OK" : "FAIL");

    if (base->startBit == 0U)
    {
        PRINTF("START missing: receive window is not aligned to STR\r\n");
    }
}

static void DEMO_PrintTimeoutDiag(FLEXIO_BISSC_Type *base)
{
    /* A bare "timeout" line says nothing, and the three status registers say
     * almost everything: FLEXIO_BISSC_SwTrigger clears them before every frame,
     * so what is left belongs to the frame that just failed. */
    flexio_bissc_diag_t diag;

    FLEXIO_BISSC_GetDiag(base, &diag);

    PRINTF("BiSS-C frame timeout: SHIFTSTAT=0x%x, SHIFTERR=0x%x, TIMSTAT=0x%x\r\n", diag.shiftStat, diag.shiftErr,
           diag.timStat);

    if (!diag.trgUnderrun && !diag.maRanLong)
    {
        /* MA never clocked: the trigger shifter cannot underrun and the MA timer
         * cannot reach its compare unless MA actually ran. */
        PRINTF("  MA never clocked: check the FlexIO clock, the P2_10 pinmux and the trigger shifter write\r\n");
    }
    else if (!diag.rxWindowDone)
    {
        PRINTF("  MA clocked but STR never arrived: check the SL wiring, SL pin index %u and the encoder supply\r\n",
               base->slPinIdx);
    }
    else
    {
        PRINTF("  RX window completed but was not reported: check SHIFTSIEN and the FLEXIO_IRQHandler wrapper\r\n");
    }

    if (diag.gateWrapped)
    {
        PRINTF("  MA gate wrapped: MA ran a whole gate period, so a later frame can be misaligned\r\n");
    }
}

static void DEMO_MaybePrintStats(FLEXIO_BISSC_Type *base, uint32_t attempts)
{
#if (DEMO_BISSC_STATS_PERIOD != 0U)
    if ((attempts % DEMO_BISSC_STATS_PERIOD) == 0U)
    {
        PRINTF("STATS: attempts=%u, frames=%u, crcErr=%u, startErr=%u, timeout=%u, overrun=%u\r\n", attempts,
               base->frameCount, base->crcErrorCount, base->startErrorCount, base->timeoutCount, base->overrunCount);
    }
#else
    (void)base;
    (void)attempts;
#endif
}

static void DEMO_CollectCds(FLEXIO_BISSC_Type *base)
{
    /* Only valid frames advance the accumulator, otherwise a rejected frame would
     * silently shift the byte boundary. A lost frame still breaks the grouping,
     * which is why the timeout path restarts the accumulator instead of letting
     * the next valid bit stand in for the missing one. */
    if (!base->frameValid)
    {
        return;
    }

    s_cdsByte = (uint8_t)((s_cdsByte << 1U) | base->cdsBit);
    s_cdsBitCount++;

    if (s_cdsBitCount >= 8U)
    {
        PRINTF("CDS byte=0x%x\r\n", s_cdsByte);
        s_cdsByte     = 0U;
        s_cdsBitCount = 0U;
    }
}

int main(void)
{
    FLEXIO_BISSC_Type *base     = &g_bisscHandle;
    uint32_t           attempts = 0U;

    BOARD_InitHardware();

    /* Init derives the receive window and the shifter count, so it has to run
     * before the header can report them. */
    if (!DEMO_InitBissc(base))
    {
        PRINTF("FlexIO BiSS-C init failed. FlexIO clock %u Hz, bit rate %u bps, MT %u bits, ST %u bits.\r\n",
               DEMO_BISSC_CLOCK_FREQUENCY, base->baudRateBps, base->mtLen, base->stLen);
        while (1)
        {
        }
    }

    DEMO_PrintHeader(base);

#if (DEMO_BISSC_RUN_PARSER_SELF_TEST != 0U)
    bool parserSelfTestPass = FLEXIO_BISSC_RunParserSelfTest(base, DEMO_BISSC_SELF_TEST_MT, DEMO_BISSC_SELF_TEST_ST,
                                                             DEMO_BISSC_SELF_TEST_ERR, DEMO_BISSC_SELF_TEST_WARN,
                                                             DEMO_BISSC_SELF_TEST_CDS);
    PRINTF("Parser self-test: MT=%u, ST=%u, POS=", base->mt, base->st);
    DEMO_PrintPosition(DEMO_CombinedPosition(base));
    PRINTF(", CDS=%u, START=%u, ERR=%u, WARN=%u, CRC=%s, RESULT=%s\r\n\r\n", base->cdsBit, base->startBit,
           base->errorBit, base->warnBit, base->crcMatch ? "OK" : "FAIL", parserSelfTestPass ? "PASS" : "FAIL");
#endif

    /* After the self-test, so the synthetic frame it parsed is not counted as a
     * live one. Init deliberately leaves the counters alone, so the timeout
     * recovery path can run without erasing the run history. */
    FLEXIO_BISSC_ResetStats(base);

    while (1)
    {
        /* Release rxdBuffer back to the ISR. This is the only place it is cleared:
         * the flag is held across the parse and the print so that a window
         * arriving in between is dropped and counted rather than overwriting the
         * frame being read. */
        base->rxdBufferReady = false;
        FLEXIO_BISSC_SwTrigger(base);
        attempts++;

        if (!DEMO_WaitBisscFrame(base))
        {
            /* MA keeps clocking while it waits for STR, so reinitialize FlexIO to
             * stop it before retrying. */
            DEMO_PrintTimeoutDiag(base);

            /* A dropped frame also drops a CDS bit, and valid-only collection
             * cannot tell which one, so restart the byte grouping. */
            s_cdsByte     = 0U;
            s_cdsBitCount = 0U;

            if (!DEMO_InitBissc(base))
            {
                PRINTF("FlexIO BiSS-C recovery failed.\r\n");
                while (1)
                {
                }
            }

            DEMO_MaybePrintStats(base, attempts);
            SDK_DelayAtLeastUs(DEMO_BISSC_FRAME_INTERVAL_US, SystemCoreClock);
            continue;
        }

        FLEXIO_BISSC_DataParser(base);
#if (DEMO_BISSC_PRINT_EVERY_FRAME != 0U)
        DEMO_PrintFrame(base);
#else
        /* Soak mode. The statistics carry the counts, but a count does not say
         * what a bad frame looked like, so the failures are still printed. */
        if (!base->frameValid)
        {
            DEMO_PrintFrame(base);
        }
#endif
        DEMO_CollectCds(base);

        DEMO_MaybePrintStats(base, attempts);
        SDK_DelayAtLeastUs(DEMO_BISSC_FRAME_INTERVAL_US, SystemCoreClock);
    }
}

void FLEXIO_IRQHandler(void)
{
    FLEXIO_BISSC_IRQHandler(&g_bisscHandle);
}
