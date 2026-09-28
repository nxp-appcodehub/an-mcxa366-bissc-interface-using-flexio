/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "flexio_bissc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TRG_SHIFTER 0U
#define RXD_SHIFTER 1U

#define RX_TIMER      0U
#define MA_TIMER      1U
#define MA_GATE_TIMER 2U

/* BiSS-C CRC6 parameters. Standard BiSS-C: poly 0x43, MSB-first, final XOR 0x3F. */
#define CRC6_POLY_BISSC 0x43U /*!< x^6 + x + 1, MSB-first feed. */
#define CRC6_MASK       0x3FU /*!< 6-bit mask. */
#define CRC6_MSB        0x20U /*!< Most significant bit of 6-bit CRC. */
#define CRC6_FINAL_XOR  0x3FU /*!< Standard BiSS-C inverted output. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint8_t FLEXIO_BISSC_CRC6(uint32_t *data, uint32_t len);
static void    FLEXIO_BISSC_ArrayRightShift(uint32_t *data, uint32_t wordLen, uint32_t shift);
static void    FLEXIO_BISSC_ArraySetBits(uint32_t *data, uint32_t wordLen, uint32_t bitOffset, uint8_t width,
                                          uint32_t value);
static uint32_t FLEXIO_BISSC_BitMask(uint8_t width);
static void     FLEXIO_BISSC_ReadRxShifters(FLEXIO_BISSC_Type *base, uint32_t *dest);

/*******************************************************************************
 * Codes
 ******************************************************************************/

int FLEXIO_BISSC_Init(FLEXIO_BISSC_Type *base, uint32_t srcClock_Hz)
{
    assert(base != NULL);
    assert(base->flexio != NULL);

    flexio_config_t         flexioConfig;
    flexio_shifter_config_t shifterConfig = {0U};
    flexio_timer_config_t   timerConfig   = {0U};
    uint32_t                payloadBitLen;
    uint32_t                windowBitLen;
    uint32_t                baudDivCalc;
    uint16_t                baudDiv;

    /* CRC covers MT + ST + ERROR + WARN. The frame from STR adds START, CDS and
     * the 6 CRC bits on top of that, and the capture window adds the trailing
     * samples on top of the frame.
     *
     * The window is derived here instead of being configured by the caller: the
     * parser aligns the frame by removing exactly rxTrailingBitLen bits from the
     * end of the window, so a window that is one bit too long shifts every field
     * and the only symptom is a CRC failure.
     *
     * payloadBitLen is capped at 64 because FLEXIO_BISSC_CRC6 folds the payload
     * into a single 64-bit accumulator. */
    payloadBitLen = FLEXIO_BISSC_PAYLOAD_BITS(base->mtLen, base->stLen);
    windowBitLen  = FLEXIO_BISSC_FRAME_BITS(base->mtLen, base->stLen) + (uint32_t)base->rxTrailingBitLen;

    if ((srcClock_Hz == 0U) || (base->baudRateBps == 0U) || (payloadBitLen > 64U) ||
        (windowBitLen > FLEXIO_BISSC_MAX_RX_WINDOW_BITS))
    {
        return -1;
    }

    base->rxWindowBitLen = (uint8_t)windowBitLen;

    /* Keep the divider in 32-bit arithmetic until it is range checked, so a large
     * clock-to-baud ratio cannot wrap into a value that looks legal. */
    baudDivCalc = (srcClock_Hz / base->baudRateBps) >> 1U;
    if ((baudDivCalc == 0U) || ((baudDivCalc - 1U) > 0xFFU))
    {
        return -1;
    }
    baudDiv = (uint16_t)(baudDivCalc - 1U);

    /* Size the shifter chain from the capture window, which is the number of bits
     * shifted in, not from the parsed frame length. */
    base->rxShifterNum = (uint8_t)((windowBitLen + 31U) >> 5U);
    if (((uint32_t)base->shifterStartIdx + RXD_SHIFTER + base->rxShifterNum > FLEXIO_SHIFTBUF_COUNT) ||
        ((uint32_t)base->timerStartIdx + MA_GATE_TIMER >= FLEXIO_TIMCTL_COUNT))
    {
        return -1;
    }

    FLEXIO_GetDefaultConfig(&flexioConfig);
    flexioConfig.enableInDebug = true;
    FLEXIO_Init(base->flexio, &flexioConfig);

    /* 1. Configure the shifter used only to generate the software trigger. */
    shifterConfig.timerSelect = base->timerStartIdx + MA_TIMER;
    shifterConfig.pinConfig   = kFLEXIO_PinConfigOutputDisabled;
    shifterConfig.shifterMode = kFLEXIO_ShifterModeTransmit;
    FLEXIO_SetShifterConfig(base->flexio, base->shifterStartIdx + TRG_SHIFTER, &shifterConfig);

    /* 2. Configure the receive shifter chain. The last shifter samples the SL pin. */
    shifterConfig.timerSelect   = base->timerStartIdx + RX_TIMER;
    shifterConfig.timerPolarity = kFLEXIO_ShifterTimerPolarityOnPositive;
    shifterConfig.pinConfig     = kFLEXIO_PinConfigOutputDisabled;
    shifterConfig.pinSelect     = base->slPinIdx;
    shifterConfig.pinPolarity   = kFLEXIO_PinActiveHigh;
    shifterConfig.shifterMode   = kFLEXIO_ShifterModeReceive;
    shifterConfig.inputSource   = kFLEXIO_ShifterInputFromNextShifterOutput;
    shifterConfig.shifterStop   = kFLEXIO_ShifterStopBitDisable;
    shifterConfig.shifterStart  = kFLEXIO_ShifterStartBitDisabledLoadDataOnEnable;
    for (uint32_t i = 0U; i < (uint32_t)base->rxShifterNum - 1U; i++)
    {
        FLEXIO_SetShifterConfig(base->flexio, base->shifterStartIdx + RXD_SHIFTER + i, &shifterConfig);
    }
    shifterConfig.inputSource = kFLEXIO_ShifterInputFromPin;
    FLEXIO_SetShifterConfig(base->flexio, base->shifterStartIdx + RXD_SHIFTER + base->rxShifterNum - 1U,
                            &shifterConfig);

    /* 3. Configure RX/tail timer. STR starts this fixed window. Its disable
     * event also stops the next timer, so MA length adapts to the STR delay.
     * The window must start on the STR rising edge: the ACK phase that precedes
     * STR has no fixed length, so any trigger-based start would sample it. */
    timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_TIMn(base->timerStartIdx + MA_GATE_TIMER);
    timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
    timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    timerConfig.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
    timerConfig.pinSelect       = base->slPinIdx;
    timerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
    timerConfig.timerMode       = kFLEXIO_TimerModeDual8BitBaudBit;
    timerConfig.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
    timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    timerConfig.timerReset      = kFLEXIO_TimerResetNever;
    timerConfig.timerDisable    = kFLEXIO_TimerDisableOnTimerCompare;
    timerConfig.timerEnable     = kFLEXIO_TimerEnableOnPinRisingEdgeTriggerHigh;
    timerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;
    timerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    timerConfig.timerCompare    = (((((uint32_t)base->rxWindowBitLen << 1U) - 1U) << 8U) | baudDiv);
    FLEXIO_SetTimerConfig(base->flexio, base->timerStartIdx + RX_TIMER, &timerConfig);

    /* 4. Configure MA clock timer. It starts from the software trigger and
     * runs until the preceding RX/tail timer is disabled after its compare. */
    if (base->hwTrigger)
    {
        timerConfig.triggerSelect   = 0U;
        timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
        timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceExternal;
    }
    else
    {
        timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(base->shifterStartIdx + TRG_SHIFTER);
        timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
        timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    }
    timerConfig.pinConfig      = kFLEXIO_PinConfigOutput;
    timerConfig.pinSelect      = base->maPinIdx;
    timerConfig.pinPolarity    = kFLEXIO_PinActiveLow;
    timerConfig.timerMode      = kFLEXIO_TimerModeDual8BitBaudBit;
    timerConfig.timerOutput    = kFLEXIO_TimerOutputZeroNotAffectedByReset;
    timerConfig.timerDecrement = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    timerConfig.timerReset     = kFLEXIO_TimerResetNever;
    timerConfig.timerDisable   = kFLEXIO_TimerDisableOnPreTimerDisable;
    timerConfig.timerEnable    = kFLEXIO_TimerEnableOnTriggerHigh;
    timerConfig.timerStop      = kFLEXIO_TimerStopBitDisabled;
    timerConfig.timerStart     = kFLEXIO_TimerStartBitDisabled;
    timerConfig.timerCompare   = (0xFFU << 8U) | baudDiv;
    FLEXIO_SetTimerConfig(base->flexio, base->timerStartIdx + MA_TIMER, &timerConfig);

    /* 5. Follow MA with a gate that rejects SL edges outside an active frame.
     * The gate output is high for the whole MA burst, so routing it to a pin
     * shows the frame window on a logic analyzer and also reveals what the
     * output does once the timer is disabled. */
    timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_TIMn(base->timerStartIdx + MA_TIMER);
    timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveHigh;
    timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
    timerConfig.pinConfig       = base->gatePinOutput ? kFLEXIO_PinConfigOutput : kFLEXIO_PinConfigOutputDisabled;
    timerConfig.pinSelect       = base->gatePinOutput ? base->gatePinIdx : base->maPinIdx;
    timerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
    timerConfig.timerMode       = kFLEXIO_TimerModeSingle16Bit;
    timerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
    timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
    timerConfig.timerReset      = kFLEXIO_TimerResetNever;
    timerConfig.timerDisable    = kFLEXIO_TimerDisableOnPreTimerDisable;
    timerConfig.timerEnable     = kFLEXIO_TimerEnableOnPrevTimerEnable;
    timerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;
    timerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled;
    timerConfig.timerCompare    = 0xFFFFU;
    FLEXIO_SetTimerConfig(base->flexio, base->timerStartIdx + MA_GATE_TIMER, &timerConfig);

    base->rxdBufferReady = false;

    /* Statistics survive on purpose: this function is also the timeout recovery
     * path, and a recovery that resets the counters would hide exactly the
     * failure rate a long run is trying to measure. */

    return 0;
}

void FLEXIO_BISSC_EnableRxInterrupt(FLEXIO_BISSC_Type *base, bool enable)
{
    assert(base != NULL);

    if (enable)
    {
        base->flexio->SHIFTSIEN |= 1UL << (base->shifterStartIdx + RXD_SHIFTER);
    }
    else
    {
        base->flexio->SHIFTSIEN &= ~(1UL << (base->shifterStartIdx + RXD_SHIFTER));
    }
}

void FLEXIO_BISSC_SwTrigger(FLEXIO_BISSC_Type *base)
{
    assert(base != NULL);

    for (uint32_t i = 0U; i < base->rxShifterNum; i++)
    {
        FLEXIO_ClearShifterStatusFlags(base->flexio, 1UL << (base->shifterStartIdx + RXD_SHIFTER + i));
    }
    FLEXIO_ClearTimerStatusFlags(base->flexio, (1UL << (base->timerStartIdx + MA_TIMER)) |
                                                   (1UL << (base->timerStartIdx + MA_GATE_TIMER)) |
                                                   (1UL << (base->timerStartIdx + RX_TIMER)));

    /* The MA timer keeps clocking while waiting for STR, so the trigger shifter
     * underruns once per 128 MA clocks. Clear the stale error before each frame
     * to keep SHIFTERR usable as a diagnostic. */
    FLEXIO_ClearShifterErrorFlags(base->flexio, 1UL << (base->shifterStartIdx + TRG_SHIFTER));

    base->flexio->SHIFTBUF[base->shifterStartIdx + TRG_SHIFTER] = 0U;
}

static uint8_t FLEXIO_BISSC_CRC6(uint32_t *data, uint32_t len)
{
    /*
     * Standard BiSS-C CRC6: poly 0x43, MSB-first, final XOR 0x3F. The receive
     * buffer stores WARN at bit 0 and MT at the most significant payload bit.
     * Up to two 32-bit words are combined for CRC calculation.
     */
    uint64_t payload;
    uint64_t mask;
    uint8_t  crc = 0U;

    if ((len == 0U) || (len > 64U))
    {
        return 0U;
    }

    payload = (uint64_t)data[0];
    if (len > 32U)
    {
        payload |= (uint64_t)data[1] << 32U;
    }

    mask = 1ULL << (len - 1U);

    for (uint32_t i = 0U; i < len; i++)
    {
        uint8_t bit      = ((payload & mask) != 0U) ? 1U : 0U;
        uint8_t feedback = ((crc & CRC6_MSB) != 0U) ? 1U : 0U;

        mask >>= 1U;
        feedback ^= bit;
        crc = (uint8_t)((crc << 1U) & CRC6_MASK);
        if (feedback != 0U)
        {
            crc ^= CRC6_POLY_BISSC;
        }
    }

    crc ^= CRC6_FINAL_XOR;

    return (uint8_t)(crc & CRC6_MASK);
}

static void FLEXIO_BISSC_ArrayRightShift(uint32_t *data, uint32_t wordLen, uint32_t shift)
{
    if ((wordLen != 0U) && (shift != 0U))
    {
        uint32_t wordShift = shift >> 5U;
        uint32_t bitShift  = shift & 31U;

        for (uint32_t i = 0U; i < wordLen; i++)
        {
            uint32_t newVal = 0U;

            if ((i + wordShift) < wordLen)
            {
                newVal = data[i + wordShift] >> bitShift;

                if ((bitShift != 0U) && ((i + wordShift + 1U) < wordLen))
                {
                    newVal |= data[i + wordShift + 1U] << (32U - bitShift);
                }
            }

            data[i] = newVal;
        }
    }
}

static uint32_t FLEXIO_BISSC_BitMask(uint8_t width)
{
    return (width >= 32U) ? 0xFFFFFFFFUL : ((1UL << width) - 1UL);
}

static void FLEXIO_BISSC_ArraySetBits(uint32_t *data, uint32_t wordLen, uint32_t bitOffset, uint8_t width,
                                      uint32_t value)
{
    for (uint32_t i = 0U; (i < width) && (i < 32U); i++)
    {
        uint32_t bitIndex  = bitOffset + i;
        uint32_t wordIndex = bitIndex >> 5U;
        uint32_t bitMask   = 1UL << (bitIndex & 31U);

        if (wordIndex >= wordLen)
        {
            break;
        }

        if (((value >> i) & 1U) != 0U)
        {
            data[wordIndex] |= bitMask;
        }
        else
        {
            data[wordIndex] &= ~bitMask;
        }
    }
}

static void FLEXIO_BISSC_ProcessRxWindow(FLEXIO_BISSC_Type *base)
{
    FLEXIO_BISSC_ArrayRightShift(base->rxdBuffer, base->rxShifterNum, base->rxTrailingBitLen);
}

void FLEXIO_BISSC_DataParser(FLEXIO_BISSC_Type *base)
{
    assert(base != NULL);
    assert((base->rxShifterNum != 0U) && (base->rxShifterNum <= FLEXIO_BISSC_MAX_RX_SHIFTERS));

    /* Parse a local copy: the shifts below are destructive, and rxdBuffer can be
     * rewritten by FLEXIO_BISSC_IRQHandler at any point. */
    uint32_t data[FLEXIO_BISSC_MAX_RX_SHIFTERS] = {0U};
    uint8_t  crcRecv;
    uint8_t  crcCal;

    for (uint32_t i = 0U; i < base->rxShifterNum; i++)
    {
        data[i] = base->rxdBuffer[i];
    }

    crcRecv = (uint8_t)(data[0] & CRC6_MASK);

    FLEXIO_BISSC_ArrayRightShift(data, base->rxShifterNum, 6U);
    crcCal         = FLEXIO_BISSC_CRC6(data, FLEXIO_BISSC_PAYLOAD_BITS(base->mtLen, base->stLen));
    base->crcMatch = (crcCal == crcRecv);

    base->warnBit  = (uint8_t)(data[0] & 1U);
    base->errorBit = (uint8_t)((data[0] >> 1U) & 1U);

    FLEXIO_BISSC_ArrayRightShift(data, base->rxShifterNum, 2U);
    base->st = data[0] & FLEXIO_BISSC_BitMask(base->stLen);

    FLEXIO_BISSC_ArrayRightShift(data, base->rxShifterNum, base->stLen);
    base->mt = data[0] & FLEXIO_BISSC_BitMask(base->mtLen);

    /* CDS and START sit above MT, at the top of the capture window. START is the
     * cheapest alignment check there is: the receive window is started by the STR
     * rising edge, so a correctly aligned frame always reads back START = 1. CRC
     * alone cannot make that statement, because it is computed over the payload
     * only and any regrouping of the same bits yields the same CRC. */
    FLEXIO_BISSC_ArrayRightShift(data, base->rxShifterNum, base->mtLen);
    base->cdsBit   = (uint8_t)(data[0] & 1U);
    base->startBit = (uint8_t)((data[0] >> 1U) & 1U);

    base->frameValid = base->crcMatch && (base->startBit != 0U);

    base->frameCount++;
    if (!base->crcMatch)
    {
        base->crcErrorCount++;
    }
    if (base->startBit == 0U)
    {
        base->startErrorCount++;
    }
}

bool FLEXIO_BISSC_RunParserSelfTest(FLEXIO_BISSC_Type *base, uint32_t mt, uint32_t st, uint8_t errorBit,
                                    uint8_t warnBit, uint8_t cdsBit)
{
    assert(base != NULL);

    uint32_t crcData[FLEXIO_BISSC_MAX_RX_SHIFTERS] = {0U};
    uint32_t expectedMt;
    uint32_t expectedSt;
    uint8_t  expectedError;
    uint8_t  expectedWarn;
    uint8_t  expectedCds;
    uint8_t  crc;

    if ((base->rxShifterNum == 0U) || (base->rxShifterNum > FLEXIO_BISSC_MAX_RX_SHIFTERS))
    {
        return false;
    }

    expectedMt    = mt & FLEXIO_BISSC_BitMask(base->mtLen);
    expectedSt    = st & FLEXIO_BISSC_BitMask(base->stLen);
    expectedError = errorBit & 1U;
    expectedWarn  = warnBit & 1U;
    expectedCds   = cdsBit & 1U;

    for (uint32_t i = 0U; i < base->rxShifterNum; i++)
    {
        base->rxdBuffer[i] = 0U;
    }

    /* Build the CRC input exactly as the parser sees it after stripping the CRC.
     * START and CDS are outside the CRC, so they are not part of this word. */
    FLEXIO_BISSC_ArraySetBits(crcData, base->rxShifterNum, 0U, 1U, expectedWarn);
    FLEXIO_BISSC_ArraySetBits(crcData, base->rxShifterNum, 1U, 1U, expectedError);
    FLEXIO_BISSC_ArraySetBits(crcData, base->rxShifterNum, 2U, base->stLen, expectedSt);
    FLEXIO_BISSC_ArraySetBits(crcData, base->rxShifterNum, 2U + base->stLen, base->mtLen, expectedMt);

    crc = FLEXIO_BISSC_CRC6(crcData, FLEXIO_BISSC_PAYLOAD_BITS(base->mtLen, base->stLen));

    /* Then build a receive buffer in post-ProcessRxWindow layout. */
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 0U, 6U, crc);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 6U, 1U, expectedWarn);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 7U, 1U, expectedError);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 8U, base->stLen, expectedSt);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 8U + base->stLen, base->mtLen, expectedMt);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 8U + base->stLen + base->mtLen, 1U, expectedCds);
    FLEXIO_BISSC_ArraySetBits(base->rxdBuffer, base->rxShifterNum, 9U + base->stLen + base->mtLen, 1U, 1U);

    FLEXIO_BISSC_DataParser(base);

    return base->frameValid && (base->mt == expectedMt) && (base->st == expectedSt) &&
           (base->errorBit == expectedError) && (base->warnBit == expectedWarn) && (base->cdsBit == expectedCds);
}

bool FLEXIO_BISSC_IsRxComplete(FLEXIO_BISSC_Type *base)
{
    assert(base != NULL);

    return ((base->flexio->SHIFTSTAT & (1UL << (base->shifterStartIdx + RXD_SHIFTER))) != 0U);
}

static void FLEXIO_BISSC_ReadRxShifters(FLEXIO_BISSC_Type *base, uint32_t *dest)
{
    /* Descending, so the shifter whose status flag is polled (RXD_SHIFTER) is read
     * last: reading SHIFTBUFBIS clears that flag, and clearing it before the rest
     * of the chain has been read would let the next window start mid-readout.
     *
     * dest == NULL discards the window and only clears the flags. SHIFTBUFBIS is
     * volatile, so the reads still happen. */
    for (uint32_t i = 0U; i < base->rxShifterNum; i++)
    {
        uint32_t word =
            base->flexio->SHIFTBUFBIS[base->shifterStartIdx + RXD_SHIFTER + base->rxShifterNum - 1U - i];

        if (dest != NULL)
        {
            dest[i] = word;
        }
    }
}

void FLEXIO_BISSC_ReadFrame(FLEXIO_BISSC_Type *base)
{
    assert(base != NULL);

    FLEXIO_BISSC_ReadRxShifters(base, base->rxdBuffer);

    FLEXIO_BISSC_ProcessRxWindow(base);
}

void FLEXIO_BISSC_GetDiag(FLEXIO_BISSC_Type *base, flexio_bissc_diag_t *diag)
{
    assert(base != NULL);
    assert(diag != NULL);

    diag->shiftStat = base->flexio->SHIFTSTAT;
    diag->shiftErr  = base->flexio->SHIFTERR;
    diag->timStat   = base->flexio->TIMSTAT;

    diag->rxComplete  = ((diag->shiftStat & (1UL << (base->shifterStartIdx + RXD_SHIFTER))) != 0U);
    diag->trgUnderrun = ((diag->shiftErr & (1UL << (base->shifterStartIdx + TRG_SHIFTER))) != 0U);

    /* The RX timer only reaches its compare after STR opened the window, so this
     * flag distinguishes a missing encoder response from a response that was not
     * captured. */
    diag->rxWindowDone = ((diag->timStat & (1UL << (base->timerStartIdx + RX_TIMER))) != 0U);

    /* On a healthy frame the RX timer disables MA well before MA reaches its own
     * compare, so this flag being set means MA was never stopped, i.e. STR never
     * arrived. */
    diag->maRanLong = ((diag->timStat & (1UL << (base->timerStartIdx + MA_TIMER))) != 0U);

    diag->gateWrapped = ((diag->timStat & (1UL << (base->timerStartIdx + MA_GATE_TIMER))) != 0U);
}

void FLEXIO_BISSC_ResetStats(FLEXIO_BISSC_Type *base)
{
    assert(base != NULL);

    base->frameCount      = 0U;
    base->crcErrorCount   = 0U;
    base->startErrorCount = 0U;
    base->timeoutCount    = 0U;
    base->overrunCount    = 0U;
}

bool FLEXIO_BISSC_ReadBlocking(FLEXIO_BISSC_Type *base, uint32_t timeoutUs)
{
    assert(base != NULL);

    /* Round up so a sub-step budget still gets one full step of tolerance. */
    uint32_t stepsLeft = (timeoutUs + FLEXIO_BISSC_POLL_STEP_US - 1U) / FLEXIO_BISSC_POLL_STEP_US;

    while (!FLEXIO_BISSC_IsRxComplete(base))
    {
        if (stepsLeft == 0U)
        {
            base->timeoutCount++;
            return false;
        }
        SDK_DelayAtLeastUs(FLEXIO_BISSC_POLL_STEP_US, SystemCoreClock);
        stepsLeft--;
    }

    FLEXIO_BISSC_ReadFrame(base);

    return true;
}

void FLEXIO_BISSC_IRQHandler(FLEXIO_BISSC_Type *base)
{
    if (base == NULL)
    {
        return;
    }

    if (!FLEXIO_BISSC_IsRxComplete(base))
    {
        return;
    }

    if (base->rxdBufferReady)
    {
        /* rxdBuffer still belongs to the consumer. Overwriting it here would tear
         * the copy FLEXIO_BISSC_DataParser is making, so drop this window instead.
         * The shifters must still be read, because reading them clears SHIFTSTAT,
         * and leaving the flag set would re-enter this handler forever. */
        base->overrunCount++;
        FLEXIO_BISSC_ReadRxShifters(base, NULL);
        return;
    }

    FLEXIO_BISSC_ReadFrame(base);

    /* Publish the buffer only after it is fully written. Without the barrier the
     * compiler is free to move the non-volatile rxdBuffer stores after the
     * volatile flag store, which at -O2 would let the consumer parse a partly
     * written frame. */
    __DMB();
    base->rxdBufferReady = true;
}
