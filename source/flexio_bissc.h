/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FLEXIO_BISSC_H_
#define _FLEXIO_BISSC_H_

#include "fsl_flexio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define FLEXIO_BISSC_MAX_RX_SHIFTERS (FLEXIO_SHIFTBUF_COUNT - 1U)

/*! @brief Largest supported RX capture window.
 *
 * Two limits apply, and the smaller one is used:
 * - the RX timer runs in dual 8-bit baud/bit mode, where the upper compare byte
 *   holds (2 * windowBits - 1), so 128 bits is the hardware ceiling;
 * - the window has to fit in the receive shifter chain, 32 bits per shifter.
 * On MCXA366 the four shifters leave three for receiving, so the shifter chain
 * is the smaller limit at 96 bits.
 */
#define FLEXIO_BISSC_MAX_RX_WINDOW_BITS \
    (((FLEXIO_BISSC_MAX_RX_SHIFTERS * 32U) < 128U) ? (FLEXIO_BISSC_MAX_RX_SHIFTERS * 32U) : 128U)

/*! @brief Bits covered by CRC6: MT + ST + ERROR + WARN. */
#define FLEXIO_BISSC_PAYLOAD_BITS(mtLen, stLen) ((uint32_t)(mtLen) + (uint32_t)(stLen) + 2U)

/*! @brief Bits from STR through CRC6: START + CDS + payload + CRC6. */
#define FLEXIO_BISSC_FRAME_BITS(mtLen, stLen) (FLEXIO_BISSC_PAYLOAD_BITS(mtLen, stLen) + 8U)

/*! @brief Poll granularity of FLEXIO_BISSC_ReadBlocking, in microseconds.
 *
 * Timeouts are expressed in real time instead of loop iterations so that they do
 * not change with the optimization level. The step is coarse enough that the
 * per-call overhead of SDK_DelayAtLeastUs stays a small fraction of it, and a
 * completed frame is only reported up to one step late: the payload is already
 * latched in SHIFTBUF, so nothing is lost by the delay.
 */
#define FLEXIO_BISSC_POLL_STEP_US 5U

typedef struct _flexio_bissc_type
{
    FLEXIO_Type  *flexio;          /*!< FlexIO base pointer. */
    uint8_t       slPinIdx;        /*!< FlexIO pin select for SL data input. */
    uint8_t       maPinIdx;        /*!< FlexIO pin select for MA clock output. */
    uint8_t       gatePinIdx;      /*!< FlexIO pin select for the MA gate debug output. */
    bool          gatePinOutput;   /*!< Drive the MA gate timer output on gatePinIdx for probing. */
    uint8_t       shifterStartIdx; /*!< FlexIO shifter start index. */
    uint8_t       timerStartIdx;   /*!< FlexIO timer start index. */
    uint8_t       rxShifterNum;    /*!< Output of Init: shifter count used for receiving. */
    bool          hwTrigger;       /*!< FlexIO MA timer trigger source by hardware or software. */
    uint32_t      baudRateBps;     /*!< BiSS-C communication baud rate in bits per second. */
    uint8_t       mtLen;           /*!< Bit length of multi-turn data. */
    uint8_t       stLen;           /*!< Bit length of single-turn data. */
    uint8_t       rxTrailingBitLen; /*!< Trailing samples removed before frame parsing. */
    uint8_t       rxWindowBitLen;   /*!< Output of Init: STR through CRC plus trailing samples. */
    uint32_t      rxdBuffer[FLEXIO_BISSC_MAX_RX_SHIFTERS]; /*!< Frame payload, trailing samples removed. */

    /*! @brief Set by the ISR once rxdBuffer holds a frame, cleared by the consumer.
     *
     * Also acts as the ownership flag for rxdBuffer: while it is set, the buffer
     * belongs to the consumer and the ISR drops incoming frames instead of
     * overwriting it. Clear it only after the frame has been parsed.
     */
    volatile bool rxdBufferReady;

    uint32_t      mt;             /*!< Multi-turn value, raw bits, not sign extended. */
    uint32_t      st;             /*!< Single-turn value. */
    uint8_t       errorBit;       /*!< Error bit value, raw, never inverted. */
    uint8_t       warnBit;        /*!< Warning bit value, raw, never inverted. */
    uint8_t       cdsBit;         /*!< CDS bit of this frame, sensor to master register channel. */
    uint8_t       startBit;       /*!< START bit of this frame, 1 on a correctly aligned frame. */
    bool          crcMatch;       /*!< CRC check matched or not. */
    bool          frameValid;     /*!< START seen and CRC matched. */

    /* Statistics. Reset by FLEXIO_BISSC_ResetStats only, so that the timeout
     * recovery path, which re-runs Init, does not wipe the history of a long run. */
    uint32_t          frameCount;      /*!< Frames handed to the parser. */
    uint32_t          crcErrorCount;   /*!< Parsed frames whose CRC6 did not match. */
    uint32_t          startErrorCount; /*!< Parsed frames whose START bit was 0. */
    uint32_t          timeoutCount;    /*!< Receive windows that never completed. */
    volatile uint32_t overrunCount;    /*!< Frames the ISR dropped because rxdBuffer was still claimed. */
} FLEXIO_BISSC_Type;

/*! @brief FlexIO status snapshot with the BiSS-C specific bits already decoded.
 *
 * The raw registers are kept as well: the decoded flags answer "which stage
 * failed", the raw values are what gets compared against the reference manual
 * when the answer is unexpected.
 */
typedef struct _flexio_bissc_diag
{
    uint32_t shiftStat;    /*!< Raw SHIFTSTAT. */
    uint32_t shiftErr;     /*!< Raw SHIFTERR. */
    uint32_t timStat;      /*!< Raw TIMSTAT. */
    bool     rxComplete;   /*!< Receive chain holds a full window. */
    bool     rxWindowDone; /*!< RX timer reached its compare, so STR was seen. */
    bool     maRanLong;    /*!< MA timer reached its own compare, so it was never stopped by STR. */
    bool     gateWrapped;  /*!< MA gate timer wrapped, so MA ran for a whole gate period. */
    bool     trgUnderrun;  /*!< Trigger shifter underran, which only happens once MA has clocked. */
} flexio_bissc_diag_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Configures FlexIO as a BiSS-C master and derives the RX capture window.
 *
 * Also resets the FlexIO instance, so any interrupt has to stay masked across
 * the call. rxWindowBitLen and rxShifterNum are outputs, computed from mtLen,
 * stLen and rxTrailingBitLen. Statistics are left untouched, so this can double
 * as a recovery step in the middle of a long run.
 *
 * @retval 0  Configuration applied.
 * @retval -1 Clock, baud rate, field length or trailing sample count out of range.
 */
int  FLEXIO_BISSC_Init(FLEXIO_BISSC_Type *base, uint32_t srcClock_Hz);

/*! @brief Enables or disables the receive shifter status interrupt. */
void FLEXIO_BISSC_EnableRxInterrupt(FLEXIO_BISSC_Type *base, bool enable);

/*! @brief Clears stale status and starts one MA burst from software. */
void FLEXIO_BISSC_SwTrigger(FLEXIO_BISSC_Type *base);

/*! @brief Returns true once the receive shifter chain holds a complete window. */
bool FLEXIO_BISSC_IsRxComplete(FLEXIO_BISSC_Type *base);

/*! @brief Copies the shifter chain into rxdBuffer and removes the trailing samples. */
void FLEXIO_BISSC_ReadFrame(FLEXIO_BISSC_Type *base);

/*! @brief Feeds the parser with synthetic fields to check CRC6 and field layout. */
bool FLEXIO_BISSC_RunParserSelfTest(FLEXIO_BISSC_Type *base, uint32_t mt, uint32_t st, uint8_t errorBit,
                                    uint8_t warnBit, uint8_t cdsBit);

/*! @brief Splits rxdBuffer into START, CDS, MT, ST, ERROR, WARN and checks CRC6. */
void FLEXIO_BISSC_DataParser(FLEXIO_BISSC_Type *base);

/*!
 * @brief Polls for one receive window and reads it out.
 *
 * @param timeoutUs Budget in microseconds, rounded up to FLEXIO_BISSC_POLL_STEP_US.
 *                  The buffer is always polled at least once.
 */
bool FLEXIO_BISSC_ReadBlocking(FLEXIO_BISSC_Type *base, uint32_t timeoutUs);

/*!
 * @brief Reads the FlexIO status registers and decodes how far the frame got.
 *
 * Meaningful right after a timeout, because FLEXIO_BISSC_SwTrigger clears these
 * flags before every frame, so what is left belongs to the frame that failed.
 */
void FLEXIO_BISSC_GetDiag(FLEXIO_BISSC_Type *base, flexio_bissc_diag_t *diag);

/*! @brief Zeroes the statistics counters. */
void FLEXIO_BISSC_ResetStats(FLEXIO_BISSC_Type *base);

/*! @brief FlexIO interrupt service routine, called from the part's IRQ wrapper. */
void FLEXIO_BISSC_IRQHandler(FLEXIO_BISSC_Type *base);

#if defined(__cplusplus)
}
#endif

#endif /* _FLEXIO_BISSC_H_ */
