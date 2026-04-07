/*
 *   @file  cascade_csirx.c
 *
 *   @brief
 *      Unit Test code for the MMWAVE Cascade 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/mssgenerated/ti_drivers_open_close.h>

#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/cascade_csirx.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/app_cli_state.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/chirp_header.h>
#include "FreeRTOS.h"
#include "task.h"
#include <ti/common/syscommon.h>

/* Set to 1 to enable CBUFF busy-guard (skip trigger if previous transfer
 * still in flight).  Set to 0 to match original am273x/ behavior. */
#ifndef CAPTURE_CBUFF_BUSY_GUARD
#define CAPTURE_CBUFF_BUSY_GUARD  0
#endif

uint32_t csirx0CommonCallBackArg = (uint32_t)CONFIG_CSIRX0;
uint32_t csirx1CommonCallBackArg = (uint32_t)CONFIG_CSIRX1;

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern MmwCascade_MCB    gMmwCascadeMCB;
extern MmwCascade_CSIRX_State gCSIRXState[MMWAVE_RADAR_DEVICES];

extern uint32_t gCSIRXErrorCode[MMWAVE_RADAR_DEVICES];
extern uint8_t CSIA_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIA_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIB_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];
extern uint8_t CSIB_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED];

/* File-scope flag: true = next EOL should activate CBUFF session.
 * Reset before MMWave_start to prevent spurious CSI-2 events from
 * prematurely consuming the activation. */
volatile Bool gCbuffFirstEOL = true;
static volatile Bool gCsirxSkipResetWait = false;

volatile uint32_t gCbuffTriggerCount = 0;
volatile uint32_t gCbuffSkipCount    = 0;

/* Startup-frame discard: set by ChirpGeom_reset, read by EOL callback */
volatile uint16_t gStartupDiscardFrames    = 0;
volatile uint16_t gFirstTransmittedFrameIdx = 0xFFFFU;
volatile uint32_t gLeakedBadFrames         = 0;

/*
 * gMasterRfFrameIdx — single source of truth for RF frame progression.
 *
 * INVARIANT (required for discard + byte sync):
 *   - During all EOL callbacks belonging to physical radar frame k,
 *     gMasterRfFrameIdx MUST equal k (k == 0, 1, 2, ...).
 *   - On master port (0) EOF, after finalizing frame k into geometry (or
 *     skipping geometry when over capacity), increment exactly once:
 *     gMasterRfFrameIdx becomes k + 1 before any EOL of frame k + 1 runs.
 *
 * Discard gating, firstTransmittedFrameIdx, and any "current frame" logic
 * MUST use only this counter — not gGeomCurrentFrameIdx.
 *
 * Finite edge case M + N > CHIRP_GEOM_MAX_FRAMES:
 *   - gMasterRfFrameIdx still advances for every EOF (discard + LVDS stay correct).
 *   - Per-frame geometry RAM holds at most CHIRP_GEOM_MAX_FRAMES linear slots
 *     (indices 0 .. cap-1); frames fi >= cap skip the RAM write. Intentional;
 *     not a regression of capture correctness.
 */
volatile uint32_t gMasterRfFrameIdx   = 0;
volatile Bool     gInfiniteRfCapture = false;

/**************************************************************************
 ************* Per-Frame Chirp Geometry (indexed by frame number) ***********
 **************************************************************************/
PerFrameChirpGeometry_t gChirpGeomByFrame[CHIRP_GEOM_MAX_FRAMES];
uint16_t                gChirpGeomConfiguredFrames = 0;

static volatile uint16_t gGeomExpectedPerFrame = 64;
/* Legacy UART / debug alias only: low 16 bits of gMasterRfFrameIdx — do not
 * use for discard or transmit decisions. */
static volatile uint16_t gGeomCurrentFrameIdx  = 0;
static volatile uint16_t gGeomEolInFrame       = 0;
static volatile uint32_t gGeomFirstEolCycles   = 0;
static volatile uint32_t gGeomLastEolCycles    = 0;

CsirxObservability_t gCsirxObs[MMWAVE_RADAR_DEVICES];

void ChirpGeom_reset(uint16_t expectedChirpsPerFrame, uint16_t numCaptureFrames,
                     uint16_t startupDiscard, Bool infiniteRf)
{
    memset(gChirpGeomByFrame, 0, sizeof(gChirpGeomByFrame));
    gInfiniteRfCapture = infiniteRf;
    if (infiniteRf)
    {
        /* Ring buffer over CHIRP_GEOM_MAX_FRAMES; slot = fi % cap */
        gChirpGeomConfiguredFrames = CHIRP_GEOM_MAX_FRAMES;
    }
    else if (numCaptureFrames > CHIRP_GEOM_MAX_FRAMES)
    {
        gChirpGeomConfiguredFrames = CHIRP_GEOM_MAX_FRAMES;
    }
    else
    {
        gChirpGeomConfiguredFrames = numCaptureFrames;
    }
    gGeomExpectedPerFrame       = expectedChirpsPerFrame;
    gMasterRfFrameIdx           = 0;
    gGeomCurrentFrameIdx        = 0;
    gGeomEolInFrame             = 0;
    gGeomFirstEolCycles         = 0;
    gGeomLastEolCycles          = 0;
    gStartupDiscardFrames       = startupDiscard;
    gFirstTransmittedFrameIdx   = 0xFFFFU;
    gLeakedBadFrames            = 0;
    memset(gCsirxObs, 0, sizeof(gCsirxObs));
}

void CsirxObs_pollAndAccumulate(MmwCascade_MCB *CascadeMCB)
{
    uint32_t devIdx;
    for (devIdx = 0; devIdx < MMWAVE_RADAR_DEVICES; devIdx++)
    {
        if (CascadeMCB->csiRxHandle[devIdx] == NULL)
            continue;

        CSIRX_ComplexioLanesIntr lanesIntr;
        memset(&lanesIntr, 0, sizeof(lanesIntr));
        if (CSIRX_complexioGetPendingIntr(CascadeMCB->csiRxHandle[devIdx], &lanesIntr) == SystemP_SUCCESS)
        {
            uint8_t lane;
            for (lane = 0; lane < 4; lane++)
            {
                if (lanesIntr.dataLane[lane].isStateTransitionToULPM)
                    gCsirxObs[devIdx].ulpmEnter++;
                if (lanesIntr.dataLane[lane].isControlError)
                    gCsirxObs[devIdx].controlError++;
                if (lanesIntr.dataLane[lane].isEscapeEntryError)
                    gCsirxObs[devIdx].escapeEntryError++;
                if (lanesIntr.dataLane[lane].isStartOfTransmissionSyncError)
                    gCsirxObs[devIdx].sotSyncError++;
                if (lanesIntr.dataLane[lane].isStartOfTransmissionError)
                    gCsirxObs[devIdx].sotError++;
            }
            CSIRX_complexioClearAllIntr(CascadeMCB->csiRxHandle[devIdx]);
        }

        CSIRX_ContextIntr ctxIntr;
        memset(&ctxIntr, 0, sizeof(ctxIntr));
        if (CSIRX_contextGetPendingIntr(CascadeMCB->csiRxHandle[devIdx], MMW_CASCADE_CSI2_CONTEXT, &ctxIntr) == SystemP_SUCCESS)
        {
            if (ctxIntr.isPayloadChecksumMismatch)
                gCsirxObs[devIdx].payloadChecksumMismatch++;
            CSIRX_contextClearAllIntr(CascadeMCB->csiRxHandle[devIdx], MMW_CASCADE_CSI2_CONTEXT);
        }

        uint32_t shortPkt = 0;
        if (CSIRX_commonGetGenericShortPacket(CascadeMCB->csiRxHandle[devIdx], &shortPkt) == SystemP_SUCCESS)
        {
            if (shortPkt != 0)
                gCsirxObs[devIdx].genericShortPacket++;
        }
    }
}

void resetCbuffEOLState(void)
{
    gCbuffFirstEOL      = true;
    gCbuffTriggerCount  = 0;
    gCbuffSkipCount     = 0;
    
    extern volatile uint32_t gEolCountPort0;
    extern volatile uint32_t gEolCountPort1;
    gEolCountPort0 = 0;
    gEolCountPort1 = 0;

    /* Re-run Fix: Reset the static flags used in the ISRs so the second
     * run of sensorStart doesn't skip session activation. */
    extern Bool gFirstTimeFlag_CbuffActivate;
    gFirstTimeFlag_CbuffActivate = true;

    extern void resetPingPongState(void);
    resetPingPongState();
}

void MmwCascade_csirxSetSkipResetWait(Bool enable)
{
    gCsirxSkipResetWait = enable;
}

/**************************************************************************
 *********************** Cascade Unit Test Functions **********************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      Callback function for common.irq interrupt, generated when
 *      end of frame code and line code detected, as per the set configuration.
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *  @param[out] IRQ        CSIRX common irq
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxCommonCallback(CSIRX_Handle handle, void *arg,
                              struct CSIRX_CommonIntr_s *IRQ)
{
    uint8_t i;
    uint32_t csiRxInstance = *((uint32_t*)arg);

    DebugP_assert (handle != NULL);

    gCSIRXState[csiRxInstance].callbackCount.common++;

    gCSIRXState[csiRxInstance].IRQ.common = *IRQ;

    /* Counts book-keeping */
    if(IRQ->isOcpError == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isOCPerror++;
    }
    if(IRQ->isComplexioError == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isComplexIOerror++; 
    }
    if(IRQ->isFifoOverflow == true)
    {
        gCSIRXState[csiRxInstance].commonIRQcount.isFIFOoverflow++;
    }

    if(IRQ->isComplexioError)
    {
        gCSIRXErrorCode[csiRxInstance] = CSIRX_complexioGetPendingIntr(handle, &gCSIRXState[csiRxInstance].IRQ.complexIOlanes);
        DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);

        gCSIRXErrorCode[csiRxInstance] = CSIRX_complexioClearAllIntr(handle);
        DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);
    }

    for(i = 0; i < CONFIG_CSIRX_NUM_INSTANCES; i++)
    {
        if(IRQ->isContextIntr[i] == true)
        {
            gCSIRXErrorCode[csiRxInstance] = CSIRX_contextGetPendingIntr(handle, i, &gCSIRXState[csiRxInstance].IRQ.context[i]);
            DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);

            if(gCSIRXState[csiRxInstance].IRQ.context[i].isFrameEndCodeDetect == true)
            {
                gCSIRXState[csiRxInstance].contextIRQcounts[i].frameEndCodeDetect++;
            }

            gCSIRXErrorCode[csiRxInstance] = CSIRX_contextClearAllIntr(handle, i);
            DebugP_assert(gCSIRXErrorCode[csiRxInstance] == SystemP_SUCCESS);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Callback function for when end of frame detected.
 *      [SEMANTIC TRACE]: This is triggered by the AM273x CSIRX hardware 
 *      when it receives a MIPI CSI-2 "Frame End" short packet from the master 
 *      CSI-2 transmitter (AWR2243). This signifies the physical end of a radar frame.
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxCombinedEOFcallback(CSIRX_Handle handle, void *arg)
{
    DebugP_assert(handle != NULL);

    uint32_t portIdx = *((uint32_t*)arg);
    gCSIRXState[portIdx].callbackCount.combinedEOF++;

    /* Geometry: master port (0) only.  Slave EOF does not finalize a frame. */
    if (portIdx == 0U)
    {
        uint32_t fi   = gMasterRfFrameIdx;
        uint16_t cfg  = gChirpGeomConfiguredFrames;

        /* Infinite RF: rolling ring. Finite: linear slot fi while fi < cfg.
         * When M+N > CHIRP_GEOM_MAX_FRAMES (finite), fi >= cfg skips RAM only;
         * gMasterRfFrameIdx still advances below — discard/transmit unchanged. */
        if (gInfiniteRfCapture && cfg > 0U)
        {
            uint32_t ring        = fi % (uint32_t)CHIRP_GEOM_MAX_FRAMES;
            uint32_t eofCyc      = CycleCounterP_getCount32();
            PerFrameChirpGeometry_t *slot = &gChirpGeomByFrame[ring];

            slot->logicalFrameIdx = fi;
            slot->frameIdx        = (uint16_t)(fi & 0xFFFFU);
            slot->expectedCount   = gGeomExpectedPerFrame;
            slot->observedCount   = gGeomEolInFrame;
            slot->firstEolCycles  = gGeomFirstEolCycles;
            slot->lastEolCycles   = gGeomLastEolCycles;
            slot->eofCycles       = eofCyc;
            slot->chirpIdSource   = CHIRP_ID_SW_COUNTER;
            slot->isComplete      = 1U;
            slot->valid           = GEOM_ENTRY_VALID;
            slot->reserved        = 0U;
        }
        else if (!gInfiniteRfCapture && cfg > 0U && fi < (uint32_t)cfg)
        {
            uint32_t eofCyc = CycleCounterP_getCount32();
            PerFrameChirpGeometry_t *slot = &gChirpGeomByFrame[fi];

            slot->logicalFrameIdx = fi;
            slot->frameIdx        = (uint16_t)(fi & 0xFFFFU);
            slot->expectedCount   = gGeomExpectedPerFrame;
            slot->observedCount   = gGeomEolInFrame;
            slot->firstEolCycles  = gGeomFirstEolCycles;
            slot->lastEolCycles   = gGeomLastEolCycles;
            slot->eofCycles       = eofCyc;
            slot->chirpIdSource   = CHIRP_ID_SW_COUNTER;
            slot->isComplete      = 1U;
            slot->valid           = GEOM_ENTRY_VALID;
            slot->reserved        = 0U;
        }

        gMasterRfFrameIdx++;
        gGeomCurrentFrameIdx = (uint16_t)(gMasterRfFrameIdx & 0xFFFFU);

        gGeomEolInFrame      = 0;
        gGeomFirstEolCycles  = 0;
        gGeomLastEolCycles   = 0;
    }
}

/**
 *  @b Description
 *  @n
 *      Callback function for when end of line detected.
 *      [SEMANTIC TRACE]: This is triggered by the AM273x CSIRX hardware based on
 *      the 'pixelsPerLine' and 'bytesPerPixel' configuration in CSIRX context setup.
 *      It typically fires after a fixed number of bytes (a "line" or "chirp") 
 *      is received into the ping/pong buffer. The software then triggers the CBUFF 
 *      SW session to DMA this line out over LVDS to the DCA1000.
 * 
 *  @param[in] handle      CSIRX Handle
 *  @param[in] arg         Callback function argument
 *
 *  @retval
 *      Not Applicable.
 */
/* Shared EOL state for CBUFF synchronization */
volatile uint32_t gEolCountPort0 = 0;
volatile uint32_t gEolCountPort1 = 0;

volatile uint32_t gCbuffActivateCycles = 0;
volatile uint32_t gCbuffTriggerCycles = 0;

#define CBUFF_CONFIG_REG_0  (0x06040000U)

extern void configureTransfer(void);
extern void skipTransfer(void);
extern volatile uint32_t gCbuffTriggerCount;

/* Re-run Fix: Must be global so resetCbuffEOLState can reset it */
Bool gFirstTimeFlag_CbuffActivate = true;

/**
 *  @b Description
 *      EOL callback for Port 0 (Master).
 *      Restores the original TI trigger model:
 *        - First EOL: activate CBUFF session
 *        - Subsequent EOLs: configureTransfer + SW trigger
 *      Includes CBUFF busy guard: skip trigger if previous
 *      EDMA transfer hasn't completed to prevent FIFO overflow.
 */
void MmwCascade_combinedEOLcallback(CSIRX_Handle handle, void *arg)
{
    int32_t errCode = 0;
    uint32_t t_start;

    DebugP_assert(handle != NULL);

    /* ---- Startup-frame discard gate (per-EOL; index = gMasterRfFrameIdx only) ---- */
    if (gMasterRfFrameIdx < (uint32_t)gStartupDiscardFrames)
    {
        skipTransfer();
        goto eol_done;
    }

    if(gFirstTimeFlag_CbuffActivate)
    {
        t_start = CycleCounterP_getCount32();
        if(CBUFF_activateSession(gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle, &errCode) < 0)
        {
            DebugP_assert(0);
        }
        gCbuffActivateCycles = CycleCounterP_getCount32() - t_start;
        gFirstTimeFlag_CbuffActivate = false;
        gCbuffTriggerCount++;

        if (gFirstTransmittedFrameIdx == 0xFFFFU)
            gFirstTransmittedFrameIdx = (uint16_t)gMasterRfFrameIdx;
    }
    else
    {
        /* CBUFF busy guard */
        if (gCbuffTriggerCount >
            gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swFrameDoneCount + 1U)
        {
            gCbuffSkipCount++;
            goto eol_done;
        }

        t_start = CycleCounterP_getCount32();
        configureTransfer();

        *((volatile uint32_t *)CBUFF_CONFIG_REG_0) |= 1 << 25;
        *((volatile uint32_t *)CBUFF_CONFIG_REG_0) |= 1 << 24;

        gCbuffTriggerCount++;

        if (gCbuffTriggerCycles == 0) {
            gCbuffTriggerCycles = CycleCounterP_getCount32() - t_start;
        }
    }

eol_done:
    {
        uint32_t cyc = CycleCounterP_getCount32();
        if (gGeomEolInFrame == 0)
            gGeomFirstEolCycles = cyc;
        gGeomLastEolCycles = cyc;
        gGeomEolInFrame++;
    }
    gCSIRXState[*((uint32_t*)arg)].callbackCount.combinedEOL++;
    gEolCountPort0++;
}

/**
 *  @b Description
 *      EOL callback for Port 1 (Slave).
 *      Only increments the counter — Port 0 drives CBUFF triggers.
 */
void MmwCascade_csirx1EOLcallback(CSIRX_Handle handle, void *arg)
{
    DebugP_assert(handle != NULL);

    gCSIRXState[*((uint32_t*)arg)].callbackCount.combinedEOL++;
    gEolCountPort1++;
}


/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called whenever start of frame interrupt is registered.
 *
 *  @param[in] handle CSIRX Handle
 *  @param[in] contextId Context ID
 * 
 *  @retval None
 */
void mmwCascade_csirxSOF0callback(CSIRX_Handle handle, uint32_t arg, uint8_t contextId)
{
    DebugP_assert(handle != NULL);
    /* DebugP_assert(contextId == MMW_CASCADE_CSI2_CONTEXT); */
    gCSIRXState[arg].callbackCount.SOF0++;
}


/**
 *  @b Description
 *  @n
 *      Initializes  CSIRX.
 *      
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxInit(MmwCascade_MCB  *CascadeMCB)
{
    int32_t errorCode;
    uint32_t u32DevIdx;

    /* Initialize global variables */
    memset(gCSIRXState, 0, MMWAVE_RADAR_DEVICES * sizeof(MmwCascade_CSIRX_State));

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        CascadeMCB->csiRxHandle[u32DevIdx] = NULL;

        gCSIRXState[u32DevIdx].isReceivedPayloadCorrect = true;

        gCSIRXErrorCode[u32DevIdx] = 0;       /* Only for debug */
    }

    errorCode = CSIRX_init();
    if (errorCode != SystemP_SUCCESS)
    {
        test_print("Error: CSIRX initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}


/**
 *  @b Description
 *  @n
 *      Open CSI RX instance.
 * 
 *  @param[in] instanceId      CSIRX Instance ID
 *  @param[out] errCode        Error Code
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxOpen(MmwCascade_MCB  *CascadeMCB, int32_t *errCode)
{
    uint32_t u32DevIdx;

     for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0)
        {
            CascadeMCB->csiRxHandle[u32DevIdx] = CSIRX_open(CONFIG_CSIRX0);
        }
        else
        {
            CascadeMCB->csiRxHandle[u32DevIdx] = CSIRX_open(CONFIG_CSIRX1);
        }

        if(CascadeMCB->csiRxHandle[u32DevIdx] == NULL)
        {
            *errCode = -1;
        }
        else
        {
            /* Reset CSI-2 */
            *errCode = CSIRX_reset(CascadeMCB->csiRxHandle[u32DevIdx]);
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Performs CSI Configuration
 * 
 *  @param[in] obj      Pointer to data path object
 * gCSIRXCfg
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_CSIConfig(MmwCascade_MCB  *CascadeMCB)
{
    int32_t errorCode;
    uint32_t u32DevIdx;
    volatile bool isComplexIOresetDone;

    CSIRX_ContextConfig *ptrCsirxContextConfig = NULL;

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0 )
        {
            gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pingAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PingBuf);
            gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pongAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PongBuf);
            
            gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.lineOffset = 0;
            
            ptrCsirxContextConfig = &gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT];
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.commonCallbackArgs = (void*)&csirx0CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.combinedEndOfFrameCallbackArgs = (void*)&csirx0CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX0].intrCallbacks.combinedEndOfLineCallbackArgs = (void*)&csirx0CommonCallBackArg;
        }
        else
        {
            gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pingAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PingBuf);
            gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.pongAddress        = (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PongBuf);
            
            gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.lineOffset = 0;

            ptrCsirxContextConfig = &gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT];
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.commonCallbackArgs = (void*)&csirx1CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.combinedEndOfFrameCallbackArgs = (void*)&csirx1CommonCallBackArg;
            gCsirxCommonConfig[CONFIG_CSIRX1].intrCallbacks.combinedEndOfLineCallbackArgs = (void*)&csirx1CommonCallBackArg;
        }

        /* config complex IO - lanes and IRQ */
        errorCode = CSIRX_complexioSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxComplexioConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            test_print("CSIRX_configComplexIO failed, errorCode = %d\n", errorCode);
            DebugP_assert (0);
            return;
        }

        /* deassert complex IO reset */
        errorCode = CSIRX_complexioDeassertReset(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_deassertComplexIOreset failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* config DPHY */
        errorCode = CSIRX_dphySetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxDphyConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configDPHY failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
         errorCode = CSIRX_complexioSetPowerCommand(CascadeMCB->csiRxHandle[u32DevIdx], 1);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_setComplexIOpowerCommand failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        uint8_t isComplexIOpowerStatus = 1;
        do
        {
            errorCode = CSIRX_complexioGetPowerStatus(CascadeMCB->csiRxHandle[u32DevIdx], &isComplexIOpowerStatus);
            if(errorCode != SystemP_SUCCESS)
            {
                /* test_print("CSIRX_getComplexIOpowerStatus failed, errorCode = %d\n", errorCode); */
                DebugP_assert (0);
                return;
            }
        }while(isComplexIOpowerStatus == 0);

        /* config common */
        errorCode = CSIRX_commonSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], &gCsirxCommonConfig[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configCommon failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        test_print("[DEBUG] CSIRX%d Context CFG -> linesForIntr=%d, genEveryNumLines=%d, EOLLinePulse=%d, EOFPulse=%d\n",
                    u32DevIdx, 
                    ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT].numLinesForIntr,
                    ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT].isGenerateIntrEveryNumLinesForIntr,
                    ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT].isEndOfLinePulseEnabled,
                    ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT].isEndOfFramePulseEnabled);

        errorCode = CSIRX_contextSetConfig(CascadeMCB->csiRxHandle[u32DevIdx], MMW_CASCADE_CSI2_CONTEXT, &ptrCsirxContextConfig[MMW_CASCADE_CSI2_CONTEXT]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_configContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* enable context */
        errorCode = CSIRX_contextEnable(CascadeMCB->csiRxHandle[u32DevIdx], MMW_CASCADE_CSI2_CONTEXT);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_enableContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* enable interface */
        errorCode = CSIRX_commonEnable(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_enableInterface failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
    }

    /* MSS initialization can continue */
    SemaphoreP_post(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle);

    if (gCsirxSkipResetWait)
    {
        return;
    }

    /* Reset will be really effective when both front-end will start to drive CSI-2 lines */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        /* Wait until complex IO reset complete */
        do
        {
            errorCode= CSIRX_complexioIsResetDone(CascadeMCB->csiRxHandle[u32DevIdx], (bool *)&isComplexIOresetDone);
            if(errorCode != SystemP_SUCCESS)
            {
                /* test_print("CSIRX_isComplexIOresetDone failed, errorCode = %d\n", errorCode); */
                DebugP_assert (0);
                return;
            }
            if (isComplexIOresetDone == false)
            {
                ClockP_usleep(1 * 1000);
            }
        }while(isComplexIOresetDone == false);
    }
    if(isComplexIOresetDone == false)
    {
        /* test_print("CSIRX_isComplexIOresetDone attempts exceeded\n"); */
        DebugP_assert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close CSI RX instance.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_csirxClose(MmwCascade_MCB  *CascadeMCB)
{
    int32_t             errorCode;
    uint32_t u32DevIdx;
    
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        /* disable context */
        errorCode = CSIRX_contextDisable(CascadeMCB->csiRxHandle[u32DevIdx], 0);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("Error: CSIRX_disableContext failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* disable interface */
        errorCode = CSIRX_commonDisable(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_disableInterface failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }

        /* close instance */
        errorCode = CSIRX_close(CascadeMCB->csiRxHandle[u32DevIdx]);
        if(errorCode != SystemP_SUCCESS)
        {
            /* test_print("CSIRX_close failed, errorCode = %d\n", errorCode); */
            DebugP_assert (0);
            return;
        }
    }
}
