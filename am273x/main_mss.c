/*
 *   @file  main_full_mss.c
 *
 *   @brief
 *      Unit Test code for the mmWave 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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
#define DebugP_LOG_ENABLED 1

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* MCU Plus Include Files. */
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/mssgenerated/ti_drivers_config.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/mssgenerated/ti_board_config.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/mssgenerated/ti_drivers_open_close.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/mssgenerated/ti_board_open_close.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "FreeRTOS.h"
#include "task.h"
#include <drivers/csirx.h>
#include <drivers/uart.h>


/* mmWave SK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/testlogger/logger.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/cascade_csirx.h>
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/chirp_header.h>

/**************************************************************************
 ******************************** MACROS  *********************************
 **************************************************************************/
/**
 * @brief
 *  The DCA1000EVM FPGA needs a minimum delay of 12ms between Bit clock starts and
 *  actual LVDS Data start to lock the LVDS PLL IP. This is documented in the DCA UG
 */
#define HSI_DCA_MIN_DELAY_MSEC     (12U * 1000U)
#define CBUFF_CONFIG_REG_0         (0x06040000U)

#define APP_TASK_PRI               (5U)
#define CSI_CONFIG_TASK_PRI        (6U)
#define MMW_CTRL_TASK_PRI          (7U)
#define MMW_TEST_TASK_PRI          (5U)

#define APP_TASK_STACK_SIZE        (32U * 1024U)
#define CSI_CONFIG_TASK_STACK_SIZE (32U * 1024U)
#define MMW_CTRL_TASK_STACK_SIZE   (16U * 1024U)
#define MMW_TEST_TASK_STACK_SIZE   (16U * 1024U)

/* Experimental runtime CSIRX rearm before MMWave_start.
 * 0: disabled (default, safest)
 * 1: enabled with non-blocking/timeout behavior */
#ifndef CSIRX_RUNTIME_REARM_MODE
#define CSIRX_RUNTIME_REARM_MODE   (0U)
#endif

/* Post-start bounded poll for CSIRX reset-done in rearm mode. */
#define CSIRX_REARM_RESET_DONE_POLL_US    (1000U)
#define CSIRX_REARM_RESET_DONE_TIMEOUT_US (50000U)

/* size used for communication with PMIC. */
#define PMIC_MSGSIZE                  (4U)

/* OFFSET for configuring BUCK4. */
#define PMIC_CONFIG_BUCK4_REG_ADDR    (0x0A)

TaskHandle_t    gAppTask;
StaticTask_t    gAppTaskObj;

TaskHandle_t    gCsiConfigTask;
StaticTask_t    gCsiConfigTaskObj;

TaskHandle_t    gMmwCtrlTask;
StaticTask_t    gMmwCtrlTaskObj;

StackType_t gAppTskStackMain[APP_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gMmwCtrlTskStack[MMW_CTRL_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gCsiRxCfgTskStack[CSI_CONFIG_TASK_STACK_SIZE] __attribute__((aligned(32)));

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

uint8_t CSIA_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIA_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIB_PingBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

uint8_t CSIB_PongBuf[BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED]__attribute__ ((aligned(64), section(".l3ram")));

/**
 * @brief
 *  Initialize the MCPI Log Message Buffer
 */
MCPI_LOGBUF_INIT(9216);


/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwCascade_MCB    gMmwCascadeMCB = {0U};

/**
 * @brief
 *  Global Variable used for storing CSIRXA instance
 */
MmwCascade_CSIRX_State gCSIRXState[MMWAVE_RADAR_DEVICES] ;

/**
 * @brief
 *  Global Variable for CSIRX error monitoring
 */
uint32_t gCSIRXErrorCode[MMWAVE_RADAR_DEVICES];

/**************************************************************************
 ************************* CLI Runtime State *******************************
 **************************************************************************/
#include <ti/utils/test/cascade/am273x_LVDS_CLI/am273x/app_cli_state.h>

/* State and configuration definitions */
/* NOTE: gCliState is volatile but not mutex-protected.  This is safe as long
 * as only two tasks (CLI + init) access it, both on the same Cortex-R5 core
 * with cooperative same-priority FreeRTOS scheduling.  If multi-core or
 * preemption is added, a semaphore guard must be introduced. */
volatile CLI_State gCliState = CLI_STATE_INIT;

/* Default configuration — profile values match the original SDK's
 * Mmwave_populateDefaultProfileCfg() in common.c (frame-mode profile).
 * IMPORTANT: frame.numAdcSamples MUST equal profile.numAdcSamples * 2
 * for complex ADC format (adcFmt == 1 or 2).  Keep both in sync. */
CLI_CaptureConfig gCfg = {
    .rxMask     = 0xFF,
    .txMask     = 0x3F, /* default 6 TX for 2-chip cascade */
    .tdmEnabled = 0,
    .adcBits    = 2, /* 16-bit */
    .adcFmt     = 1, /* complex 1x */
    .chirpHeaderEnabled = 0,
    .frame      = {
        .chirpStartIdx      = 0,
        .chirpEndIdx        = 0,
        .numLoops           = 64,  /* matches TEST_NUM_LOOPS */
        .numFrames          = 100, /* default */
        .numAdcSamples      = 256 * 2, /* profile.numAdcSamples * 2 (complex) */
        .framePeriodicity   = 10000000, /* 50ms in 5ns units (matches SDK: 50*1000000/5) */
        .triggerSelect      = 1,
        .frameTriggerDelay  = 0
    },
    .profile    = {
        .profileId             = 0,
        .startFreqConst        = (uint32_t) (77.0 * (1U << 26) / 3.6),
        .idleTimeConst         = 7 * 1000 / 10,       /* 7µs */
        .adcStartTimeConst     = 7 * 1000 / 10,       /* 7µs */
        .rampEndTime           = 2083U,                /* ~20.83µs */
        .txOutPowerBackoffCode = 0,
        .txPhaseShifter        = 0,
        .freqSlopeConst        = 11 * 1000 / 48,       /* ~11 MHz/µs */
        .txStartTime           = 0,
        .numAdcSamples         = 256,
        .digOutSampleRate      = 20000,                /* 20 Msps */
        .hpfCornerFreq1        = 0,
        .hpfCornerFreq2        = 0,
        .rxGain                = 42
    },
    .startupDiscardFrames = 0
};

CLI_CaptureConfig gActive __attribute__((used)) = { 0 };

/**************************************************************************
 ********************* Chirp Header Ping/Pong Buffers *********************
 **************************************************************************/
ChirpHeader_t gChirpHdrPing __attribute__((aligned(16)));
ChirpHeader_t gChirpHdrPong __attribute__((aligned(16)));
volatile uint16_t gChirpHdrFrameIdx   = 0U;
volatile uint16_t gChirpHdrChirpIdx   = 0U;
volatile uint32_t gChirpHdrGlobalIdx  = 0U;
uint32_t          gChirpHdrChirpsPerFrame = 64U;

/* CLI task handle — stored globally so initTask can delete it before teardown */
static TaskHandle_t        gCliTaskHandle = NULL;
static StaticTask_t        gCliTaskObj;
static StackType_t         gCliTaskStack[4096U / sizeof(StackType_t)];
#define CLI_TASK_PRI       (4U)   /* Below APP_TASK_PRI so it doesn't starve init */
#define CLI_CMD_BUF_SIZE   (128U) /* profileCfg with 14 args can exceed 64 bytes */

extern volatile Bool gCbuffFirstEOL;
extern volatile Bool gPingPongSwitchFlag;
void configureTransfer(void);
static void populateChirpHeader(void);

/**************************************************************************
 ************************** CLI Helper: UART Write ************************
 **************************************************************************/
static UART_Handle gCliUartHandle = NULL;

static void CLI_uartWrite(const char* msg)
{
    if (gCliUartHandle == NULL) return;
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf   = (void*)msg;
    trans.count = strlen(msg);
    UART_write(gCliUartHandle, &trans);
}

/**************************************************************************
 ************************** CLI: snprintf helper **************************
 **************************************************************************/
static void CLI_uartPrintf(const char* fmt, ...)
{
    /* Stack-local buffer avoids reentrancy issues if multiple tasks ever
     * call this function.  512 bytes fits comfortably on the CLI task's
     * 4 KiB stack (only ~12.5% usage). */
    char fmtBuf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(fmtBuf, sizeof(fmtBuf), fmt, ap);
    va_end(ap);
    CLI_uartWrite(fmtBuf);
}

/**************************************************************************
 ************************** CLI: State Name Helper ************************
 **************************************************************************/
static const char* CLI_stateName(CLI_State s)
{
    switch (s)
    {
        case CLI_STATE_INIT:                return "INIT";
        case CLI_STATE_WAITING_FOR_COMMAND: return "WAITING_FOR_COMMAND";
        case CLI_STATE_START_REQUESTED:     return "START_REQUESTED";
        case CLI_STATE_RUNNING_CAPTURE:     return "RUNNING_CAPTURE";
        case CLI_STATE_CAPTURE_DONE:        return "CAPTURE_DONE";
        case CLI_STATE_ERROR:               return "ERROR";
        default:                            return "UNKNOWN";
    }
}

/**************************************************************************
 ************************** CLI: Config Validation *************************
 **************************************************************************/
static uint8_t countBits(uint32_t mask) {
    uint8_t count = 0;
    while(mask) { count += mask & 1; mask >>= 1; }
    return count;
}

static uint32_t CLI_getChirpsPerFrame(const CLI_CaptureConfig* cfg)
{
    uint8_t ntx = countBits(cfg->txMask);
    uint32_t nTdmChirps = cfg->tdmEnabled ? ntx : 1U;

    if (nTdmChirps == 0U)
    {
        nTdmChirps = 1U;
    }
    return nTdmChirps * cfg->frame.numLoops;
}

uint32_t CLI_getBytesPerChirp(const CLI_CaptureConfig* cfg)
{
    /* frame.numAdcSamples is in "half-words" (16-bit units) as per rlFrameCfg_t spec:
     *   Real format:    numAdcSamples = profile.numAdcSamples
     *   Complex format: numAdcSamples = profile.numAdcSamples * 2
     * Each half-word is 2 bytes, so total bytes = numAdcSamples * 2.
     *
     * The ping-pong buffers process one chirp at a time per chip.
     * Each chip has a MAXIMUM of 4 RX antennas.
     */
    return (uint32_t)(cfg->frame.numAdcSamples * 2U * 4U);
}

static bool CLI_validateConfigSize(CLI_CaptureConfig* testCfg)
{
    uint32_t bytes_per_chirp = CLI_getBytesPerChirp(testCfg);

    /* We compare against BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED 
     * which MUST be sized to accommodate a single chirp from 4 RX antennas. */
    if (bytes_per_chirp > BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED)
    {
        CLI_uartPrintf("{\"error\":\"memory limit exceeded\",\"req_bytes\":%u,\"max_bytes\":%u}\r\n",
                       bytes_per_chirp, (unsigned)BOARD_DIAG_PING_OR_PONG_BUF_SIZE_ALIGNED);
        return false;
    }
    return true;
}

/**************************************************************************
 ************************** CLI: Command Handler ***************************
 **************************************************************************/
static void CLI_handleCommand(char* cmdLine)
{
    /* Tokenize: split into command and optional argument.
     * strtok is safe here because CLI parsing runs in a single task. */
    char* cmdToken = NULL;
    char* argToken = NULL;

    cmdToken = strtok(cmdLine, " \t");
    if (cmdToken == NULL)
    {
        return; /* empty line */
    }
    argToken = strtok(NULL, " \t");

    /* ---- status ---- */
    if (strcmp(cmdToken, "status") == 0)
    {
        uint8_t ntx = countBits(gCfg.txMask);
        uint8_t nrx = countBits(gCfg.rxMask);
        uint32_t n_tdm_chirps = gCfg.tdmEnabled ? ntx : 1;
        if (n_tdm_chirps == 0) n_tdm_chirps = 1;

        uint32_t chirps_per_frame = n_tdm_chirps * gCfg.frame.numLoops;
        uint32_t logical_sweeps = gCfg.frame.numLoops;
        
        /* frame.numAdcSamples is in half-words (already doubled for complex).
         * Each half-word = 2 bytes. */
        uint32_t bytes_per_frame = gCfg.frame.numAdcSamples * 2U * nrx * chirps_per_frame;

        CLI_uartPrintf(
            "{\"state\":\"%s\","
            "\"cfg\":{\"tdm\":%u,\"masks\":{\"rx\":\"0x%02X\",\"tx\":\"0x%02X\"},"
            "\"frame\":{\"start\":%u,\"end\":%u,\"loops\":%u,\"frames\":%u,\"period\":%u,\"trig\":%u,\"delay\":%u}},"
            "\"derived\":{\"ntx_active\":%u,\"nrx_active\":%u,\"n_tdm_chirps\":%u,"
            "\"chirps_per_frame\":%u,\"logical_sweeps_per_frame\":%u,\"bytes_per_frame\":%u,\"total_sweeps\":%u},"
            "\"err\":0}\r\n",
            CLI_stateName(gCliState),
            gCfg.tdmEnabled, gCfg.rxMask, gCfg.txMask,
            gCfg.frame.chirpStartIdx, gCfg.frame.chirpEndIdx, gCfg.frame.numLoops, gCfg.frame.numFrames,
            gCfg.frame.framePeriodicity, gCfg.frame.triggerSelect, gCfg.frame.frameTriggerDelay,
            ntx, nrx, n_tdm_chirps,
            chirps_per_frame, logical_sweeps, bytes_per_frame, (logical_sweeps * gCfg.frame.numFrames)
        );
        return;
    }

    /* ---- sensorStart ---- */
    if (strcmp(cmdToken, "sensorStart") == 0)
    {
        if (gCliState == CLI_STATE_WAITING_FOR_COMMAND)
        {
            /* Check if memory sizes are valid before allowing start */
            if (!CLI_validateConfigSize(&gCfg))
            {
                /* validateConfigSize already prints the JSON error */
                return;
            }

            /* Finite: startupDiscard must be < user M. Infinite (numFrames==0): no inflate. */
            if (gCfg.frame.numFrames > 0U)
            {
                if (gCfg.startupDiscardFrames >= gCfg.frame.numFrames)
                {
                    CLI_uartPrintf("{\"error\":\"startupDiscardFrames (%u) must be < numFrames (%u)\"}\r\n",
                                   gCfg.startupDiscardFrames, gCfg.frame.numFrames);
                    return;
                }
            }

            /* Latch the configured state */
            gActive = gCfg;

            /* Finite only: inflate to M+N internal RF frames. Infinite: leave numFrames 0. */
            if (gActive.frame.numFrames > 0U && gActive.startupDiscardFrames > 0U)
            {
                gActive.frame.numFrames += gActive.startupDiscardFrames;
            }
            gCliState = CLI_STATE_START_REQUESTED;

            /* Calculate derived geometry to print */
            uint8_t ntx = countBits(gActive.txMask);
            uint8_t nrx = countBits(gActive.rxMask);
            uint32_t n_tdm_chirps = gActive.tdmEnabled ? ntx : 1;
            if (n_tdm_chirps == 0) n_tdm_chirps = 1;
            uint32_t chirps_per_frame = n_tdm_chirps * gActive.frame.numLoops;
            uint32_t bytes_per_chirp = CLI_getBytesPerChirp(&gActive);
            uint32_t bytes_per_frame = bytes_per_chirp * chirps_per_frame;
            
            /* Independent check (Consistency computations) */
            uint32_t bytes_per_sample_expected = (gActive.adcFmt == 1 || gActive.adcFmt == 2) ? 4 : 2; /* 4 for Complex I/Q, 2 for Real */
            uint32_t expected_bytes_per_chirp_per_chip = gActive.profile.numAdcSamples * 4 * bytes_per_sample_expected; /* Assumes 4 RX per chip! */
            uint32_t expected_bytes_per_frame_per_chip = expected_bytes_per_chirp_per_chip * chirps_per_frame;

            test_print("==================================\n");
            test_print("   Capture Geometry (Latched)\n");
            test_print("==================================\n");
            test_print(" ADC Samples (profile) : %u\n", gActive.profile.numAdcSamples);
            test_print(" ADC Samples (frame)   : %u (half-words)\n", gActive.frame.numAdcSamples);
            test_print(" RX Mask / Active      : 0x%X / %u\n", gActive.rxMask, nrx);
            test_print(" TX Mask / Active      : 0x%X / %u\n", gActive.txMask, ntx);
            test_print(" TDM Enabled           : %d\n", gActive.tdmEnabled);
            test_print(" ADC Bits              : %d\n", gActive.adcBits);
            test_print(" ADC Format            : %d (%s)\n", gActive.adcFmt, bytes_per_sample_expected == 4 ? "Complex" : "Real");
            test_print(" Chirps per Frame      : %u\n", chirps_per_frame);
            test_print(" Bytes/Chirp (1 Chip)  : %u (Computed vs %u Expected)\n", bytes_per_chirp, expected_bytes_per_chirp_per_chip);
            test_print(" Bytes/Frame (1 Chip)  : %u (Computed vs %u Expected)\n", bytes_per_frame, expected_bytes_per_frame_per_chip);
            test_print("==================================\n");

            if (bytes_per_chirp != expected_bytes_per_chirp_per_chip) {
                test_print("[GEOMETRY WARNING] Computed bytes/chirp (%u) does NOT match expected bytes/chirp (%u)\n", 
                    bytes_per_chirp, expected_bytes_per_chirp_per_chip);
            }

            if (gActive.frame.numFrames > 0U)
            {
                CLI_uartPrintf(
                    "{\"status\":\"ok\",\"cmd\":\"sensorStart\",\"infinite\":0,\"validFrames\":%u,"
                    "\"startupDiscard\":%u,\"internalFrames\":%u}\r\n",
                    (unsigned)(gActive.frame.numFrames - gActive.startupDiscardFrames),
                    (unsigned)gActive.startupDiscardFrames,
                    (unsigned)gActive.frame.numFrames);
                DebugP_log("CLI: sensorStart finite valid=%u discard=%u internal=%u\n",
                           (unsigned)(gActive.frame.numFrames - gActive.startupDiscardFrames),
                           (unsigned)gActive.startupDiscardFrames,
                           (unsigned)gActive.frame.numFrames);
            }
            else
            {
                CLI_uartPrintf(
                    "{\"status\":\"ok\",\"cmd\":\"sensorStart\",\"infinite\":1,\"startupDiscard\":%u}\r\n",
                    (unsigned)gActive.startupDiscardFrames);
                DebugP_log("CLI: sensorStart infinite rf, startupDiscard=%u\n",
                           (unsigned)gActive.startupDiscardFrames);
            }
        }
        else
        {
            CLI_uartPrintf(
                "{\"error\":\"busy or already completed\",\"state\":\"%s\"}\r\n",
                CLI_stateName(gCliState));
        }
        return;
    }

    /* ---- setFrames <N> ---- */
    if (strcmp(cmdToken, "setFrames") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n",
                           CLI_stateName(gCliState));
            return;
        }
        if (argToken == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires a numeric argument\"}\r\n");
            return;
        }

        char* endPtr = NULL;
        long val = strtol(argToken, &endPtr, 10);
        if ((endPtr == argToken) || (*endPtr != '\0') ||
            (val < 1) || (val > 65535))
        {
            CLI_uartWrite("{\"error\":\"invalid or out-of-range value [1..65535]\"}\r\n");
            return;
        }

        gCfg.frame.numFrames = (uint32_t)val;
        CLI_uartPrintf("{\"status\":\"ok\",\"frames\":%u}\r\n",
                       (unsigned)gCfg.frame.numFrames);
        return;
    }

    /* ---- channelCfg <rxMask> <txMask> ---- */
    if (strcmp(cmdToken, "channelCfg") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }

        char* argRx = argToken;
        char* argTx = strtok(NULL, " \t");

        if (argRx == NULL || argTx == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires rxMask and txMask arguments\"}\r\n");
            return;
        }

        char* endRx = NULL;
        char* endTx = NULL;
        long valRx = strtol(argRx, &endRx, 16);
        long valTx = strtol(argTx, &endTx, 16);
        
        if ((endRx == argRx) || (*endRx != '\0') || (valRx < 1) || (valRx > 0xFF))
        {
            CLI_uartWrite("{\"error\":\"invalid rxMask [1..0xFF]\"}\r\n");
            return;
        }
        if ((endTx == argTx) || (*endTx != '\0') || (valTx < 1) || (valTx > 0x3F))
        {
            CLI_uartWrite("{\"error\":\"invalid txMask [1..0x3F]\"}\r\n");
            return;
        }

        CLI_CaptureConfig testCfg = gCfg;
        testCfg.rxMask = (uint8_t)valRx;
        testCfg.txMask = (uint8_t)valTx;
        
        if (!CLI_validateConfigSize(&testCfg)) return;

        gCfg = testCfg;
        CLI_uartPrintf("{\"status\":\"ok\",\"rxMask\":\"0x%02X\",\"txMask\":\"0x%02X\"}\r\n",
                       gCfg.rxMask, gCfg.txMask);
        return;
    }

    /* ---- setTDM <0|1> ---- */
    if (strcmp(cmdToken, "setTDM") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }

        if (argToken == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires 0 or 1 argument\"}\r\n");
            return;
        }

        CLI_CaptureConfig testCfg = gCfg;
        
        if (strcmp(argToken, "0") == 0)
        {
            testCfg.tdmEnabled = 0;
            if (!CLI_validateConfigSize(&testCfg)) return;
            gCfg = testCfg;
            CLI_uartWrite("{\"status\":\"ok\",\"tdm\":0}\r\n");
        }
        else if (strcmp(argToken, "1") == 0)
        {
            testCfg.tdmEnabled = 1;
            if (!CLI_validateConfigSize(&testCfg)) return;
            gCfg = testCfg;
            CLI_uartWrite("{\"status\":\"ok\",\"tdm\":1}\r\n");
        }
        else
        {
            CLI_uartWrite("{\"error\":\"invalid argument, must be 0 or 1\"}\r\n");
        }
        return;
    }

    /* ---- frameCfg <startIdx> <endIdx> <loops> <frames> <periodicity> <triggerSel> <delay> ---- */
    if (strcmp(cmdToken, "frameCfg") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }

        float args[7] = {0};
        int argc = 0;
        char* token = argToken;

        while (token != NULL && argc < 7)
        {
            char* endPtr = NULL;
            float val = strtof(token, &endPtr);
            if (endPtr == token) {
                CLI_uartPrintf("{\"error\":\"invalid argument at position %d\"}\r\n", argc);
                return;
            }
            args[argc++] = val;
            token = strtok(NULL, " \t");
        }

        if (argc != 7)
        {
            CLI_uartWrite("{\"error\":\"frameCfg requires 7 arguments\"}\r\n");
            return;
        }

        if (args[0] > args[1] || args[2] == 0 || args[3] == 0)
        {
            CLI_uartWrite("{\"error\":\"invalid frame bounds or zero loops/frames\"}\r\n");
            return;
        }

        CLI_CaptureConfig testCfg = gCfg;
        testCfg.frame.chirpStartIdx     = (uint16_t)args[0];
        testCfg.frame.chirpEndIdx       = (uint16_t)args[1];
        testCfg.frame.numLoops          = (uint16_t)args[2];
        testCfg.frame.numFrames         = (uint16_t)args[3];
        testCfg.frame.framePeriodicity  = (uint32_t)(args[4] * 1000000.0f / 5.0f);
        testCfg.frame.triggerSelect     = (uint16_t)args[5];
        testCfg.frame.frameTriggerDelay = (uint32_t)(args[6] * 1000000.0f / 5.0f);

        if (!CLI_validateConfigSize(&testCfg)) return;
        
        gCfg = testCfg;
        CLI_uartWrite("{\"status\":\"ok\",\"cmd\":\"frameCfg\"}\r\n");
        return;
    }

    /* ---- profileCfg <profileId> <startFreq> <idleTime> <adcStartTime> <rampEndTime> <txOutPower> <txPhaseShift> <freqSlope> <txStartTime> <numAdcSamples> <digOutSampleRate> <hpfCornerFreq1> <hpfCornerFreq2> <rxGain> ---- */
    if (strcmp(cmdToken, "profileCfg") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }

        float args[14] = {0};
        int argc = 0;
        char* token = argToken;

        while (token != NULL && argc < 14)
        {
            char* endPtr = NULL;
            float val = strtof(token, &endPtr);
            if (endPtr == token) {
                CLI_uartPrintf("{\"error\":\"invalid argument at position %d\"}\r\n", argc);
                return;
            }
            args[argc++] = val;
            token = strtok(NULL, " \t");
        }

        if (argc != 14)
        {
            CLI_uartWrite("{\"error\":\"profileCfg requires 14 arguments\"}\r\n");
            return;
        }

        CLI_CaptureConfig testCfg = gCfg;
        
        testCfg.profile.profileId             = (uint16_t)args[0];
        testCfg.profile.startFreqConst        = (uint32_t)(args[1] * (1U << 26) / 3.6f);
        testCfg.profile.idleTimeConst         = (uint32_t)(args[2] * 100.0f);
        testCfg.profile.adcStartTimeConst     = (uint32_t)(args[3] * 100.0f);
        testCfg.profile.rampEndTime           = (uint32_t)(args[4] * 100.0f);
        testCfg.profile.txOutPowerBackoffCode = (uint32_t)args[5];
        testCfg.profile.txPhaseShifter        = (uint32_t)args[6];
        testCfg.profile.freqSlopeConst        = (int16_t)(args[7] * 20.7126f);
        testCfg.profile.txStartTime           = (int16_t)(args[8] * 100.0f);
        testCfg.profile.numAdcSamples         = (uint16_t)args[9];
        testCfg.profile.digOutSampleRate      = (uint16_t)args[10];
        testCfg.profile.hpfCornerFreq1        = (uint8_t)args[11];
        testCfg.profile.hpfCornerFreq2        = (uint8_t)args[12];
        testCfg.profile.rxGain                = (uint16_t)args[13];
        
        /* rlFrameCfg_t.numAdcSamples must be doubled for complex ADC format.
         * Real/PseudoReal: numAdcSamples = profile.numAdcSamples
         * Complex1x/2x:    numAdcSamples = profile.numAdcSamples * 2
         */
        testCfg.frame.numAdcSamples = (testCfg.adcFmt == 1 || testCfg.adcFmt == 2)
            ? testCfg.profile.numAdcSamples * 2
            : testCfg.profile.numAdcSamples;

        if (!CLI_validateConfigSize(&testCfg)) return;

        gCfg = testCfg;
        CLI_uartWrite("{\"status\":\"ok\",\"cmd\":\"profileCfg\"}\r\n");
        return;
    }

    /* ---- adcCfg <bits> <fmt> ---- */
    if (strcmp(cmdToken, "adcCfg") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }

        char* argBits = argToken;
        char* argFmt  = strtok(NULL, " \t");

        if (argBits == NULL || argFmt == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires bits and fmt arguments\"}\r\n");
            return;
        }

        char* endBits = NULL;
        char* endFmt = NULL;
        long valBits = strtol(argBits, &endBits, 10);
        long valFmt = strtol(argFmt, &endFmt, 10);
        
        if ((endBits == argBits) || (*endBits != '\0') || valBits < 0 || valBits > 2)
        {
            CLI_uartWrite("{\"error\":\"invalid bits [0=12, 1=14, 2=16]\"}\r\n");
            return;
        }
        if ((endFmt == argFmt) || (*endFmt != '\0') || valFmt < 0 || valFmt > 2)
        {
            CLI_uartWrite("{\"error\":\"invalid fmt [0=real, 1=cpx_1x, 2=cpx_2x]\"}\r\n");
            return;
        }

        CLI_CaptureConfig testCfg = gCfg;
        testCfg.adcBits = (uint8_t)valBits;
        testCfg.adcFmt  = (uint8_t)valFmt;

        /* Recalculate frame.numAdcSamples for the new ADC format */
        testCfg.frame.numAdcSamples = (testCfg.adcFmt == 1 || testCfg.adcFmt == 2)
            ? testCfg.profile.numAdcSamples * 2
            : testCfg.profile.numAdcSamples;
        
        if (!CLI_validateConfigSize(&testCfg)) return;

        gCfg = testCfg;
        CLI_uartPrintf("{\"status\":\"ok\",\"adcBits\":%u,\"adcFmt\":%u}\r\n",
                       gCfg.adcBits, gCfg.adcFmt);
        return;
    }

    /* ---- setChirpHeader <0|1> ---- */
    if (strcmp(cmdToken, "setChirpHeader") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }
        if (argToken == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires 0 or 1 argument\"}\r\n");
            return;
        }
        if (strcmp(argToken, "0") == 0)
        {
            gCfg.chirpHeaderEnabled = 0;
            CLI_uartWrite("{\"status\":\"ok\",\"chirpHeader\":0}\r\n");
        }
        else if (strcmp(argToken, "1") == 0)
        {
            gCfg.chirpHeaderEnabled = 1;
            CLI_uartWrite("{\"status\":\"ok\",\"chirpHeader\":1}\r\n");
        }
        else
        {
            CLI_uartWrite("{\"error\":\"invalid argument, must be 0 or 1\"}\r\n");
        }
        return;
    }

    /* ---- setStartupDiscard <N> ---- */
    if (strcmp(cmdToken, "setStartupDiscard") == 0)
    {
        if (gCliState != CLI_STATE_WAITING_FOR_COMMAND)
        {
            CLI_uartPrintf("{\"error\":\"busy\",\"state\":\"%s\"}\r\n", CLI_stateName(gCliState));
            return;
        }
        if (argToken == NULL)
        {
            CLI_uartWrite("{\"error\":\"requires number of startup frames to discard (0=disabled)\"}\r\n");
            return;
        }
        char* endPtr = NULL;
        long val = strtol(argToken, &endPtr, 10);
        if (endPtr == argToken || *endPtr != '\0' || val < 0 || val > 50)
        {
            CLI_uartWrite("{\"error\":\"invalid value, range 0..50\"}\r\n");
            return;
        }
        gCfg.startupDiscardFrames = (uint16_t)val;
        CLI_uartPrintf("{\"status\":\"ok\",\"startupDiscardFrames\":%u}\r\n",
                       gCfg.startupDiscardFrames);
        return;
    }

    /* ---- unknown ---- */
    CLI_uartPrintf("{\"error\":\"Unknown command\",\"cmd\":\"%s\"}\r\n",
                   cmdToken);
}

/**************************************************************************
 ************************** CLI: FreeRTOS Task *****************************
 **************************************************************************/
static void CLI_task(void* args)
{
    char            cmdBuf[CLI_CMD_BUF_SIZE];
    uint32_t        idx = 0U;
    char            rxByte;
    UART_Transaction trans;

    CLI_uartWrite("\r\nCLI ready. Waiting for commands.\r\n");
    DebugP_log("CLI: task started, waiting for commands\n");

    memset(cmdBuf, 0, sizeof(cmdBuf));

    for (;;)
    {
        UART_Transaction_init(&trans);
        trans.buf   = (void*)&rxByte;
        trans.count = 1U;

        int32_t status = UART_read(gCliUartHandle, &trans);
        if ((status != SystemP_SUCCESS) || (trans.count == 0U))
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Ignore \r, process on \n */
        if (rxByte == '\r')
            continue;

        if (rxByte == '\n')
        {
            cmdBuf[idx] = '\0';

            /* Trim leading spaces */
            char *pCmd = cmdBuf;
            while (*pCmd == ' ') pCmd++;

            /* Trim trailing spaces */
            {
                int32_t end = (int32_t)strlen(pCmd) - 1;
                while (end >= 0 && pCmd[end] == ' ')
                {
                    pCmd[end] = '\0';
                    end--;
                }
            }

            if (strlen(pCmd) > 0U)
            {
                CLI_handleCommand(pCmd);
            }

            idx = 0U;
            memset(cmdBuf, 0, sizeof(cmdBuf));
        }
        else
        {
            if (idx < (sizeof(cmdBuf) - 1U))
            {
                cmdBuf[idx++] = rxByte;
            }
            else
            {
                CLI_uartWrite("{\"error\":\"command too long\"}\r\n");
                idx = 0U;
                memset(cmdBuf, 0, sizeof(cmdBuf));
            }
        }
    }
}

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern void Mmwave_populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg);
extern void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern void Mmwave_populateDefaultAdvancedControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern void Mmwave_populateDefaultCalibrationCfg (MMWave_CalibrationCfg* ptrCalibrationCfg, MMWave_DFEDataOutputMode dfeOutputMode);
extern int32_t Mmwave_eventFxn (uint8_t devIndex,uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);
extern void Mmwave_ctrlTask(void* args);

/* CSI RX */;
extern void MmwCascade_csirxInit(MmwCascade_MCB  *CascadeMCB);
extern void MmwCascade_csirxOpen(MmwCascade_MCB  *CascadeMCB, int32_t *errCode);
static void MmwCascade_CsiConfigTask(void* args);

/**************************************************************************
 ********** Phase 1F: Config Snapshot + Phase 1B2: Sanity Review **********
 **************************************************************************/
static void printConfigSnapshot(void)
{
    uint32_t chirpsPerFrame = CLI_getChirpsPerFrame(&gActive);
    uint32_t bytesPerChirp  = CLI_getBytesPerChirp(&gActive);
    uint32_t bytesPerSample = (gActive.adcFmt == 1 || gActive.adcFmt == 2) ? 4U : 2U;

    test_print("[CONFIG] chirpsPerFrame=%u samplesPerChirp=%u adcBits=%u adcFmt=%u\n",
               chirpsPerFrame, gActive.profile.numAdcSamples,
               gActive.adcBits, gActive.adcFmt);
    test_print("[CONFIG] rxMask=0x%02X txMask=0x%02X tdm=%u laneCount=4 laneRate=600Mbps\n",
               gActive.rxMask, gActive.txMask, gActive.tdmEnabled);
    test_print("[CONFIG] idleTime=%u rampEnd=%u adcStart=%u (10ns units)\n",
               gActive.profile.idleTimeConst,
               gActive.profile.rampEndTime,
               gActive.profile.adcStartTimeConst);
    test_print("[CONFIG] isPowerAuto=%u isOcpAutoIdle=%u\n",
               gCsirxComplexioConfig[0].isPowerAuto ? 1U : 0U,
               gCsirxCommonConfig[0].isOcpAutoIdle ? 1U : 0U);
    test_print("[CONFIG] stopStateFsmTimeout=%u bytesPerChirp=%u bytesPerSample=%u\n",
               gCsirxCommonConfig[0].stopStateFsmTimeoutInNanoSecs,
               bytesPerChirp, bytesPerSample);
    test_print("[CONFIG] framePeriod=%u frameTrigDelay=%u numFrames=%u\n",
               gActive.frame.framePeriodicity,
               gActive.frame.frameTriggerDelay,
               gActive.frame.numFrames);
    if (gActive.frame.numFrames > 0U)
    {
        uint32_t validReq    = gActive.frame.numFrames - gActive.startupDiscardFrames;
        uint32_t internalTgt = gActive.frame.numFrames;
        test_print("[CONFIG] rfMode=finite startupDiscardFrames=%u validFramesRequested=%u internalFramesTarget=%u\n",
                   gActive.startupDiscardFrames, validReq, internalTgt);
    }
    else
    {
        test_print("[CONFIG] rfMode=infinite startupDiscardFrames=%u (geom=256-slot ring)\n",
                   gActive.startupDiscardFrames);
    }
    test_print("[CONFIG] chirpIdSource=SW_COUNTER: per-frame EOL count + timing only; "
               "NOT missing chirp index within a frame (no HW line ID in ADC-only path)\n");
}

static void printConfigSanityReview(void)
{
    uint32_t bytesPerSample = (gActive.adcFmt == 1 || gActive.adcFmt == 2) ? 4U : 2U;
    uint32_t expectedBytesPerLine = gActive.profile.numAdcSamples * bytesPerSample * TEST_NUM_RX;
    uint32_t computedBytesPerChirp = CLI_getBytesPerChirp(&gActive);

    test_print("[SANITY] CSIRX0: format=RAW8 VC=%u userDefMap=RAW8 lineOffset=%u ppSwitch=LINE numLinesSw=1\n",
               gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].virtualChannelId,
               gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.lineOffset);
    test_print("[SANITY] CSIRX1: format=RAW8 VC=%u userDefMap=RAW8 lineOffset=%u ppSwitch=LINE numLinesSw=1\n",
               gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].virtualChannelId,
               gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].pingPongConfig.lineOffset);
    test_print("[SANITY] lanes: pos=1,2,4,5 clk=3 pol=PLUS_MINUS (both ports)\n");
    test_print("[SANITY] payloadBytesPerLine: expected=%u computed=%u %s\n",
               expectedBytesPerLine, computedBytesPerChirp,
               (expectedBytesPerLine == computedBytesPerChirp) ? "OK" : "MISMATCH");

    if (expectedBytesPerLine != computedBytesPerChirp)
        test_print("[SANITY WARNING] payload size mismatch!\n");

    if (gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].virtualChannelId != 0)
        test_print("[SANITY WARNING] CSIRX0 VC != 0\n");
    if (gConfigCsirx1ContextConfig[MMW_CASCADE_CSI2_CONTEXT].virtualChannelId != 0)
        test_print("[SANITY WARNING] CSIRX1 VC != 0\n");
    if (gConfigCsirx0ContextConfig[MMW_CASCADE_CSI2_CONTEXT].enableIntr.isPayloadChecksumMismatch == false)
        test_print("[SANITY WARNING] CSIRX0 payload checksum mismatch IRQ disabled - mismatches would be silent\n");

    test_print("[SANITY] cqConfig=0x%02X transferFmtPkt0=0x%02X (ADC_DATA_ONLY expected: 0x01)\n",
               0x00U, 0x01U);
    test_print("[SANITY] review complete\n");
}
/**************************************************************************
 *********************** mmWave Unit Test Functions ***********************
 **************************************************************************/
static uint8_t readPmicReg(MIBSPI_Handle handle, uint8_t regOffset)
{
    uint8_t txBuffer[PMIC_MSGSIZE];
    uint8_t rxBuffer[PMIC_MSGSIZE];
    uint8_t regValue = 0U;
    MIBSPI_Transaction spiTransaction;

    /* Configure Data Transfer */
    spiTransaction.count = PMIC_MSGSIZE-1;
    spiTransaction.txBuf = txBuffer;
    spiTransaction.rxBuf = rxBuffer;
    spiTransaction.peripheralIndex = 0;
    txBuffer[0] = regOffset;
    // Indicate PMIC a read sequence */
    txBuffer[1] = 0x10;
    txBuffer[2] = 0;

    CacheP_wbInv((void *)&txBuffer[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    /* Start Data Transfer */
    MIBSPI_transfer(handle, &spiTransaction);

    CacheP_inv((void *)&rxBuffer[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    /*PMIC GPIO Out register value */
    regValue = rxBuffer[2];

    return regValue;
}

volatile uint8_t pmicRegRead = 0;
/**
 *  @b Description
 *  @n
 *      Configures PMIC BUCK4
 *
 *  @retval
 *      Not Applicable.
 */
void Enable_BUCK4_ViaPMIC(void)
{
    MIBSPI_Transaction stTransaction = {0U};
    uint8_t u8TxBuff[4] = {0U};
    int32_t transferOK;

    /* Now, configure the PMIC */
    stTransaction.peripheralIndex = 0U;
    stTransaction.rxBuf      = NULL;
    stTransaction.txBuf      = (void *)&u8TxBuff[0];
    stTransaction.count      = PMIC_MSGSIZE - 1U;

    u8TxBuff[0] = PMIC_CONFIG_BUCK4_REG_ADDR;    /* Offset */
    u8TxBuff[1] = 0x00U;    /* Page number + Write access */
    u8TxBuff[2] = 0x33U;    /*  BUCK 4 configuration*/

    /* It is important to invalidate the cache because the SPI driver will use eDMA transfer
    *   between the memory and the internal SPI RAM buffer. */
    CacheP_wbInv((void *)&u8TxBuff[0], PMIC_MSGSIZE, CacheP_TYPE_ALLD);

    transferOK = MIBSPI_transfer(gMmwCascadeMCB.pmicMIBSPIhandle, &stTransaction);

    ClockP_sleep(1);

    if((SystemP_SUCCESS != transferOK) ||
        (MIBSPI_TRANSFER_COMPLETED != stTransaction.status))
    {
        DebugP_assert(FALSE); /* MIBSPI transfer failed!! */
    }

    /* Read back and verify BUCK4 configuration. */
    pmicRegRead = readPmicReg(gMmwCascadeMCB.pmicMIBSPIhandle, PMIC_CONFIG_BUCK4_REG_ADDR);

    if(pmicRegRead == u8TxBuff[2])
    {
        test_print ("PMIC register 0x0%X configured to 0x%X.\n", PMIC_CONFIG_BUCK4_REG_ADDR, pmicRegRead);
    }
    else
    {
        test_print ("PMIC register config failed.\n");
    }

    return;
}


/**
 *  @b Description
 *  @n
 *      Test implementation
 *
 *  @retval
 *      Not Applicable.
 */
void MmwCascade_mmWaveTest (void)
{
    MMWave_InitCfg          initCfg;
    MMWave_CtrlCfg          ctrlCfg;
    MMWave_OpenCfg          openCfg;
    int32_t                 errCode;
    int32_t                 retVal;
    MMWave_CalibrationCfg   calibrationCfg;
    MMWave_ErrorLevel       errorLevel;
    int16_t                 mmWaveErrorCode;
    int16_t                 subsysErrorCode;
    uint32_t                u32DevIdx;

    /* Initialize the configuration: */
    memset ((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.eventFxn                = Mmwave_eventFxn;
    initCfg.linkCRCCfg.crcBaseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_CHANNEL_1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;

    /* Initialize and setup the mmWave Control module */
    gMmwCascadeMCB.mmWaveHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwCascadeMCB.mmWaveHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        test_print ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Log into the MCPI Test Logger: */
        MCPI_setFeatureTestResult ("MMWave MSS Initialization", MCPI_TestResult_FAIL);
        return;
    }
    
    test_print ("MMWave MSS Initialization\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    /* Launch the CSIRX Task */
    gMmwCtrlTask = xTaskCreateStatic( Mmwave_ctrlTask,   /* Pointer to the function that implements the task. */
                                "test_mmw_ctrl_task", /* Text name for the task.  This is to facilitate debugging only. */
                                MMW_CTRL_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,              /* We are not using the task parameter. */
                                MMW_CTRL_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gMmwCtrlTskStack,  /* pointer to stack base */
                                &gMmwCtrlTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gMmwCtrlTask != NULL);

    Mmwave_populateDefaultOpenCfg (&openCfg);
    Mmwave_populateDefaultChirpControlCfg (&ctrlCfg);
    /* Mmwave_populateDefaultOpenCfg is memsetting the openCfg to zero, that is why
       had to save the spi handle in local var and populate it here*/

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(u32DevIdx == 0)
        {
            /* Master AWR2243 */
            openCfg.frontEndCfg[u32DevIdx].spiHandle = gMibspiHandle[CONFIG_MIBSPI0];
            openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(NRESET_FE1_BASE_ADDR);

            /* Inform mmwave which GPIO pin is used for the front end NRESET*/
            openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex = (uint32_t) NRESET_FE1_PIN;

            /* Inform mmwave which GPIO pin is used for the SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex = (uint32_t) RCSS_MIBSPIA_HOST_IRQ_PIN;

            /* Inform mmwave which interrupt to be configured for SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].gpioPinIntrNum = (uint32_t) RCSS_MIBSPIA_HOST_IRQ_INTR_HIGH;
        }
        else
        {
            /* Slave#1 AWR2243 */
            openCfg.frontEndCfg[u32DevIdx].spiHandle = gMibspiHandle[CONFIG_MIBSPI1];
            openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(NRESET_FE2_BASE_ADDR);

            /* Inform mmwave which GPIO pin is used for the front end NRESET*/
            openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex = (uint32_t) NRESET_FE2_PIN;

            /* Inform mmwave which GPIO pin is used for the SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex = (uint32_t) RCSS_MIBSPIB_HOST_IRQ_PIN;

            /* Inform mmwave which interrupt to be configured for SPI IRQ*/
            openCfg.frontEndCfg[u32DevIdx].gpioPinIntrNum = (uint32_t) RCSS_MIBSPIB_HOST_IRQ_INTR_HIGH;
        }
    }

    openCfg.iqSwapSel = 0;
    openCfg.chInterleave = 1;

    /************************************************************************
     * Open the mmWave:
     ************************************************************************/
    if (MMWave_open (gMmwCascadeMCB.mmWaveHandle, &openCfg, NULL, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave open failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Open", MCPI_TestResult_FAIL);
        return;
    }
   
    test_print ("MMWave MSS Open done.\n");

    /************************************************************************
     * Configure the mmWave:
     ************************************************************************/
    if (MMWave_config (gMmwCascadeMCB.mmWaveHandle, &ctrlCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        test_print ("Error: mmWave configuration failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Configuration", MCPI_TestResult_FAIL);
        return;
    }
    
    test_print ("MMWave MSS Configuration done\n");

    /* Populate the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    Mmwave_populateDefaultCalibrationCfg (&calibrationCfg, MMWave_DFEDataOutputMode_FRAME);

    extern void resetCbuffEOLState(void);
    extern void resetPingPongState(void);
    extern void ChirpGeom_reset(uint16_t expectedChirpsPerFrame, uint16_t numCaptureFrames,
                                uint16_t startupDiscard, Bool infiniteRf);
    extern void Mmwave_resetAsyncDiagCounters(void);
    extern volatile uint32_t gCbuffTriggerCount;
    extern volatile uint32_t gCbuffSkipCount;
    extern volatile uint32_t gAsyncFrameStartCount;
    extern volatile uint32_t gAsyncFrameEndCount;
    extern volatile uint32_t gAsyncFrameStopCount;
    extern volatile uint32_t gAsyncErrorCount;
    extern volatile uint32_t gAsyncFaultCount;
    extern volatile uint32_t gAsyncOtherCount;
    extern volatile uint16_t gAsyncLastMsgId;
    extern volatile uint16_t gAsyncLastSbId;
    extern volatile uint32_t gAsyncLastTick;
    extern volatile uint8_t  gAsyncLastDev;
    extern void MmwCascade_csirxSetSkipResetWait(Bool enable);

#if CSIRX_RUNTIME_REARM_MODE
    {
        int32_t csirxRearmErr = 0;
        test_print("CSIRX runtime rearm: enabled (non-blocking mode)\n");
        MmwCascade_csirxClose(&gMmwCascadeMCB);
        MmwCascade_csirxOpen(&gMmwCascadeMCB, &csirxRearmErr);
        if (csirxRearmErr == 0)
        {
            MmwCascade_csirxSetSkipResetWait(true);
            MmwCascade_CSIConfig(&gMmwCascadeMCB);
            MmwCascade_csirxSetSkipResetWait(false);
        }
        else
        {
            test_print("Warning: CSIRX runtime rearm open failed [%d], continuing\n", csirxRearmErr);
        }
    }
#endif

    /* Reset CBUFF/EDMA activation state (needed for session activation on
     * first EOL) and async diagnostic counters.
     * IMPORTANT: do NOT memset gCSIRXState or zero swFrameDoneCount here.
     * Those are shared with live CSIRX ISRs and zeroing them races with
     * concurrent interrupt-driven updates.  Use baseline-delta accounting
     * instead. */
    resetCbuffEOLState();
    resetPingPongState();
    ChirpGeom_reset((uint16_t)CLI_getChirpsPerFrame(&gActive),
                    (uint16_t)gActive.frame.numFrames,
                    gActive.startupDiscardFrames,
                    (Bool)(gActive.frame.numFrames == 0U));
    Mmwave_resetAsyncDiagCounters();
    gChirpHdrFrameIdx      = 0U;
    gChirpHdrChirpIdx      = 0U;
    gChirpHdrGlobalIdx     = 0U;
    gChirpHdrChirpsPerFrame = CLI_getChirpsPerFrame(&gActive);

    printConfigSnapshot();
    printConfigSanityReview();

    /* Capture baselines IMMEDIATELY before MMWave_start so that any
     * pre-existing counter values (from boot-time CSI link training,
     * idle-period spurious events, etc.) are accounted for. */
    uint32_t baseEOF0  = gCSIRXState[0].callbackCount.combinedEOF;
    uint32_t baseEOL0  = gCSIRXState[0].callbackCount.combinedEOL;
#if CONFIG_CSIRX_NUM_INSTANCES > 1
    uint32_t baseEOF1  = gCSIRXState[1].callbackCount.combinedEOF;
    uint32_t baseEOL1  = gCSIRXState[1].callbackCount.combinedEOL;
#endif
    uint32_t baseTrigger = gCbuffTriggerCount;
    uint32_t baseSwDone  = gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swFrameDoneCount;
    uint32_t baseSkips   = gCbuffSkipCount;

    /************************************************************************
     * Start the mmWave:
     ************************************************************************/
    if (MMWave_start (gMmwCascadeMCB.mmWaveHandle, &calibrationCfg, &errCode) < 0)
    {
        test_print ("Error: mmWave start failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Start", MCPI_TestResult_FAIL);
        return;
    }

    test_print ("MMWave MSS Start done\n");

    if (gActive.frame.numFrames == 0)
    {
        test_print ("--- Capture Started (Infinite RF) ---\n");
        test_print ("  Baselines: EOF0=%u EOL0=%u swDone=%u triggers=%u\n",
                    baseEOF0, baseEOL0, baseSwDone, baseTrigger);
        if (gActive.startupDiscardFrames > 0U)
        {
            test_print ("  startupDiscard=%u: first %u RF frames suppressed on LVDS\n",
                        (unsigned)gActive.startupDiscardFrames,
                        (unsigned)gActive.startupDiscardFrames);
        }
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    else
    {
        uint32_t targetEOF0 = baseEOF0 + gActive.frame.numFrames;
        uint32_t chirpsPerFrame = CLI_getChirpsPerFrame(&gActive);
        uint8_t  captureEndByEofTarget = 0U;
        uint8_t  captureEndByWatchdogTimeout = 0U;


        test_print ("--- Capture Started (%u frames, targetEOF=%u) ---\n",
                    (unsigned)gActive.frame.numFrames, (unsigned)targetEOF0);
        test_print ("  Baselines: EOF0=%u EOL0=%u swDone=%u triggers=%u\n",
                    baseEOF0, baseEOL0, baseSwDone, baseTrigger);

        uint32_t lastTrigger = 0;
        uint32_t lastSwDone  = 0;
        uint32_t stallCheckTick   = ClockP_getTicks();

        /* ISR-driven capture loop: The EOL ISR in cascade_csirx.c handles
         * CBUFF_activateSession (first EOL), configureTransfer + trigger (subsequent).
         * This loop only monitors progress and detects stalls.
         *
         * Phase 1 "silent" rules:
         *   - NO per-frame geometry UART during capture (RAM ring only)
         *   - NO CsirxObs polling during capture (deferred to end)
         *   - Stall watchdog checks every ~2 s (no UART unless stall) */
        while(gCSIRXState[0].callbackCount.combinedEOF < targetEOF0)
        {
            ClockP_usleep(10000); /* 10ms sleep -- just monitoring, ISR does the work */

            uint32_t currentTick  = ClockP_getTicks();
            uint32_t triggersRel  = gCbuffTriggerCount - baseTrigger;
            uint32_t swDoneRel    = gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swFrameDoneCount - baseSwDone;

            /* Stall watchdog (~2 s cadence, no UART unless stall detected) */
            if (currentTick - stallCheckTick >= (uint32_t)pdMS_TO_TICKS(2000))
            {
                if (triggersRel > 0 &&
                    triggersRel == lastTrigger &&
                    swDoneRel   == lastSwDone)
                {
                    test_print("[STALL] Freeze at trigger %u, swDone %u. Breaking.\n",
                               (unsigned)triggersRel, (unsigned)swDoneRel);
                    captureEndByWatchdogTimeout = 1U;
                    break;
                }
                lastTrigger   = triggersRel;
                lastSwDone    = swDoneRel;
                stallCheckTick = currentTick;
            }
        }

        if (gCSIRXState[0].callbackCount.combinedEOF >= targetEOF0)
        {
            captureEndByEofTarget = 1U;
        }

        /****************************************************************
         * Diagnostics BEFORE teardown (so counters are still valid)
         ****************************************************************/
        {
            uint32_t deltaEOF0  = gCSIRXState[0].callbackCount.combinedEOF - baseEOF0;
            uint32_t deltaEOL0  = gCSIRXState[0].callbackCount.combinedEOL - baseEOL0;
#if CONFIG_CSIRX_NUM_INSTANCES > 1
            uint32_t deltaEOF1  = gCSIRXState[1].callbackCount.combinedEOF - baseEOF1;
            uint32_t deltaEOL1  = gCSIRXState[1].callbackCount.combinedEOL - baseEOL1;
#endif
            uint32_t deltaTrigger = gCbuffTriggerCount - baseTrigger;
            uint32_t deltaSwDone  = gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swFrameDoneCount - baseSwDone;
            uint32_t deltaSkips   = gCbuffSkipCount - baseSkips;
            uint32_t expectedEol  = gActive.frame.numFrames * chirpsPerFrame;
            uint32_t eolFromEof   = deltaEOF0 * chirpsPerFrame;
            uint32_t bytesPerTrigger = CLI_getBytesPerChirp(&gActive) * MMWAVE_RADAR_DEVICES;
            uint32_t totalBytesSent  = deltaTrigger * bytesPerTrigger;
            uint32_t expectedBytes   = expectedEol * bytesPerTrigger;

            /* Collect CSIRX observability once, now that capture is idle */
            CsirxObs_pollAndAccumulate(&gMmwCascadeMCB);

            /* Per-frame geometry: same valid slots used for summary and [GEOM] dump */
            {
                uint32_t fi;
                uint32_t validCount   = 0U;
                uint32_t idxMismatch  = 0U;
                uint32_t steadyOk     = 0U;
                uint32_t steadyLoss   = 0U;
                uint32_t totalLoss    = 0U;
                uint32_t minObs       = 0xFFFFU;
                uint32_t maxObs       = 0U;
                uint16_t firstShortFi = 0xFFFFU;
                uint16_t lastShortFi  = 0U;
                uint16_t ncfg         = gChirpGeomConfiguredFrames;
                uint32_t steadyLo     = 25U;
                uint32_t steadyHi     = (gActive.frame.numFrames > 10U) ?
                                        (gActive.frame.numFrames - 10U) : gActive.frame.numFrames;

                for (fi = 0U; fi < (uint32_t)ncfg; fi++)
                {
                    PerFrameChirpGeometry_t *g = &gChirpGeomByFrame[fi];
                    if (g->valid != GEOM_ENTRY_VALID)
                    {
                        continue;
                    }
                    validCount++;
                    if (g->frameIdx != (uint16_t)fi)
                    {
                        idxMismatch++;
                    }
                    {
                        uint8_t isSteady = (g->frameIdx >= steadyLo && g->frameIdx < steadyHi) ? 1U : 0U;
                        if (isSteady)
                        {
                            if (g->observedCount == g->expectedCount)
                            {
                                steadyOk++;
                            }
                            else
                            {
                                steadyLoss++;
                            }
                        }
                    }
                    if (g->observedCount < g->expectedCount)
                    {
                        totalLoss++;
                        if (firstShortFi == 0xFFFFU)
                        {
                            firstShortFi = (uint16_t)fi;
                        }
                        lastShortFi = (uint16_t)fi;
                    }
                    if (g->observedCount < minObs)
                    {
                        minObs = g->observedCount;
                    }
                    if (g->observedCount > maxObs)
                    {
                        maxObs = g->observedCount;
                    }
                }

                if (validCount == 0U)
                {
                    minObs = 0U;
                    maxObs = 0U;
                }

                test_print("[GEOM-SUMMARY] slots=%u valid=%u idxMismatch=%u steadyOk=%u steadyLoss=%u "
                           "framesObsLtExp=%u minObs=%u maxObs=%u\n",
                           (unsigned)ncfg, (unsigned)validCount, (unsigned)idxMismatch,
                           (unsigned)steadyOk, (unsigned)steadyLoss, (unsigned)totalLoss,
                           (unsigned)minObs, (unsigned)maxObs);
                if (totalLoss > 0U)
                {
                    test_print("[GEOM-SUMMARY] lossSpan: firstFi=%u lastFi=%u (SW_COUNTER: "
                               "count-only, not chirp index within frame)\n",
                               (unsigned)firstShortFi, (unsigned)lastShortFi);
                }
                else
                {
                    test_print("[GEOM-SUMMARY] lossSpan: none in valid slots (SW_COUNTER: "
                               "count-only, not chirp index within frame)\n");
                }
                test_print("[GEOM-SUMMARY] masterEOF=%u geomCap=%u (max storage %u frames)\n",
                           (unsigned)deltaEOF0, (unsigned)ncfg, CHIRP_GEOM_MAX_FRAMES);

                for (fi = 0U; fi < (uint32_t)ncfg; fi++)
                {
                    PerFrameChirpGeometry_t *g = &gChirpGeomByFrame[fi];
                    if (g->valid != GEOM_ENTRY_VALID)
                    {
                        continue;
                    }
                    test_print("[GEOM] slotFi=%u logFi=%u: obs=%u exp=%u eol1cy=%u eolNcy=%u eofcy=%u\n",
                               (unsigned)g->frameIdx, (unsigned)g->logicalFrameIdx,
                               (unsigned)g->observedCount,
                               (unsigned)g->expectedCount,
                               (unsigned)g->firstEolCycles, (unsigned)g->lastEolCycles,
                               (unsigned)g->eofCycles);
                }
            }

            test_print ("--- Capture Finished! ---\n");
            test_print ("  Baselines: EOF0=%u EOL0=%u swDone=%u triggers=%u\n",
                        baseEOF0, baseEOL0, baseSwDone, baseTrigger);
            test_print ("  Port 0 (delta): EOF=%u EOL=%u CIO=%u OCP=%u FIFO_OVF=%u\n",
                        deltaEOF0, deltaEOL0,
                        gCSIRXState[0].commonIRQcount.isComplexIOerror,
                        gCSIRXState[0].commonIRQcount.isOCPerror,
                        gCSIRXState[0].commonIRQcount.isFIFOoverflow);
#if CONFIG_CSIRX_NUM_INSTANCES > 1
            test_print ("  Port 1 (delta): EOF=%u EOL=%u CIO=%u OCP=%u FIFO_OVF=%u\n",
                        deltaEOF1, deltaEOL1,
                        gCSIRXState[1].commonIRQcount.isComplexIOerror,
                        gCSIRXState[1].commonIRQcount.isOCPerror,
                        gCSIRXState[1].commonIRQcount.isFIFOoverflow);
#endif
            extern volatile uint32_t gCbuffActivateCycles;
            extern volatile uint32_t gCbuffTriggerCycles;
            test_print ("  CBUFF (delta): triggers=%u swDone=%u skips=%u\n",
                        deltaTrigger, deltaSwDone, deltaSkips);
            test_print ("  LATENCY (cycles): Activate=%u Trigger=%u\n",
                        (unsigned)gCbuffActivateCycles, (unsigned)gCbuffTriggerCycles);
            test_print ("  Bytes: bytesPerTrigger=%u totalBytesSent=%u expectedBytes=%u\n",
                        bytesPerTrigger, totalBytesSent, expectedBytes);
            test_print ("  EOL math: chirpsPerFrame=%u expected=%u fromEOF=%u actual=%u\n",
                        chirpsPerFrame, expectedEol, eolFromEof, deltaEOL0);
            test_print ("  End reason: eof_target=%u watchdog_timeout=%u\n",
                        captureEndByEofTarget, captureEndByWatchdogTimeout);
            test_print ("  [CSIRX-OBS-FINAL] P0: ulpmE=%u sotErr=%u sotSync=%u ctrlErr=%u escErr=%u cksum=%u genSP=%u\n",
                        gCsirxObs[0].ulpmEnter, gCsirxObs[0].sotError,
                        gCsirxObs[0].sotSyncError, gCsirxObs[0].controlError,
                        gCsirxObs[0].escapeEntryError, gCsirxObs[0].payloadChecksumMismatch,
                        gCsirxObs[0].genericShortPacket);
#if CONFIG_CSIRX_NUM_INSTANCES > 1
            test_print ("  [CSIRX-OBS-FINAL] P1: ulpmE=%u sotErr=%u sotSync=%u ctrlErr=%u escErr=%u cksum=%u genSP=%u\n",
                        gCsirxObs[1].ulpmEnter, gCsirxObs[1].sotError,
                        gCsirxObs[1].sotSyncError, gCsirxObs[1].controlError,
                        gCsirxObs[1].escapeEntryError, gCsirxObs[1].payloadChecksumMismatch,
                        gCsirxObs[1].genericShortPacket);
#endif

            /* ---- [DISCARD] Startup-frame discard validation ---- */
            {
                extern volatile uint16_t gStartupDiscardFrames;
                extern volatile uint16_t gFirstTransmittedFrameIdx;
                extern volatile uint32_t gLeakedBadFrames;

                uint16_t discN       = gStartupDiscardFrames;
                uint32_t validReq    = gActive.frame.numFrames - discN;
                uint32_t expectedTx  = validReq * chirpsPerFrame;
                uint32_t expectedByt = expectedTx * bytesPerTrigger;

                test_print("[DISCARD] requestedValid=%u startupDiscard=%u internalTarget=%u\n",
                           (unsigned)validReq, (unsigned)discN,
                           (unsigned)gActive.frame.numFrames);
                test_print("[DISCARD] deltaTriggers=%u deltaSwDone=%u expectedTriggers=%u\n",
                           (unsigned)deltaTrigger, (unsigned)deltaSwDone,
                           (unsigned)expectedTx);
                test_print("[DISCARD] bytesSent=%u expectedBytes=%u\n",
                           (unsigned)totalBytesSent, (unsigned)expectedByt);
                test_print("[DISCARD] firstTransmittedFrameIdx=%u (expected=%u)\n",
                           (unsigned)gFirstTransmittedFrameIdx, (unsigned)discN);
                test_print("[DISCARD] leakedBadFrames=%u\n",
                           (unsigned)gLeakedBadFrames);

                if (discN > 0U)
                {
                    if (deltaTrigger != expectedTx)
                        test_print("[DISCARD WARNING] trigger count mismatch: got %u expected %u\n",
                                   (unsigned)deltaTrigger, (unsigned)expectedTx);
                    if (deltaSwDone != deltaTrigger)
                        test_print("[DISCARD WARNING] swDone != triggers: %u vs %u\n",
                                   (unsigned)deltaSwDone, (unsigned)deltaTrigger);
                    if (totalBytesSent != expectedByt)
                        test_print("[DISCARD WARNING] byte count mismatch: %u vs %u\n",
                                   (unsigned)totalBytesSent, (unsigned)expectedByt);
                    if (gFirstTransmittedFrameIdx != discN)
                        test_print("[DISCARD WARNING] firstTxFrame=%u != startupDiscard=%u\n",
                                   (unsigned)gFirstTransmittedFrameIdx, (unsigned)discN);
                    if (gLeakedBadFrames != 0U)
                        test_print("[DISCARD WARNING] leakedBadFrames=%u (should be 0)\n",
                                   (unsigned)gLeakedBadFrames);
                }
            }
        }

        /****************************************************************
         * Stop / Close / Deinit mmWave — matching original am273x/ flow
         ****************************************************************/
        retVal = MMWave_stop(gMmwCascadeMCB.mmWaveHandle, &errCode);
        if (retVal < 0)
        {
            MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            test_print("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);
        }
        else
        {
            test_print("MMWave MSS Stop done.\n");
        }

        if (MMWave_close(gMmwCascadeMCB.mmWaveHandle, &errCode) < 0)
        {
            test_print("Error: mmWave close failed [Error code %d]\n", errCode);
        }
        else
        {
            test_print("MMWave MSS close done.\n");
        }

        if (MMWave_deinit(gMmwCascadeMCB.mmWaveHandle, &errCode) < 0)
        {
            test_print("Error: mmWave deinit failed [Error code %d]\n", errCode);
        }
        else
        {
            test_print("MMWave MSS deinit done.\n");
        }
    }

    return;
}

static void MmwCascade_CsiConfigTask(void* args)
{
    MmwCascade_CSIConfig(&gMmwCascadeMCB);

    vTaskDelete(NULL);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwCascade_initTask(void* args)
{
    int32_t   status = SystemP_SUCCESS;
    int32_t   errCode = 0;

    Drivers_open();
    Board_driversOpen();

    /* Debug Message: */
    test_print ("*********************************************\n");
    test_print ("Debug: Launching mmwave Cascade Application. \n");
    test_print ("*********************************************\n");

    /* Configure HSI interface Clock 
     * Clock Source selected: PLL_PER_CLK - 1728MHz
     */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x333);

    /* Configure CSIRX interface Clock */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_CSIRX_CLK_SRC_SEL, 0x222);

    /* Initialize the result buffer: */
    memset ((void *)&gMmwCascadeMCB, 0, sizeof(MmwCascade_MCB));

    /* Populate edma handle. */
    gMmwCascadeMCB.lvdsStreamcfg.edmaHandle = gEdmaHandle[CONFIG_EDMA2];

    /* Populate PMIC SPI handle. */
    gMmwCascadeMCB.pmicMIBSPIhandle = gMibspiHandle[CONFIG_MIBSPI2];

    /* Initialize LVDS streaming components */
    if ((status = Cascade_LVDSStreamInit()) < 0 )
    {
        test_print ("Error: MMWCascade LVDS stream init failed with Error[%d]\n",status);
    }

    status = SemaphoreP_constructBinary(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    ClockP_usleep(HSI_DCA_MIN_DELAY_MSEC);

    /* Initialize CSIRX interface. */
    MmwCascade_csirxInit(&gMmwCascadeMCB);

    /* Open CSIRX-A handle. */
    MmwCascade_csirxOpen(&gMmwCascadeMCB, &errCode);

    /* Launch the CSIRX Task */
    gCsiConfigTask = xTaskCreateStatic( MmwCascade_CsiConfigTask,   /* Pointer to the function that implements the task. */
                                "test_csi_config_task", /* Text name for the task.  This is to facilitate debugging only. */
                                CSI_CONFIG_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,              /* We are not using the task parameter. */
                                CSI_CONFIG_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gCsiRxCfgTskStack,  /* pointer to stack base */
                                &gCsiConfigTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gCsiConfigTask != NULL);

    /* Wait till CSI RX configuration is complete. */
    status = SemaphoreP_pend(&gMmwCascadeMCB.CSI2RXConfigCompleteSemHandle, SystemP_WAIT_FOREVER);
    DebugP_assert(SystemP_SUCCESS == status);

    Enable_BUCK4_ViaPMIC();

    /*=======================================================================
     * CLI: Spawn CLI task and wait for START_REQUESTED.
     *======================================================================*/
    gCliUartHandle = gUartHandle[CONFIG_UART0];
    gCliState = CLI_STATE_WAITING_FOR_COMMAND;

    gCliTaskHandle = xTaskCreateStatic(
        CLI_task,
        "cli_task",
        sizeof(gCliTaskStack) / sizeof(StackType_t),
        NULL,
        CLI_TASK_PRI,
        gCliTaskStack,
        &gCliTaskObj);
    configASSERT(gCliTaskHandle != NULL);
    DebugP_log("CLI: task spawned, waiting for sensorStart\n");

    /* Block until the CLI task sets START_REQUESTED */
    while (gCliState != CLI_STATE_START_REQUESTED)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Configure SW session for this LVDS Stream NOW that gActive is populated! */
    {
        uint32_t chirpDataSize = CLI_getBytesPerChirp(&gActive);
        int32_t  lvdsRet;
        int32_t  errCode;

        /* SAFETY: If there is an existing session (e.g. from a previous failed run), close it first */
        if (gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle != NULL)
        {
            CBUFF_deactivateSession(gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle, &errCode);
            CBUFF_close(gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle, &errCode);
            gMmwCascadeMCB.lvdsStreamcfg.lvdsStream.swSessionHandle = NULL;
        }

        if (gActive.chirpHeaderEnabled)
        {
            memset(&gChirpHdrPing, 0, sizeof(gChirpHdrPing));
            memset(&gChirpHdrPong, 0, sizeof(gChirpHdrPong));
            lvdsRet = Cascade_LVDSStreamSwConfigWithHeader(
                (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PingBuf),
                (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PingBuf),
                chirpDataSize,
                (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &gChirpHdrPing),
                CHIRP_HDR_SIZE_BYTES);
        }
        else
        {
            lvdsRet = Cascade_LVDSStreamSwConfig(
                (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIA_PingBuf),
                (uint32_t) AddrTranslateP_getLocalAddr((uint32_t) &CSIB_PingBuf),
                chirpDataSize);
        }
        if (lvdsRet < 0)
        {
            test_print("Failed LVDS stream SW configuration\n");
            DebugP_assert(0);
        }
        /* Session activation is done by the EOL ISR on the first chirp arrival.
         * Do NOT activate here — the LVDS link is not ready yet. */
    }

    /* Transition to RUNNING */
    gCliState = CLI_STATE_RUNNING_CAPTURE;
    DebugP_log("CLI: state -> RUNNING_CAPTURE (frames=%u)\n",
               (unsigned)gActive.frame.numFrames);

    /* Configure the Front-End. */
    MmwCascade_mmWaveTest();

    /* Capture done — update state and safely delete CLI task before teardown */
    gCliState = CLI_STATE_CAPTURE_DONE;
    DebugP_log("CLI: state -> CAPTURE_DONE\n");

    if (gCliTaskHandle != NULL)
    {
        vTaskDelete(gCliTaskHandle);
        gCliTaskHandle = NULL;
    }

    /* Close CSIRX */
    MmwCascade_csirxClose(&gMmwCascadeMCB);

    test_print ("--- Test Completed ---\n");

    Board_driversClose();
    Drivers_close();

    vTaskDelete(NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *      Configures CBUFF EDMA channel SRC address for Ping/Pong 
 *      Switch
 *
 *  @retval
 *      Not Applicable.
 */
/* File-scope ping-pong flag for EDMA source address switching.
 * Reset before MMWave_start to stay in sync with gCbuffFirstEOL. */
volatile Bool gPingPongSwitchFlag = true;

static void populateChirpHeader(void)
{
    ChirpHeader_t *hdr = gPingPongSwitchFlag ? &gChirpHdrPing : &gChirpHdrPong;
    hdr->magic          = CHIRP_HDR_MAGIC;
    hdr->version        = CHIRP_HDR_VERSION;
    hdr->flags          = (gActive.tdmEnabled ? 1U : 0U)
                        | ((gActive.adcFmt != 0U) ? 2U : 0U);
    hdr->frameIdx       = gChirpHdrFrameIdx;
    hdr->chirpIdx       = gChirpHdrChirpIdx;
    hdr->globalChirpIdx = gChirpHdrGlobalIdx;
    hdr->timestampTicks = ClockP_getTicks();

    gChirpHdrGlobalIdx++;
    gChirpHdrChirpIdx++;
    if (gChirpHdrChirpIdx >= gChirpHdrChirpsPerFrame)
    {
        gChirpHdrChirpIdx = 0U;
        gChirpHdrFrameIdx++;
    }
}

void resetPingPongState(void)
{
    gPingPongSwitchFlag = true;
}

void configureTransfer(void)
{
    if(gPingPongSwitchFlag)
    {
        /* Update PaRAM set Source address for capturing CSIRX data received on PONG buffer. */
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PongBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PongBuf));

        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PongBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PongBuf));

        gPingPongSwitchFlag = false;
    }
    else
    {
        /* Update PaRAM set Source address for capturing CSIRX data received on PING buffer. */
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PingBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIA_PingBuf));

        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PingBuf));
        EDMA_dmaSetPaRAMEntry(CONFIG_EDMA2_BASE_ADDR, CASCADE_LVDS_STREAM_SW_SESSION_EDMA_SHADOW_CH_0, EDMACC_PARAM_ENTRY_SRC, (uint32_t) SOC_virtToPhy((void *)&CSIB_PingBuf));

        gPingPongSwitchFlag = true;
    }

    return;
}

void skipTransfer(void)
{
    gPingPongSwitchFlag = !gPingPongSwitchFlag;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the mmWave Unit Test
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main (void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

    /* This task is created at highest priority, it should create more tasks and then delete itself */
    gAppTask = xTaskCreateStatic( MmwCascade_initTask,   /* Pointer to the function that implements the task. */
                                  "test_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  APP_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  APP_TASK_PRI,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gAppTskStackMain,  /* pointer to stack base */
                                  &gAppTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gAppTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);
}

