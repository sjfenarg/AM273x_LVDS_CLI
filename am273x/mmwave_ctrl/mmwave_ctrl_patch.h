/*
 * mmwave_ctrl_patch.h
 *
 * Local patches applied to the vendored TI mmWave control sources in this
 * directory (copied from mmwave_mcuplus_sdk_04_06_01_02/ti/control/mmwave/src;
 * see ORIGINAL_SHA256.txt and PATCHES.md).
 *
 * The TI library asserts (DebugP_assert -> infinite spin) when an AWR2243
 * reports an MSS CPU fault. On the cascade EVM the slave intermittently reports
 * faultType 4 (MSS Processor Chirp Errors) at the first HW-triggered frame,
 * which froze the AM273x and left LVDS silent. These hooks record the fault
 * instead so the application can report it and recover.
 */
#ifndef MMWAVE_CTRL_PATCH_H
#define MMWAVE_CTRL_PATCH_H

#include <stdint.h>

#define MMWAVE_PATCH_MAX_DEVICES        (2U)
#define MMWAVE_PATCH_FAULT_WORDS        (10U)

/* Per front-end MSS CPU fault bookkeeping (written from the mmWave ctrl task). */
typedef struct MMWavePatch_FeFault_t
{
    volatile uint32_t count;                              /* faults since last clear       */
    volatile uint32_t totalCount;                         /* faults since boot             */
    volatile uint32_t lastTick;                           /* xTaskGetTickCount at report   */
    volatile uint32_t payload[MMWAVE_PATCH_FAULT_WORDS];  /* raw rlCpuFault_t words        */
} MMWavePatch_FeFault;

extern MMWavePatch_FeFault gMMWavePatchFeFault[MMWAVE_PATCH_MAX_DEVICES];

/* Async messages with an unknown msgId (TI code asserted on these). */
extern volatile uint32_t gMMWavePatchInvalidAsyncMsgCount;

/* Delay between rlSensorStart(slaves) and rlSensorStart(master).
 * TI default is 1000 us. Runtime-tunable (CLI: setStartGap <us>). */
extern volatile uint32_t gMMWavePatchSlaveStartGapUs;

/* Clear the per-start fault counters (totalCount is kept). */
void MMWavePatch_clearFeFaults(void);

/* Disable the AWR2243 SPI host-IRQ GPIO interrupts and destruct their Hwi
 * (mmwave_link_spi_22xx.c); called from MMWave_deinitLink. */
void MMWavePatch_disableSpiGpio(void);
/* Wake an idle MMWave_execute so its application task can acknowledge stop. */
void MMWavePatch_wakeControlTask(void);

/* Bounded waits for front-end async events (replaces TI's unbounded loops).
 * HOST_IRQ is edge-triggered; a lost edge leaves the line high forever. If the
 * line stays high for MMWAVE_PATCH_IRQ_STUCK_MS the mmWaveLink IRQ handler is
 * invoked directly; after MMWAVE_PATCH_LINK_SYNC_TIMEOUT_MS the wait fails. */
#define MMWAVE_PATCH_IRQ_STUCK_MS           (50U)
#define MMWAVE_PATCH_LINK_SYNC_TIMEOUT_MS   (10000U)

extern volatile uint32_t gMMWavePatchIrqKickCount;
extern volatile uint32_t gMMWavePatchLinkSyncTimeoutCount;

/* One 1 ms poll step for device devIdx. Returns 0 to keep waiting, -1 on timeout. */
int32_t MMWavePatch_feWaitStep(uint32_t devIdx, uint32_t* waitedMs, uint32_t* highMs);

#endif /* MMWAVE_CTRL_PATCH_H */
