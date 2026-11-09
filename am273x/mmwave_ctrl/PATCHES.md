# Vendored TI mmWave control library (patched)

Source: `mmwave_mcuplus_sdk_04_06_01_02/ti/control/mmwave/src` (the sources of
`libmmwave_cascade_am273x.aer5f`, built with `-DCASCADE_EVM`). SHA-256 of the
unmodified copies: `ORIGINAL_SHA256.txt`.

These files are compiled into the application (`am273x/mssTest.mak`). The
prebuilt `libmmwave_cascade_am273x` is no longer linked.

| File | Changed |
|---|---|
| mmwave.c, mmwave_osal.c, mmwave_fullcfg.c, mmwave_listlib.c | no (identical to TI) |
| mmwave_link_common.c | yes |
| mmwave_link_spi_22xx.c | yes |

Every change is marked `CHIRP PATCH` in the source.

## Why

On the AWR2243 2-chip cascade EVM the **slave** AWR2243 intermittently reports
`RL_DEV_AE_MSS_CPUFAULT_SB` at the first HW-triggered frame:
faultType 4 ("MSS Processor Chirp Errors"), errorCode 0, faultLR `0x00018800`.
In stress tests this happened in about 60% of power-ups.

TI's `MMWave_asyncEventHandler` called `DebugP_assert(0)` for any CPU fault except
LR `0xFFFFE580`. That spins forever, so CSI-2 interrupts were never serviced,
CBUFF was never triggered, LVDS stayed silent, the UART CLI died, and the
DCA1000 captured nothing.

## Changes

1. **mmwave_link_common.c, `MMWave_asyncEventHandler`, `RL_DEV_AE_MSS_CPUFAULT_SB`**:
   the assert is replaced by recording the fault in `gMMWavePatchFeFault[dev]`
   (count, totalCount, tick, raw `rlCpuFault_t` words). The event is still passed
   on to the application `eventFxn`, and `frontEndMssStatus` is still updated.
2. **mmwave_link_common.c, async handler `default:`**: an unknown msgId is
   counted in `gMMWavePatchInvalidAsyncMsgCount` instead of asserting.
3. **mmwave_link_common.c, `MMWave_startLink`**: the fixed 1 ms gap between
   `rlSensorStart(slaves)` and `rlSensorStart(master)` is now
   `gMMWavePatchSlaveStartGapUs` (default 1000 us; CLI `setStartGap <us>`).
4. **mmwave_link_spi_22xx.c, `MMWave_configSpiGpio`**: destructs the SPI host-IRQ
   Hwi before reconstructing it, so `MMWave_open` can run more than once
   (needed for front-end recovery).

5. **Interrupt teardown and bounded waits**: disable/clear SPI GPIO interrupts
   and destruct their Hwi before deinitializing the control block. Power-up and
   RF-init waits time out after 10 seconds; a persistently high HOST_IRQ is
   serviced after 50 ms to recover a possibly missed edge. These changes were
   present in Claude's last successful injected-failure bench test.
6. **Device reset during teardown**: `MMWave_disableDevice`
   now actually drives the device's nRESET low when mmWaveLink powers it down.
   Previously this registered callback was empty, so teardown did not hold the
   radar chips in reset as intended. This asserts reset; it does not remove
   power from the board. The 18:40 test recovered from a real slave fault with
   this callback implemented.
7. **Bound the lower-level handshake**: bound the lower-level
   `MMWave_waitIRQStatus` handshake as well. An unknown SPI handle or a stuck
   GPIO now returns an error instead of indexing beyond the device array or
   waiting indefinitely. No extra IRQ handler is invoked inside this SPI
   handshake; mmWaveLink handles the failed handshake and retries CNYS.

The application (`main_mss.c`) checks, after `MMWave_start`, that both CSI-2
ports deliver two frames with no front-end fault (one for a one-frame finite
capture). If not, it tears the
mmWave stack down (`MMWave_stop/close`, delete ctrl task, `MMWave_deinit`, which
now holds the AWR2243s in reset through nRESET), reports
`{"event":"fe_start_retry",...}` on the CLI UART, and retries (CLI
`setStartRetries <n>`, default 5).

All failed attempts, including init/open/config failures, count toward the
`retries` diagnostic. Events and `status.fe` include `lastError`, `mmWaveError`,
`subsysError`, `irqKicks` and `linkTimeouts`. Negative 14-bit mmWaveLink errors
are sign-extended for these reports. A successful `rf_started` event retains
the last failure diagnostics for inspection; the final `reason` tracks the
actual last failed stage if all attempts fail.

The recorded configuration error `-203685918` (`0xF3DBFFE2`) decodes to
`MMWAVE_ECHIRPCFG` (-3109) and `RL_RET_CODE_RESP_TIMEOUT` (-8), from
`rlSetChirpConfig`. That identifies a missing response during chirp programming;
it does not establish why the response was missed.

The test hook `setFeTestFailures <n>` rejects commands once capture startup has
begun, matching the other startup controls.

## Ordered application shutdown

The application now requests cooperative shutdown of `Mmwave_ctrlTask`
(`common.c`), wakes its `MMWave_execute` wait with
`MMWavePatch_wakeControlTask`, and waits for the task to suspend after its last
SPI operation returns. It then deletes the suspended task before deinitializing
mmWave and closing drivers. Deleting a task while it was blocked in
`MIBSPI_transfer` left a pointer to its stack-local transaction in the slave SPI
driver and caused `MIBSPI_closeCore` to assert. The debugger confirmed that
outstanding pointer on the failed finite test.

If cooperative shutdown times out, the application reports
`fe_teardown_failed` and retains the task, its stack and driver objects rather
than closing an active driver. This timeout fallback has not been deliberately
triggered in hardware testing.

The finite EOL callback also suppresses transfers after the physical RF target
has completed. The target is separate from the capped geometry storage, so
captures longer than the geometry array remain supported. This removed one
extra chirp emitted by a pending EOL after final EOF in the one-frame test.

## Hardware validation, 2026-11-09

- At 18:40 the real slave CPU fault (`faultType=4`, LR `0x00018800`) occurred.
  The AM273x remained responsive, retried a chirp-config timeout, then captured
  112311 packets on attempt 3; CSI counts and CBUFF completion counts matched.
- The final cooperative-shutdown build passed a one-frame test: 384 triggers,
  384 completions, 6291456 emitted bytes, idle call stack after driver cleanup.
- The same final build passed two forced startup failures with CCS detached,
  then a full 60-second watch window: 1533938 received packets, matched CSI
  counts (140928 each), and matched CBUFF triggers/completions (138624 each).
  The receiver recorded 36 initial packet gaps; this was not a lossless capture.

The original chip fault's underlying trigger and the missed chirp-config
response remain unexplained. Forced healthy-start failures do not emulate the
CPU-fault handler. Repeated physical power-cycle testing is still appropriate
before claiming a startup failure rate. Per-run logs, frozen executables and
matching symbols are under `C:/CHIRP/FIRMWARE/ccs_pipeline/runs`; see
`C:/CHIRP/FIRMWARE/ccs_pipeline/HARDWARE_RESULTS.md`.
