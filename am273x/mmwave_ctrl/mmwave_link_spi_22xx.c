/**
 *   @file  mmwave_link_spi_22xx.c
 *
 *   @brief
 *      The file implements the mmWave control module which interfaces
 *      with the mmWave Link API and communicates with the Front end
 *      radar device.
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
/* Uncomment below to enable DebugP traces*/
#define DebugP_LOG_ENABLED 1

/* Uncomment below to enable SPI traces*/
/*#define MMWAVE_ENABLE_SPI_TRACE 1*/

#include <stdint.h>
#include <string.h>

#include <drivers/hw_include/am273x/soc_config.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/gpio.h>

#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwave/include/mmwave_internal.h>
#include "mmwave_ctrl_patch.h"


HwiP_Object         gGpioHwiObject[MMWAVE_RADAR_DEVICES];

/* CHIRP PATCH: tracks whether gGpioHwiObject[0] is constructed (see
 * MMWave_configSpiGpio and MMWavePatch_disableSpiGpio). */
static uint8_t      gGpioHwiConstructed = 0U;

/* CHIRP PATCH: called from MMWave_deinitLink before the front ends are powered
 * off and before MMWave_init memsets gMMWave_MCB. Without this the host-IRQ Hwi
 * stays live while openCfg (GPIO base addresses) is zeroed, so the ISR can never
 * identify/clear the pending pin and the MCU livelocks in an interrupt storm. */
void MMWavePatch_disableSpiGpio(void)
{
    uint32_t u32DevIdx;

    for (u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        uint32_t base = gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr;
        uint32_t pin  = gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex;
        if (base != 0U)
        {
            GPIO_disableInterrupt(base, pin);
            GPIO_clearInterrupt(base, pin);
        }
    }
    if (gGpioHwiConstructed != 0U)
    {
        HwiP_destruct(&gGpioHwiObject[0]);
        gGpioHwiConstructed = 0U;
    }
    gMMWave_MCB.mmwavelinkInterruptFunc = NULL;
}

/* CHIRP PATCH: bounded wait step, see mmwave_ctrl_patch.h */
volatile uint32_t gMMWavePatchIrqKickCount         = 0U;
volatile uint32_t gMMWavePatchLinkSyncTimeoutCount = 0U;

int32_t MMWavePatch_feWaitStep(uint32_t devIdx, uint32_t* waitedMs, uint32_t* highMs)
{
    uint32_t base = 0U;
    uint32_t pin  = 0U;

    if (devIdx < MMWAVE_RADAR_DEVICES)
    {
        base = gMMWave_MCB.openCfg.frontEndCfg[devIdx].gpioBaseAddr;
        pin  = gMMWave_MCB.openCfg.frontEndCfg[devIdx].spiIrqGpioIndex;
    }

    ClockP_usleep(1000U);
    (*waitedMs)++;

    if ((base != 0U) && (GPIO_pinRead(base, pin) != 0U))
    {
        (*highMs)++;
        if ((*highMs >= MMWAVE_PATCH_IRQ_STUCK_MS) && (gMMWave_MCB.mmwavelinkInterruptFunc != NULL))
        {
            /* Lost edge: service the pending host IRQ as the ISR would have. */
            gMMWavePatchIrqKickCount++;
            gMMWave_MCB.mmwavelinkInterruptFunc((uint8_t)devIdx, NULL);
            *highMs = 0U;
        }
    }
    else
    {
        *highMs = 0U;
    }

    if (*waitedMs >= MMWAVE_PATCH_LINK_SYNC_TIMEOUT_MS)
    {
        gMMWavePatchLinkSyncTimeoutCount++;
        return -1;
    }
    return 0;
}
/**************************************************************************
 *************************** Definitions ********************************
 **************************************************************************/

/* CRC type of Async Event message from MSS to Host. Use 16 Bit CRC. */
#define AWR_22xx_ES_1_1                       1U

#if (AWR_22xx_ES_1_1 == 1U)
    /* Boot Mode = SPI (Deployment Phase) */
    #define FIRMWARE_DOWNLOAD                 1U
#else
    #define FIRMWARE_DOWNLOAD                 0U
#endif


#if (FIRMWARE_DOWNLOAD == 1U)
/* AWR2243 meta image file */
#include <firmware/xwr22xx_metaImage.h>

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/
/* Firmware File Type */
#define MMWL_FILETYPE_META_IMG                       (0U)

#define MMWL_FW_FIRST_CHUNK_SIZE                   (224U)
#define MMWL_FW_CHUNK_SIZE                         (232U)
#define MMWL_META_IMG_FILE_SIZE       (sizeof(metaImage))
#endif


/**************************************************************************
 ***************************** Local Functions ****************************
 **************************************************************************/
static rlComIfHdl_t MMWave_spiOpen(rlUInt8_t deviceIndex, uint32_t flags);
static int32_t MMWave_spiRead(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len);
static int32_t MMWave_spiWrite(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len);
static int32_t MMWave_spiClose(rlComIfHdl_t fd);
static int32_t MMWave_enableDevice(rlUInt8_t deviceIndex);
static int32_t MMWave_disableDevice(rlUInt8_t deviceIndex);
static void MMWave_maskHostIRQ(rlComIfHdl_t fd);
static void MMWave_unmaskHostIRQ(rlComIfHdl_t fd);
static int32_t MMWave_waitIRQStatus(rlComIfHdl_t fd, rlUInt8_t Level);
static int32_t MMWave_registerInterruptHandler(rlUInt8_t deviceIndex, RL_P_EVENT_HANDLER pHandler, void* pValue);
static int32_t MMWave_initMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
#if (AWR_22xx_ES_1_1 == 0U)
static int32_t MMWave_SwapResetAndPowerOn(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
#endif
static void MMWave_configNresetGpio (const MMWave_MCB* ptrMMWaveMCB);
static void MMWave_configSpiGpio (const MMWave_MCB* ptrMMWaveMCB);
static int32_t MMWave_setMiscConfig(const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
static void MMWave_gpioSpiIrqDevCallbackFxn (void *args);

#if (FIRMWARE_DOWNLOAD == 1U)

int32_t MMWave_fileWrite(uint8_t deviceMap,
                uint16_t remChunks,
                uint16_t chunkLen,
                uint8_t *chunk)
{
    int32_t ret_val = -1;

    rlFileData_t fileChunk = { 0 };
    fileChunk.chunkLen = chunkLen;
    memcpy(fileChunk.fData, chunk, chunkLen);

    ret_val = rlDeviceFileDownload(deviceMap, &fileChunk, remChunks);
    return ret_val;
}

/**
 *  @b Description
 *  @n
 *      File Download API.
 *
 *   @param[in] deviceMap - Devic Index
 *   @param[in] fileLen - firmware/file length
 *
 *  @retval
 *      Success - 0
 *      Failure - Error Code
 */
int32_t MMWave_fileDownload(uint8_t deviceMap,
                  uint32_t fileLen)
{
    uint32_t imgLen = fileLen;
    int32_t ret_val = -1;
    int32_t mmwl_iRemChunks = 0;
    uint16_t usChunkLen = 0U;
    uint32_t iNumChunks = 0U;
    uint16_t usLastChunkLen = 0;
    uint16_t usFirstChunkLen = 0;
    uint16_t usProgress = 0;

    /*First Chunk*/
    uint8_t firstChunk[MMWL_FW_CHUNK_SIZE];
    uint8_t* pmmwl_imgBuffer = NULL;

    /* Patch file header file included for ES1.1 device */
    pmmwl_imgBuffer = (uint8_t*)&metaImage[0];

    if(pmmwl_imgBuffer == NULL)
    {
        DebugP_logInfo("MMWL_fileDwld Fail : File Buffer is NULL \n\n\r");
        return -1;
    }

    /*Download to Device*/
    usChunkLen = MMWL_FW_CHUNK_SIZE;
    iNumChunks = (imgLen + 8) / usChunkLen;
    mmwl_iRemChunks = iNumChunks;

    if (mmwl_iRemChunks > 0)
    {
        usLastChunkLen = (imgLen + 8) % usChunkLen;
        usFirstChunkLen = MMWL_FW_CHUNK_SIZE;
        mmwl_iRemChunks += 1;
    }
    else
    {
        usFirstChunkLen = imgLen + 8;
    }

    *((uint32_t*)&firstChunk[0]) = (uint32_t)MMWL_FILETYPE_META_IMG;
    *((uint32_t*)&firstChunk[4]) = (uint32_t)imgLen;
    memcpy((char*)&firstChunk[8], (char*)pmmwl_imgBuffer,
                usFirstChunkLen - 8);

    ret_val = MMWave_fileWrite(deviceMap, (mmwl_iRemChunks-1), usFirstChunkLen,
                              firstChunk);
    if (ret_val < 0)
    {
        DebugP_logInfo("MMWL_fileDwld Fail : Ftype: %d\n\n\r", MMWL_FILETYPE_META_IMG);
        return ret_val;
    }
    pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
    mmwl_iRemChunks--;

    if(mmwl_iRemChunks > 0)
    {
        DebugP_logInfo("Download in Progress: ");
    }
    /*Remaining Chunk*/
    while (mmwl_iRemChunks > 0)
    {
        usProgress = (((iNumChunks - mmwl_iRemChunks) * 100) / iNumChunks);
        DebugP_logInfo("Progress %%d.....\n\r", usProgress);

        /* Last chunk */
        if ((mmwl_iRemChunks == 1) && (usLastChunkLen > 0))
        {
            ret_val = MMWave_fileWrite(deviceMap, 0, usLastChunkLen,
                pmmwl_imgBuffer);
            if (ret_val < 0)
            {
                DebugP_logInfo("MMWL_fileDwld last chunk Fail : Ftype: %d\n\n\r", MMWL_FILETYPE_META_IMG);
                return ret_val;
            }
        }
        else
        {
            ret_val = MMWave_fileWrite(deviceMap, (mmwl_iRemChunks - 1),
                MMWL_FW_CHUNK_SIZE, pmmwl_imgBuffer);

            if (ret_val < 0)
            {
                DebugP_logInfo("\n\n\r MMWL_fileDwld rem chunk Fail : Ftype: %d\n\n\r", MMWL_FILETYPE_META_IMG);
                return ret_val;
            }
            pmmwl_imgBuffer += MMWL_FW_CHUNK_SIZE;
        }

        mmwl_iRemChunks--;
    }

    return ret_val;
}

/**
 *  @b Description
 *  @n
 *      Firmware Download API.
 *
 *  @param[in]  deviceMap
 *      Device Index
 *
 *  @retval
 *      Success - 0
 *      Failure - Error Code
 */
static int32_t MMWave_FirmwareDownload(uint8_t deviceMap)
{
    int32_t retVal = RL_RET_CODE_OK;

    /* Meta Image download */
    DebugP_logInfo("Meta Image download started for deviceMap %u\n\n", deviceMap);
    retVal = MMWave_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
    DebugP_logInfo("Meta Image download complete ret = %d\n\n", retVal);

    return retVal;
}
#endif
/**************************************************************************
 ************************ mmWave Link Functions ***************************
 **************************************************************************/


/**
 *  @b Description
 *  @n
 *      IRQ GPIO call back function for device 0. This call back should be
 *      invoked only when transition from IRQ low to IRQ high.
 *
 *  @param[in]  GPIOindex
 *      GPIO index
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void MMWave_gpioSpiIrqDevCallbackFxn (void *args)
{
    volatile uint32_t    pendingInterrupt = 0;
    volatile uint32_t    pin = 0;

    /* Get the pending GPIO interrupt: */
    pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(gMMWave_MCB.openCfg.frontEndCfg[0].gpioBaseAddr, GPIO_INTR_LEVEL_HIGH);
    pin = (pendingInterrupt - 1) % 8U;

    if((GPIO_GET_PIN_INDEX(pin)  == gMMWave_MCB.openCfg.frontEndCfg[0].spiIrqGpioIndex) && (pendingInterrupt))
    {
        GPIO_clearInterrupt(gMMWave_MCB.openCfg.frontEndCfg[0].gpioBaseAddr, gMMWave_MCB.openCfg.frontEndCfg[0].spiIrqGpioIndex);
        gMMWave_MCB.debugStats.numIRQReceived++;
        DebugP_logInfo("IRQ received value %d\n",GPIO_pinRead(gMMWave_MCB.openCfg.frontEndCfg[0].gpioBaseAddr,
                                                        gMMWave_MCB.openCfg.frontEndCfg[0].spiIrqGpioIndex));
        DebugP_logInfo("Number of IRQ received: %d\n",gMMWave_MCB.debugStats.numIRQReceived);

        /* Invoke the mmwavelink event handler function - rlDriverHostIrqHandler().
        Device index is zero. This may change in the cascade case.   */
        if(gMMWave_MCB.mmwavelinkInterruptFunc != NULL)
        {
            gMMWave_MCB.mmwavelinkInterruptFunc(0, NULL);
        }
    }

    if (MMWAVE_RADAR_DEVICES > 1)
    {
        if((GPIO_GET_PIN_INDEX(pin)  == gMMWave_MCB.openCfg.frontEndCfg[1].spiIrqGpioIndex) && (pendingInterrupt))
        {
            GPIO_clearInterrupt(gMMWave_MCB.openCfg.frontEndCfg[1].gpioBaseAddr, gMMWave_MCB.openCfg.frontEndCfg[1].spiIrqGpioIndex);
            gMMWave_MCB.debugStats.numIRQReceived++;
            DebugP_logInfo("IRQ received value %d\n",GPIO_pinRead(gMMWave_MCB.openCfg.frontEndCfg[1].gpioBaseAddr,
                                                            gMMWave_MCB.openCfg.frontEndCfg[1].spiIrqGpioIndex));
            DebugP_logInfo("Number of IRQ received: %d\n",gMMWave_MCB.debugStats.numIRQReceived);

            /* Invoke the mmwavelink event handler function - rlDriverHostIrqHandler().
            Device index is zero. This may change in the cascade case.   */
            if(gMMWave_MCB.mmwavelinkInterruptFunc != NULL)
            {
                gMMWave_MCB.mmwavelinkInterruptFunc(1, NULL);
            }
        }
    }
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to register the
 *      interrupt handler.
 *
 *  @param[in]  deviceIndex
 *      Device index for which the interrupt is to be registered
 *  @param[in]  pHandler
 *      ISR Handler
 *  @param[in]  pValue
 *      Argument to the ISR
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Always returns 0
 */
int32_t MMWave_registerInterruptHandler(rlUInt8_t deviceIndex, RL_P_EVENT_HANDLER pHandler, void* pValue)
{
    /* As described in the mmwavelink documentation:
       The Application should store this function handler
       and invoke when it receives host Interrupt.*/
    gMMWave_MCB.mmwavelinkInterruptFunc = pHandler;
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to open the communication
 *      interface channel
 *
 *  @param[in]  deviceIndex
 *      Device index for which the communication channel to be opened
 *  @param[in]  flags
 *      Flags which are passed to configure the interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - Communicate Interface Channel Handle
 *  @retval
 *      Error   - NULL
 */
static rlComIfHdl_t MMWave_spiOpen(rlUInt8_t deviceIndex, uint32_t flags)
{
    /* Application is responsible of the SPI configuration */
    DebugP_logInfo("MMWave_spiOpen: SPI Handle 0x%x\n",(uintptr_t)(gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].spiHandle));
    return (rlComIfHdl_t)gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].spiHandle;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to close the communication
 *      interface channel
 *
 *  @param[in]  fd
 *      Opaque Handle to the communication channel which is to be closed
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MMWave_spiClose(rlComIfHdl_t fd)
{
    return 0;
}

uint8_t gReadBuf[1024 * 4];
uint32_t gReadBufIndex = 0;


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to read data from the
 *      communication interface channel
 *
 *  @param[in]  fd
 *      Opaque Handle to the communication channel
 *  @param[out]  pBuff
 *      Pointer to the data buffer which is populated with the received message
 *  @param[in]  len
 *      Size of the data buffer
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - Number of bytes of received data
 *  @retval
 *      Error   - <0
 */
static int32_t MMWave_spiRead(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    MIBSPI_Transaction transaction;
    int32_t retVal = len;

    CacheP_wbInv((void*)pBuff, len, CacheP_TYPE_ALLD);

    /* Configure Data Transfer */
    transaction.count = len;
    transaction.txBuf = NULL;
    transaction.rxBuf = (void*)pBuff;
    transaction.peripheralIndex = 0U;

    /* Start Data Transfer */
    if (MIBSPI_transfer((MIBSPI_Handle)fd, &transaction) != SystemP_SUCCESS)
    {
        retVal = MINUS_ONE;
        DebugP_logInfo("Error!!  MMWave_spiRead \n");
    }

#ifdef SUBSYS_MSS
    CacheP_inv((void*)pBuff, len, CacheP_TYPE_ALLD);
#endif

    gMMWave_MCB.debugStats.numControlChannelRead++;

    {
        uint32_t i;
        for (i = 0; i < len; i++)
        {
            gReadBuf[(gReadBufIndex % (sizeof(gReadBuf)))] = pBuff[i];
            gReadBufIndex++;
        }
        gReadBuf[(gReadBufIndex % (sizeof(gReadBuf)))] = 0xAB;
        gReadBufIndex++;
        gReadBuf[(gReadBufIndex % (sizeof(gReadBuf)))] = 0xBA;
        gReadBufIndex++;
    }

    #ifdef MMWAVE_ENABLE_SPI_TRACE
    uint8_t devIdx = *((rlUInt8_t*)fd);
    uint32_t i = 0;
    DebugP_logInfo("Device [%d] [RD]", devIdx);
    for (i = 0; i < len; i += 2)
    {
        DebugP_logInfo("0x%02x%02x", pBuff[i + 1],pBuff[i]);
    }
    #endif

    return retVal;
}

uint8_t gWriteBuf[1024];
uint32_t gWriteBufIndex = 0;

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to write data to the
 *      communication interface channel
 *
 *  @param[in]  fd
 *      Opaque Handle to the communication channel
 *  @param[in]  pBuff
 *      Pointer to the data buffer which is to be written out
 *  @param[in]  len
 *      Size of the data buffer
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - Number of bytes of data which is sent
 *  @retval
 *      Error   - <0
 */
static int32_t MMWave_spiWrite(rlComIfHdl_t fd, uint8_t* pBuff, uint16_t len)
{
    int32_t retVal = len;
    MIBSPI_Transaction transaction;

    #ifdef MMWAVE_ENABLE_SPI_TRACE
    uint8_t devIdx = *((rlUInt8_t*)fd);
    DebugP_logInfo("Device [%d] [WR]", devIdx);
    uint32_t i = 0;
    for (i = 0; i < len; i += 2)
    {
        DebugP_logInfo("0x%02x%02x", pBuff[i + 1],pBuff[i]);
    }
    #endif

    CacheP_wbInv((void*)pBuff, len, CacheP_TYPE_ALLD);

    /* Configure Data Transfer */
    transaction.count = len;
    transaction.txBuf = (void*)pBuff;
    transaction.rxBuf = NULL;
    transaction.peripheralIndex = 0;
    transaction.arg        = NULL;

    gMMWave_MCB.debugStats.numControlChannelWrite++ ;

    /* Start Data Transfer */
    if (MIBSPI_transfer((MIBSPI_Handle)fd, &transaction) != SystemP_SUCCESS)
    {
        retVal = MINUS_ONE;
        DebugP_logInfo("Error!!  MMWave_spiWrite \n");
    }

    {
        uint32_t i;
        for (i = 0; i < len; i++)
        {
            gWriteBuf[(gWriteBufIndex % (sizeof(gWriteBuf)))] = pBuff[i];
            gWriteBufIndex++;
        }
        gWriteBuf[(gWriteBufIndex % (sizeof(gWriteBuf)))] = 0xFA;
        gWriteBufIndex++;
        gWriteBuf[(gWriteBufIndex % (sizeof(gWriteBuf)))] = 0xCE;
        gWriteBufIndex++;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to power on and enable
 *      the XWR1XX device
 *
 *  @param[in]  deviceIndex
 *      Device index which is to be enabled
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MMWave_enableDevice(rlUInt8_t deviceIndex)
{
    DebugP_logInfo("will set nreset low\n");

    GPIO_pinWriteLow(gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].gpioBaseAddr,
                     gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].nresetGpioIndex);

    ClockP_usleep(2 *1000);

    DebugP_logInfo("will set nreset high\n");

    GPIO_pinWriteHigh(gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].gpioBaseAddr,
                      gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].nresetGpioIndex);

    ClockP_usleep(2 *1000);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to power off and disable
 *      the device
 *
 *  @param[in]  deviceIndex
 *      Device index which is to be disabled
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
static int32_t MMWave_disableDevice(rlUInt8_t deviceIndex)
{
    /* CHIRP PATCH: rlDevicePowerOff calls this for each connected front end.
     * TI's empty callback left the chips running during deinit/reinit. Hold
     * nRESET low until MMWave_enableDevice performs the next boot sequence. */
    if ((deviceIndex >= MMWAVE_RADAR_DEVICES) ||
        (gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].gpioBaseAddr == 0U))
    {
        return -1;
    }
    GPIO_pinWriteLow(gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].gpioBaseAddr,
                     gMMWave_MCB.openCfg.frontEndCfg[deviceIndex].nresetGpioIndex);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to mask the
 *      interrupts.
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_maskHostIRQ(rlComIfHdl_t fd)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to unmask the
 *      interrupts to indicate that the message has been successfully handled
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_unmaskHostIRQ(rlComIfHdl_t fd)
{
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to poll the
 *      host interrupt status.
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *  @param[in]  level
 *      Wait for IRQ Level(high/low)
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      returns GPIO level (high/low)
 */
static int32_t MMWave_waitIRQStatus(rlComIfHdl_t fd, rlUInt8_t level)
{
    volatile int32_t value = 0;
    uint32_t u32DevIdx;
    uint32_t waitedMs = 0U;

    /* Recover the index of the device */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if(fd == (rlComIfHdl_t) gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].spiHandle)
        {
            /* Index of the device matching with the handle of the SPI communication found. */
            break;
        }
    }

    /* CHIRP PATCH: never index past frontEndCfg for an unknown SPI handle. */
    if (u32DevIdx >= MMWAVE_RADAR_DEVICES)
    {
        return -1;
    }

    value = GPIO_pinRead(gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                         gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex);

    DebugP_logInfo("MMWave_waitIRQStatus want level %d got value %d\n",level, value);

    while(value != level)
    {
        /* CHIRP PATCH: this handshake runs inside mmWaveLink's SPI exchange.
         * Do not invoke another IRQ handler here; return a timeout so the
         * driver can retry CNYS and eventually report HOSTIRQ_TIMEOUT. */
        if (waitedMs >= MMWAVE_PATCH_LINK_SYNC_TIMEOUT_MS)
        {
            gMMWavePatchLinkSyncTimeoutCount++;
            return -1;
        }
        ClockP_usleep(1000);
        waitedMs++;

        value = GPIO_pinRead(gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                            gMMWave_MCB.openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex);
    }

    return value;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave Link
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_initMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    rlClientCbs_t   RlApp_ClientCtx;
    int32_t         retVal;
    uint32_t        u32DevIdx;
    uint8_t         u8MasterDevIdx;
    uint8_t         u8SlaveSingleDevMap;

    /* Initialize local variables */
    retVal         = MINUS_ONE;
    u8MasterDevIdx = ptrMMWaveMCB->openCfg.MasterDevIdx;

    /* Initialize the link status: */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        ptrMMWaveMCB->linkStatus[u32DevIdx] = 0U;
    }

    /* Reset the client context: */
    memset ((void *)&RlApp_ClientCtx, 0, sizeof(rlClientCbs_t));

    /* Populate the mmWave link context configuration: */
    RlApp_ClientCtx.ackTimeout = MMWAVE_ACK_TIMEOUT;

    /* Are we allowed to use the CRC Driver? */
    if (ptrMMWaveMCB->initCfg.linkCRCCfg.useCRCDriver == 1U)
    {
        /* Should be in line with configuration sent by MMWave_setMiscConfig(); */
        RlApp_ClientCtx.crcType = RL_CRC_TYPE_16BIT_CCITT;
    }
    else
    {
        /* NO: No CRC is used. */
        RlApp_ClientCtx.crcType = RL_CRC_TYPE_NO_CRC;
    }

    /* Setup the platform on which the mmWave Link executes */
    RlApp_ClientCtx.platform = RL_PLATFORM_HOST;
    RlApp_ClientCtx.arDevType = RL_AR_DEVICETYPE_22XX;

    /* Initialize the Communication Interface API: */
    RlApp_ClientCtx.comIfCb.rlComIfOpen  = &MMWave_spiOpen;
    RlApp_ClientCtx.comIfCb.rlComIfClose = &MMWave_spiClose;
    RlApp_ClientCtx.comIfCb.rlComIfRead  = &MMWave_spiRead;
    RlApp_ClientCtx.comIfCb.rlComIfWrite = &MMWave_spiWrite;

    /* Initialize OSI Mutex Interface */
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexCreate = &MMWave_osalMutexCreate;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexLock   = &MMWave_osalMutexLock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexUnLock = &MMWave_osalMutexUnlock;
    RlApp_ClientCtx.osiCb.mutex.rlOsiMutexDelete = &MMWave_osalMutexDelete;

    /* Initialize OSI Semaphore Interface */
    RlApp_ClientCtx.osiCb.sem.rlOsiSemCreate = &MMWave_osalSemCreate;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemWait   = &MMWave_osalSemWait;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemSignal = &MMWave_osalSemSignal;
    RlApp_ClientCtx.osiCb.sem.rlOsiSemDelete = &MMWave_osalSemDelete;

    /* Initialize OSI Queue Interface */
    RlApp_ClientCtx.osiCb.queue.rlOsiSpawn = &MMWave_spawn;

    /* Initialize OSI Timer Interface */
    RlApp_ClientCtx.timerCb.rlDelay = NULL;

    /* Initialize the CRC Interface */
    RlApp_ClientCtx.crcCb.rlComputeCRC = &MMWave_computeCRC;

    /* Initialize Device Control Interface */
    RlApp_ClientCtx.devCtrlCb.rlDeviceDisable            = &MMWave_disableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceEnable             = &MMWave_enableDevice;
    RlApp_ClientCtx.devCtrlCb.rlDeviceMaskHostIrq        = &MMWave_maskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceUnMaskHostIrq      = &MMWave_unmaskHostIRQ;
    RlApp_ClientCtx.devCtrlCb.rlDeviceWaitIrqStatus      = &MMWave_waitIRQStatus;
    RlApp_ClientCtx.devCtrlCb.rlRegisterInterruptHandler = &MMWave_registerInterruptHandler;

    /* Initialize the Asynchronous Event Handler: */
    RlApp_ClientCtx.eventCb.rlAsyncEvent = &MMWave_asyncEventHandler;

    /* Select the Master only for Power On */
    ptrMMWaveMCB->deviceMap = (uint8_t) ((uint8_t) 1U << u8MasterDevIdx);

    /****************************************************************************************
     * Power On (Master only)
     ****************************************************************************************/
    retVal = (int32_t)rlDevicePowerOn(ptrMMWaveMCB->deviceMap, RlApp_ClientCtx);

    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Unable to initialize and power on the BSS. Encode the error code to account
         * for the subsystem error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Wait the end of the power up (receiving confirmation via asynchronous events) */
#if (AWR_22xx_ES_1_1 == 0U)
    /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
    {
        uint32_t feWaitMs = 0U;
        uint32_t feHighMs = 0U;
        while ((gMMWave_MCB.frontEndMssStatus[u8MasterDevIdx] & (uint32_t) (MMWAVE_FRONT_END_MSS_POWER_UP_DONE | MMWAVE_FRONT_END_MSS_CPU_FAULT)) == 0U)
        {
            if (MMWavePatch_feWaitStep((uint32_t)(u8MasterDevIdx), &feWaitMs, &feHighMs) != 0)
            {
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                retVal   = MINUS_ONE;
                goto exit;
            }
        }
    }

    /* Reset these flags as they will be polled again later on */
    gMMWave_MCB.frontEndMssStatus[u8MasterDevIdx] &= ~ (uint32_t) (MMWAVE_FRONT_END_MSS_POWER_UP_DONE | MMWAVE_FRONT_END_MSS_CPU_FAULT);
#else
    /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
    {
        uint32_t feWaitMs = 0U;
        uint32_t feHighMs = 0U;
        while ((gMMWave_MCB.frontEndMssStatus[u8MasterDevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE) == 0U )
        {
            if (MMWavePatch_feWaitStep((uint32_t)(u8MasterDevIdx), &feWaitMs, &feHighMs) != 0)
            {
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                retVal   = MINUS_ONE;
                goto exit;
            }
        }
    }

    /* Reset these flags as they will be polled again later on */
    gMMWave_MCB.frontEndMssStatus[u8MasterDevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE;

#endif

    /****************************************************************************************
     * Update Firmware (Master only)
     ****************************************************************************************/

#if (FIRMWARE_DOWNLOAD == 1U)
    retVal = MMWave_FirmwareDownload(ptrMMWaveMCB->deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        retVal = MINUS_ONE;
        goto exit;
    }

    /* Wait the error status of AWR22xx MSS (only when booted over SPI). This asynchronous event is received
     *   after the boot-up over SPI is complete and AWR22xx MSS self-tests have been done.
     *
     * NOTE: - Do not send any new command before receiving the boot-up error status asynchronous event.
     *       - Boot-up time is about 5.3 ms. */
    /*while ((ptrMMWaveMCB->frontEndMssStatus[u8MasterDevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_BOOT_ERROR) == 0U)
    {
        ClockP_usleep(1 *1000);
    }*/

    /* Reset the flag*/
    ptrMMWaveMCB->frontEndMssStatus[u8MasterDevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_BOOT_ERROR;
#endif

#if (AWR_22xx_ES_1_1 == 0U)
    /* Execute swap reset*/
    if(MMWave_SwapResetAndPowerOn(ptrMMWaveMCB, errCode) != 0)
    {
        retVal = MINUS_ONE;
        goto exit;
    }
#endif

    /****************************************************************************************
     * RF Power Up = Run Firmware (Master only)
     ****************************************************************************************/
    retVal = rlDeviceRfStart(ptrMMWaveMCB->deviceMap);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFEN, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Wait the end of the RF power up (receiving confirmation via asynchronous event) */
    /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
    {
        uint32_t feWaitMs = 0U;
        uint32_t feHighMs = 0U;
        while ((ptrMMWaveMCB->frontEndMssStatus[u8MasterDevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_RF_POWER_UP_DONE) == 0U)
        {
            if (MMWavePatch_feWaitStep((uint32_t)(u8MasterDevIdx), &feWaitMs, &feHighMs) != 0)
            {
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                retVal   = MINUS_ONE;
                goto exit;
            }
        }
    }

    /* Reset the flag*/
    ptrMMWaveMCB->frontEndMssStatus[u8MasterDevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_RF_POWER_UP_DONE;

    /****************************************************************************************
     * Set the channel configuration (Master only)
     ****************************************************************************************/

    /* Apply cascading Master configuration to the Master device:
     *  - Cascading mode = Multi-chip Master
     *  - Pin-out Master CLKOUT  = 0 = Enabled
     *  - Pin-out Master SYNCOUT = 0 = Enabled
     *  - Pin-out Slave  CLKOUT  = 0 = Disabled (Ignore in Master mode)
     *  - Pin-out Slave  SYNCOUT = 0 = Disabled (Ignore in Master mode)
     *  - Pin-out Master LO      = 0 = Use looped back
     *  - Pin-out OSCCLKOUT      = 0 = Enabled
     *  - Pin-out Master SYNCIN  = 0 = Use looped back
     *
     * After the following command, the Master will output the MCU clock for all
     *   Slaves (OSC_CLKOUT). This clock should be available when Slave devices are
     *   powered up. */

    if(MMWAVE_RADAR_DEVICES > 1)
    {
        ptrMMWaveMCB->openCfg.frontEndCfg[u8MasterDevIdx].chCfg.cascading = (uint16_t) RL_FRAME_SYNC_MODE_MULT_CHIP_MASTER;
    }
    else
    {
        ptrMMWaveMCB->openCfg.frontEndCfg[u8MasterDevIdx].chCfg.cascading = (uint16_t) RL_FRAME_SYNC_MODE_SINGLE_CHIP;
    }

    ptrMMWaveMCB->openCfg.frontEndCfg[u8MasterDevIdx].chCfg.cascadingPinoutCfg = (uint16_t) 0x0000U;

    retVal = rlSetChannelConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.frontEndCfg[u8MasterDevIdx].chCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the channel configuration failed */
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECHCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Select all Slaves and de-select Master
     ****************************************************************************************/
    /* Update the device Map to select all slaves only */
    ptrMMWaveMCB->deviceMap  = (uint8_t) ((uint8_t) 1U << MMWAVE_RADAR_DEVICES) - 1U;
    ptrMMWaveMCB->deviceMap &= ~(uint8_t) ((uint8_t) 1U << u8MasterDevIdx);

    /****************************************************************************************
     * Power On (Slaves only)
     ****************************************************************************************/

    if(ptrMMWaveMCB->deviceMap)
    {
        retVal = rlDeviceAddDevices(ptrMMWaveMCB->deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            retVal   = MINUS_ONE;
            goto exit;
        }

        /* Wait the end of the power up for every Slave (receiving confirmation via asynchronous events) */
        for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
        {
            if(u32DevIdx != (uint32_t) u8MasterDevIdx)
            {
    #if (AWR_22xx_ES_1_1 == 0U)
                /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
                {
                    uint32_t feWaitMs = 0U;
                    uint32_t feHighMs = 0U;
                    while ((gMMWave_MCB.frontEndMssStatus[u32DevIdx] & (uint32_t) (MMWAVE_FRONT_END_MSS_POWER_UP_DONE | MMWAVE_FRONT_END_MSS_CPU_FAULT)) == 0U)
                    {
                        if (MMWavePatch_feWaitStep((uint32_t)(u32DevIdx), &feWaitMs, &feHighMs) != 0)
                        {
                            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                            retVal   = MINUS_ONE;
                            goto exit;
                        }
                    }
                }

                /* Reset these flags as they will be polled again later on */
                gMMWave_MCB.frontEndMssStatus[u32DevIdx] &= ~ (uint32_t) (MMWAVE_FRONT_END_MSS_POWER_UP_DONE | MMWAVE_FRONT_END_MSS_CPU_FAULT);
    #else
                /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
                {
                    uint32_t feWaitMs = 0U;
                    uint32_t feHighMs = 0U;
                    while ((gMMWave_MCB.frontEndMssStatus[u32DevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE) == 0U )
                    {
                        if (MMWavePatch_feWaitStep((uint32_t)(u32DevIdx), &feWaitMs, &feHighMs) != 0)
                        {
                            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                            retVal   = MINUS_ONE;
                            goto exit;
                        }
                    }
                }

                /* Reset these flags as they will be polled again later on */
                gMMWave_MCB.frontEndMssStatus[u32DevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE;
    #endif
            }
        }

        /****************************************************************************************
         * Update Firmware (Slaves only)
         ****************************************************************************************/
    #if (FIRMWARE_DOWNLOAD == 1U)
        retVal = MMWave_FirmwareDownload(ptrMMWaveMCB->deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            retVal = MINUS_ONE;
            goto exit;
        }

        /* Wait the error status of AWR22xx MSS (only when booted over SPI). This asynchronous event is received
        *   after the boot-up over SPI is complete and AWR22xx MSS self-tests have been done.
        *
        * NOTE: - Do not send any new command before receiving the boot-up error status asynchronous event.
        *       - Boot-up time is about 5.3 ms. */
        for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
        {
            if(u32DevIdx != (uint32_t) u8MasterDevIdx)
            {
                /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
                {
                    uint32_t feWaitMs = 0U;
                    uint32_t feHighMs = 0U;
                    while ((ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_BOOT_ERROR) == 0U)
                    {
                        if (MMWavePatch_feWaitStep((uint32_t)(u32DevIdx), &feWaitMs, &feHighMs) != 0)
                        {
                            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                            retVal   = MINUS_ONE;
                            goto exit;
                        }
                    }
                }

                /* Reset the flag*/
                ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_BOOT_ERROR;
            }
        }
    #endif

    #if (AWR_22xx_ES_1_1 == 0U)
        /* Execute swap reset*/
        if(MMWave_SwapResetAndPowerOn(ptrMMWaveMCB, errCode) != 0)
        {
            retVal = MINUS_ONE;
            goto exit;
        }
    #endif

        /****************************************************************************************
         * RF Power Up = Run Firmware (Slaves)
         ****************************************************************************************/
        retVal = rlDeviceRfStart(ptrMMWaveMCB->deviceMap);

        if (retVal != RL_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFEN, retVal);
            retVal   = MINUS_ONE;
            goto exit;
        }

        /* Wait the end of the RF power up for every Slave (receiving confirmation via asynchronous event) */
        for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
        {
            if(u32DevIdx != (uint32_t) u8MasterDevIdx)
            {
                /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
                {
                    uint32_t feWaitMs = 0U;
                    uint32_t feHighMs = 0U;
                    while ((ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_RF_POWER_UP_DONE) == 0U)
                    {
                        if (MMWavePatch_feWaitStep((uint32_t)(u32DevIdx), &feWaitMs, &feHighMs) != 0)
                        {
                            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                            retVal   = MINUS_ONE;
                            goto exit;
                        }
                    }
                }

                /* Reset the flag*/
                ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_RF_POWER_UP_DONE;
            }
        }

        /****************************************************************************************
        * Set the channel configuration (Slaves only)
        ****************************************************************************************/

        /* Apply cascading Slave configuration to Slave devices:
        *  - Cascading mode = Multi-chip Slave
        *  - Pin-out Master CLKOUT  = 0 = Enabled (Ignore in Slave mode)
        *  - Pin-out Master SYNCOUT = 0 = Enabled (Ignore in Slave mode)
        *  - Pin-out Slave  CLKOUT  = 0 = Disabled
        *  - Pin-out Slave  SYNCOUT = 0 = Disabled
        *  - Pin-out Master LO      = 0 = Use looped back (Ignore in Slave mode)
        *  - Pin-out OSCCLKOUT      = 1 = Disabled
        *  - Pin-out Master SYNCIN  = 0 = Use looped back (Ignore in Slave mode)
        */

        for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
        {
            if(u32DevIdx != (uint32_t) u8MasterDevIdx)
            {
                u8SlaveSingleDevMap = (uint8_t) ((uint8_t) 1U << (uint8_t) u32DevIdx);

                ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].chCfg.cascading = (uint16_t) RL_FRAME_SYNC_MODE_MULT_CHIP_SLAVE;
                ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].chCfg.cascadingPinoutCfg = (uint16_t) 0x0020U;

                retVal = rlSetChannelConfig(u8SlaveSingleDevMap, &ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].chCfg);

                if (retVal != RL_RET_CODE_OK)
                {
                /* Error: Set the channel configuration failed */
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECHCFG, retVal);
                retVal   = MINUS_ONE;
                goto exit;
                }
            }
        }
    }

    /* At this point all devices (Master and Slaves) are powered up.
     * Device map can be updated to select all devices. */
    gMMWave_MCB.deviceMap  = (uint8_t) ((uint8_t) 1U << MMWAVE_RADAR_DEVICES) - 1U;

    /* Link has been setup successfully. */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in] ptrCalibrationData
 *      Optional pointer to the calibration data which needs to be
 *      restored.
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_openLink
(
    MMWave_MCB*                 ptrMMWaveMCB,
    MMWave_CalibrationData*     ptrCalibrationData,
    int32_t*                    errCode
)
{
    int32_t                     retVal = MINUS_ONE;
    uint32_t                    u32DevIdx;
    uint32_t                    masterDevIdx;
    rlRfCalMonFreqLimitConf_t   freqLimit;
    rlRfInitCalConf_t           rfInitCalib;
    rlRfCalMonTimeUntConf_t     timeCfg;
    rlRfLdoBypassCfg_t          stRFLdoBypassCfg;
    rlRfDevCfg_t                asyncEventCfg;
    rlDevDataFmtCfg_t           stDataFmtCfg;

    /* Initialize local variables */
    masterDevIdx = ptrMMWaveMCB->openCfg.MasterDevIdx;

    /****************************************************************************************
     * Configure the SPI IRQ GPIO:
     ****************************************************************************************/
    MMWave_configSpiGpio(ptrMMWaveMCB);

    /****************************************************************************************
     * Configure the NRESET GPIO:
     ****************************************************************************************/
    MMWave_configNresetGpio(ptrMMWaveMCB);

    /****************************************************************************************
     * Boot all devices:
     ****************************************************************************************/
    /* SPI and NRESET must be configured before this function is called*/
    retVal = MMWave_initMMWaveLink(ptrMMWaveMCB, errCode);

    if (retVal != 0)
    {
        retVal = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Get version for all devices:
     ****************************************************************************************/
    /* Getting the version of each device is a way to check that each device's firmware
     * is running.
     * Note: only data from last device is saved.
     */
    retVal = MMWave_deviceGetVersion(ptrMMWaveMCB, errCode);

    if(retVal != RL_RET_CODE_OK)
    {
        retVal = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Set the CRC type for asynchronous messages:
     ****************************************************************************************/
    retVal = MMWave_setMiscConfig(ptrMMWaveMCB, errCode);

    if(retVal != RL_RET_CODE_OK)
    {
        retVal = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Set the ADC Output configuration:
     ****************************************************************************************/
    retVal = rlSetAdcOutConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.adcOutCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the ADC configuration failed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EADCCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Set the LDO Bypass configuration:
     ****************************************************************************************/
    /* LDO Bypass configuration:
     * - RF1 = RF2 = 1.0V
     * - VOUT_PA shorted to RF1 */
    stRFLdoBypassCfg.ldoBypassEnable   = 3U;    /* 1.0V RF1 and 1.0V RF2 and RF1 shorted to VOUT_PA */
    stRFLdoBypassCfg.supplyMonIrDrop   = 1U;    /* IR drop of 3%                                    */
    stRFLdoBypassCfg.ioSupplyIndicator = 0U;    /* 3.3V IO supply                                   */

    retVal = rlRfSetLdoBypassConfig(ptrMMWaveMCB->deviceMap, (rlRfLdoBypassCfg_t*) &stRFLdoBypassCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        retVal = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Set the low power mode configuration:
     ****************************************************************************************/
    retVal = rlSetLowPowerModeConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.lowPowerMode);

    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Set the Low power configuration failed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EPOWERCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Link is not operational: */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        ptrMMWaveMCB->linkStatus[u32DevIdx] = 0U;
    }

    /****************************************************************************************
     * Setup the RF Calibration Frequency limit:
     ****************************************************************************************/
    freqLimit.freqLimitLow  = ptrMMWaveMCB->openCfg.freqLimitLow;
    freqLimit.freqLimitHigh = ptrMMWaveMCB->openCfg.freqLimitHigh;
    freqLimit.reserved0     = 0U;
    freqLimit.reserved1     = 0U;

    retVal = rlRfSetCalMonFreqLimitConfig(ptrMMWaveMCB->deviceMap, &freqLimit);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Asynchronous event configuration:
     ****************************************************************************************/
    memset ((void*)&asyncEventCfg, 0, sizeof(rlRfDevCfg_t));

    /* Set the direction of the "Global Asynchronous events" and "Monitoring Asynchronous events" to be "radarSS to HOST" */
    asyncEventCfg.aeDirection = 0x5U;

    /*Disable Frame Start Asynchronous event? */
    if(ptrMMWaveMCB->openCfg.disableFrameStartAsyncEvent)
    {
        asyncEventCfg.aeControl |= 0x1U;
    }

    /*Disable Frame Stop Asynchronous event? */
    if(ptrMMWaveMCB->openCfg.disableFrameStopAsyncEvent)
    {
        asyncEventCfg.aeControl |= 0x2U;
    }

    retVal = rlRfSetDeviceCfg(ptrMMWaveMCB->deviceMap, (rlRfDevCfg_t*) &asyncEventCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EASYNCEVENT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Configure the format of ADC samples over CSI2/LVDS:
     ****************************************************************************************/
    /* It is assume that all devices will use the same number of Rx channels than the master. */
    stDataFmtCfg.rxChannelEn  = ptrMMWaveMCB->openCfg.frontEndCfg[masterDevIdx].chCfg.rxChannelEn;
    stDataFmtCfg.adcBits      = ptrMMWaveMCB->openCfg.adcOutCfg.fmt.b2AdcBits;

    /* NOTE: adcFmt can take values 0 (Real) or 1(Complex) only.
             b2AdcOutFmt can take 4 different values 0(Real), 1(Complex), 2(Complex with Image band) or 3(Pseudo Real)*/
    if(ptrMMWaveMCB->openCfg.adcOutCfg.fmt.b2AdcOutFmt == 0U)
    {
        stDataFmtCfg.adcFmt = 0U;
    }
    else
    {
        stDataFmtCfg.adcFmt = 1U;
    }

    stDataFmtCfg.iqSwapSel    = ptrMMWaveMCB->openCfg.iqSwapSel;      /* Should be the same as ADC buff configuration */
    stDataFmtCfg.chInterleave = ptrMMWaveMCB->openCfg.chInterleave;   /* Should be the same as ADC buff configuration */
    stDataFmtCfg.reserved     = 0U;

    retVal = rlDeviceSetDataFmtConfig(ptrMMWaveMCB->deviceMap, &stDataFmtCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EDATAFMTCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Configure data exported over CSI2/LVDS lines:
     ****************************************************************************************/
    retVal = rlDeviceSetDataPathConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.dataPathCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EDATAPATHCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Enable data lines of CSI2/LVDS communication:
     ****************************************************************************************/
    retVal = rlDeviceSetLaneConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.laneEnCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ELANECFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Configure clock mode and data rate of the CSI2/LVDS communication:
     ****************************************************************************************/
    retVal = rlDeviceSetDataPathClkConfig(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.dataPathClkCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EDATAPATHCLOCKCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Configure the high speed interface input clock rate:
     ****************************************************************************************/
    retVal = rlDeviceSetHsiClk(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.hsiClkCfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EHSICLKCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Configure data/clock lines of CSI2 communication:
     ****************************************************************************************/
    retVal = rlDeviceSetCsi2Config(ptrMMWaveMCB->deviceMap, &ptrMMWaveMCB->openCfg.csi2Cfg);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECSI2CFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Restore calibration data (If needed):
     ****************************************************************************************/
    if (ptrCalibrationData != NULL)
    {
        if (ptrCalibrationData->ptrCalibData != NULL)
        {
            /* YES: Restore the calibration data */
            retVal = rlRfCalibDataRestore(ptrMMWaveMCB->deviceMap, ptrCalibrationData->ptrCalibData);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Calibration data restore failed */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALFAIL, retVal);
                retVal   = MINUS_ONE;
                goto exit;
            }

            /* As per the Link documentation; radarSS will consume the data and will send an
             * asynchronous message to indicate the result of the calibration based on the
             * calibration data which has been restored. So we need to wait on the status of
             * the operation. */
            MMWave_internalLinkSync(ptrMMWaveMCB, MMWAVE_RF_INITIALIZED | MMWAVE_RF_INIT_FAILED);

            /* Was the calibration restored successfully? */
            for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
            {
                if (ptrMMWaveMCB->linkStatus[u32DevIdx] & MMWAVE_RF_INIT_FAILED)
                {
                    /* NO: Calibration was not restored. The asynchronous message has been
                     * passed to the application too. Setup the error code */
                    *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALFAIL, 0);
                    retVal = MINUS_ONE;
                    goto exit;
                }
                else
                {
                    /* Calibration was restored successfully so we need to reset the link operational
                     * status again. */
                    ptrMMWaveMCB->linkStatus[u32DevIdx] = 0U;
                }
            }
        }

        if (ptrCalibrationData->ptrPhaseShiftCalibData != NULL)
        {
            /* User should have set txIndex equal to PhShiftcalibChunk array index when calling
               rlRfPhShiftCalibDataStore() so that data for all Tx are obtained regardless of
               channels enabled in chCfg. However, phase shift data will be non-zero only
               for Tx channels that are enabled in chCfg */
            if ((ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[0].txIndex != 0) ||
                (ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[1].txIndex != 1) ||
                (ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[2].txIndex != 2))
            {
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, 0);
                retVal   = MINUS_ONE;
                goto exit;
            }
            /* User should not set calibApply to 1 for any Tx. rlRfPhShiftCalibDataRestore() will take care of
               it internally */
            if ((ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[0].calibApply != 0) ||
                (ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[1].calibApply != 0) ||
                (ptrCalibrationData->ptrPhaseShiftCalibData->PhShiftcalibChunk[2].calibApply != 0))
            {
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, 0);
                retVal   = MINUS_ONE;
                goto exit;
            }
            /* Basic validation passed: Restore the phase shift calibration data */
            retVal = rlRfPhShiftCalibDataRestore(ptrMMWaveMCB->deviceMap, ptrCalibrationData->ptrPhaseShiftCalibData);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Phase shift Calibration data restore failed */
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EPHASESHIFTCALDATARESTOREFAIL, retVal);
                retVal   = MINUS_ONE;
                goto exit;
            }
        }
    }

    /****************************************************************************************
     * Initialize the RF Calibration configuration:
     ****************************************************************************************/
    memset ((void *) &rfInitCalib, 0, sizeof(rlRfInitCalConf_t));

    if (ptrMMWaveMCB->openCfg.useCustomCalibration == true)
    {
        /* Custom Calibration: Use the application provided calibration mask */
        rfInitCalib.calibEnMask = ptrMMWaveMCB->openCfg.customCalibrationEnableMask;
    }
    else
    {
        /* Default: Enable all calibrations */
        rfInitCalib.calibEnMask = CSL_FMKR (4U, 4U, 1U)     |   /* LODIST calibration            */
                                  CSL_FMKR (5U, 5U, 1U)     |   /* RX ADC DC offset calibration  */
                                  CSL_FMKR (6U, 6U, 1U)     |   /* HPF cutoff calibration        */
                                  CSL_FMKR (7U, 7U, 1U)     |   /* LPF cutoff calibration        */
                                  CSL_FMKR (8U, 8U, 1U)     |   /* Peak detector calibration     */
                                  CSL_FMKR (9U, 9U, 1U)     |   /* TX Power calibration          */
                                  CSL_FMKR (10U, 10U, 1U)   |   /* RX gain calibration           */
                                  CSL_FMKR (11U, 11U, 1U)   |   /* TX phase calibration          */
                                  CSL_FMKR (12U, 12U, 1U);      /* RX IQMM calibration           */
    }

    retVal = rlRfInitCalibConfig(ptrMMWaveMCB->deviceMap, &rfInitCalib);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /****************************************************************************************
     * Initialize the RF:
     ****************************************************************************************/
    DebugP_logInfo ("rlRfInit...\n");
    retVal = rlRfInit(ptrMMWaveMCB->deviceMap);

    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* SYNCHRONIZATION: We need to loop around till the Front-end has performed the
     * RF initialization.
     */
    MMWave_internalLinkSync(ptrMMWaveMCB, (uint32_t) (MMWAVE_RF_INITIALIZED | MMWAVE_RF_INIT_FAILED));

    /* Was the RF Initialized successfully? */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        if (ptrMMWaveMCB->linkStatus[u32DevIdx] & MMWAVE_RF_INIT_FAILED)
        {
            /* NO: The RF initialization failed. The asynchronous event was passed to the application
             * The application can decode the exact reason for the failure. We simply set the error
             * code and report failure */
            *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECALFAIL, 0);
            retVal = MINUS_ONE;
            goto exit;
        }
    }

    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     ****************************************************************************************/
    if(ptrMMWaveMCB->openCfg.calibMonTimeUnit != 0U)
    {
        memset ((void *) &timeCfg, 0, sizeof(rlRfCalMonTimeUntConf_t));

        timeCfg.numOfCascadeDev  = 1U;
        timeCfg.devId            = 0U;
        timeCfg.monitoringMode   = 1U;
        timeCfg.calibMonTimeUnit = ptrMMWaveMCB->openCfg.calibMonTimeUnit;

        retVal = rlRfSetCalMonTimeUnitConfig(ptrMMWaveMCB->deviceMap, &timeCfg);

        if (retVal != RL_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALPERIOD, retVal);
            retVal   = MINUS_ONE;
            goto exit;
        }
    }
    else
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVALIDCALMONUNIT, 0);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* No error - Update the return value */
    retVal = 0;

exit:
    return retVal;
}


#if (AWR_22xx_ES_1_1 == 0U)

/**
 *  @b Description
 *  @n
 *      The function is used to do internal BSS configuration
 *      swap memory, reset and wait for power ON.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_SwapResetAndPowerOn(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t retVal = RL_RET_CODE_OK;
    uint32_t u32DevIdx;
    uint8_t  u8SingleDevMap;

	/* Disable MSS Watchdog */
	retVal = rlDeviceSetInternalConf(ptrMMWaveMCB->deviceMap, 0xFFFFFF0C, 0x000000AD);
    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINTERNALCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Swap RAM memory map with ROM memory map  */
    retVal = rlDeviceSetInternalConf(ptrMMWaveMCB->deviceMap, 0xFFFFFF20, 0x00ADAD00);
    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINTERNALCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Disable the Ack*/
    rlDeviceConfigureAckTimeout(0);

    /* reset the core */
    retVal = rlDeviceSetInternalConf(ptrMMWaveMCB->deviceMap, 0xFFFFFF04, 0x000000AD);
    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINTERNALCFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Enable the Ack*/
    rlDeviceConfigureAckTimeout(1000U);

    /* Wait for Power ON complete */
    if (0 == retVal)
    {
        for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
        {
            /* Get the device map value for the selected device */
            u8SingleDevMap = (uint8_t) ((uint8_t) 1U << (uint8_t) u32DevIdx);

            if((uint8_t) (ptrMMWaveMCB->deviceMap & u8SingleDevMap) != 0U)
            {
                /* CHIRP PATCH: bounded wait (TI waited forever; see mmwave_ctrl_patch.h) */
                {
                    uint32_t feWaitMs = 0U;
                    uint32_t feHighMs = 0U;
                    while ((ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] & (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE) == 0U)
                    {
                        if (MMWavePatch_feWaitStep((uint32_t)(u32DevIdx), &feWaitMs, &feHighMs) != 0)
                        {
                            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, 0);
                            retVal   = MINUS_ONE;
                            goto exit;
                        }
                    }
                }

                /* Reset the flag*/
                ptrMMWaveMCB->frontEndMssStatus[u32DevIdx] &= ~ (uint32_t) MMWAVE_FRONT_END_MSS_POWER_UP_DONE;
            }
        }



    }

    exit:
    return retVal;
}
#endif
/**
 *  @b Description
 *  @n
 *      The function is used to configure the NRESET GPIO
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void MMWave_configNresetGpio (const MMWave_MCB* ptrMMWaveMCB)
{
    uint32_t u32DevIdx;

    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        DebugP_logInfo("NRESET GPIO configured index %d \n",ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex);
        GPIO_pinWriteHigh(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                        ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex);

        GPIO_setDirMode(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                        ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].nresetGpioIndex, GPIO_DIRECTION_OUTPUT);
    }
}


/**
 *  @b Description
 *  @n
 *      The function is used to configure the GPIO used for the
 *      SPI IRQ. This GPIO should trigger an interrupt only when
 *      the IRQ is high (no interrupt on IRQ low).
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void MMWave_configSpiGpio (const MMWave_MCB* ptrMMWaveMCB)
{
    HwiP_Params     hwiPrms;
    int32_t         retVal;
    uint32_t        u32DevIdx;

    /* HOST_IRQ GPIO should trigger an interrupt only when the IRQ is high.
       This is the mmwavelink assumption .*/
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_RADAR_DEVICES; u32DevIdx++)
    {
        DebugP_logInfo("IRQ GPIO configured index %d \n", ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex);

        /* Configure GPIO. This GPIO should trigger an interrupt only when the IRQ is high.
        This is the mmwavelink assumption.*/
        GPIO_setDirMode(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                        ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex, GPIO_DIRECTION_INPUT);

        GPIO_setTrigType(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                        ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex,
                        GPIO_TRIG_TYPE_RISE_EDGE);

        GPIO_markHighLowLevelInterrupt(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                                    ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex,
                                    GPIO_INTR_LEVEL_HIGH);

        GPIO_clearInterrupt(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                            ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex);

        GPIO_enableInterrupt(ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].gpioBaseAddr,
                            ptrMMWaveMCB->openCfg.frontEndCfg[u32DevIdx].spiIrqGpioIndex);
    }

    /* CHIRP PATCH: MMWave_open can now run more than once (front-end fault
     * recovery). Destruct the previous SPI host-IRQ Hwi before reconstructing it. */
    if (gGpioHwiConstructed != 0U)
    {
        HwiP_destruct(&gGpioHwiObject[0]);
    }
    gGpioHwiConstructed = 1U;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = ptrMMWaveMCB->openCfg.frontEndCfg[0].gpioPinIntrNum;
    hwiPrms.callback = &MMWave_gpioSpiIrqDevCallbackFxn;
    hwiPrms.args = (void *) ptrMMWaveMCB->openCfg.frontEndCfg[0].spiIrqGpioIndex;
    retVal = HwiP_construct(&gGpioHwiObject[0], &hwiPrms);
    if(SystemP_SUCCESS != retVal)
    {
        DebugP_assert(FALSE);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_initLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t                 retVal = 0;
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uint32_t                index;

    /* Initialize and setup the spawn lists */
    for (index = 0U; index < MMWAVE_MAX_NUM_SPAWN_LIST; index++)
    {
        /* Get the pointer to the spawn node */
        ptrSpawnFxnNode = &ptrMMWaveMCB->spawnTable[index];

        /* Initialize the spawn node */
        memset ((void*)ptrSpawnFxnNode, 0, sizeof(MMWave_SpawnFxnNode));

        /* Add the node to the free list: */
        MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);
    }

    /* Reset the active list: */
    ptrMMWaveMCB->ptrSpawnFxnActiveList = NULL;

    /* Setup and Initialize the CRC Driver: */
    if (MMWave_initCRC (ptrMMWaveMCB, errCode) == 0)
    {
        retVal = 0;
    }
    else
    {
        /* Error: Unable to setup the CRC Driver */
        DebugP_logInfo("CRC init failed\n");
        retVal = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set CRC type for async event
 *      message sent by MSS to Host.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_setMiscConfig(const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t retVal = RL_RET_CODE_OK;
    rlDevMiscCfg_t devMiscCfg;

    /* Configure 16 bit CRC */
    memset(&devMiscCfg, 0 , sizeof(devMiscCfg));

    devMiscCfg.aeCrcConfig = RL_CRC_TYPE_16BIT_CCITT;

    DebugP_logInfo ("rlDeviceSetMiscConfig...\n");
    retVal = rlDeviceSetMiscConfig(ptrMMWaveMCB->deviceMap, &devMiscCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EMISCCFG, retVal);
        DebugP_logInfo ("Error: rlDeviceSetMiscConfig %d.\n",retVal);
        retVal   = MINUS_ONE;
    }

    return retVal;
}
