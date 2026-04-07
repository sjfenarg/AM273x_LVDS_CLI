/*
 *   @file  app_cli_state.h
 *
 *   @brief
 *      Runtime CLI state and configuration for the Cascade LVDS application.
 *      Shared between main_mss.c and common.c.
 *
 *  Copyright (C) 2024 — CLI extension for cascade LVDS capture.
 */

#ifndef APP_CLI_STATE_H
#define APP_CLI_STATE_H

#include <stdint.h>
#include <ti/control/mmwavelink/mmwavelink.h>

/**************************************************************************
 *************************** CLI State Enum *******************************
 **************************************************************************/
typedef enum {
    CLI_STATE_INIT = 0,
    CLI_STATE_WAITING_FOR_COMMAND,
    CLI_STATE_START_REQUESTED,
    CLI_STATE_RUNNING_CAPTURE,
    CLI_STATE_CAPTURE_DONE,
    CLI_STATE_ERROR
} CLI_State;

/**************************************************************************
 *************************** CLI Config Structures ************************
 **************************************************************************/
typedef struct {
    /* channelCfg */
    uint8_t  rxMask;          /* 8-bit user mask [1, 0xFF] */
    uint8_t  txMask;          /* 6-bit user mask [1, 0x3F] */
    
    /* setTDM */
    uint8_t  tdmEnabled;      /* 0 or 1 */
    
    /* adcCfg */
    uint8_t  adcBits;         /* 0=12, 1=14, 2=16 */
    uint8_t  adcFmt;          /* 0=real, 1=cpx_1x, 2=cpx_2x */
    
    /* chirp header */
    uint8_t  chirpHeaderEnabled; /* 0=off (default), 1=on */
    
    /* frameCfg - full mmWave link frame struct */
    rlFrameCfg_t frame;
    
    /* profileCfg - full mmWave link profile struct */
    rlProfileCfg_t profile;

    /* Startup-frame discard: skip the first N frames' CBUFF/LVDS triggers
     * to protect against known-bad startup chirps.  0 = disabled. */
    uint16_t startupDiscardFrames;
} CLI_CaptureConfig;


/**************************************************************************
 *************************** CLI Externs ***********************************
 **************************************************************************/

/** Current application state (written by init task and CLI task). */
extern volatile CLI_State gCliState;

/**
 * Configured settings (written when idle).
 * Replaces gConfiguredNumFrames.
 */
extern CLI_CaptureConfig gCfg;

/**
 * Latched settings (snapshot at sensorStart).
 * RF configuration and datapath use this exclusively during capture.
 * Replaces gActiveNumFrames.
 */
extern CLI_CaptureConfig gActive;

#endif /* APP_CLI_STATE_H */
