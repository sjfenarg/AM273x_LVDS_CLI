/*
 *   @file  chirp_header.h
 *
 *   @brief
 *      Per-chirp metadata header for LVDS stream synchronization.
 *      When enabled, a 16-byte header is appended to each CBUFF
 *      transfer so downstream tools can identify frame/chirp index
 *      and detect gaps in the captured data.
 */

#ifndef CHIRP_HEADER_H
#define CHIRP_HEADER_H

#include <stdint.h>

#define CHIRP_HDR_MAGIC         (0xA5C3U)
#define CHIRP_HDR_VERSION       (1U)
#define CHIRP_HDR_SIZE_BYTES    (16U)

typedef struct __attribute__((packed)) {
    uint16_t magic;            /* 0xA5C3 — sync word for frame detection     */
    uint8_t  version;          /* Header format version (1)                  */
    uint8_t  flags;            /* Bit 0: TDM active, Bit 1: complex ADC     */
    uint16_t frameIdx;         /* Frame index (0-based, wraps at 65535)      */
    uint16_t chirpIdx;         /* Chirp index within frame (0-based)         */
    uint32_t globalChirpIdx;   /* Monotonic chirp counter since capture start*/
    uint32_t timestampTicks;   /* FreeRTOS tick at trigger time              */
} ChirpHeader_t;

_Static_assert(sizeof(ChirpHeader_t) == CHIRP_HDR_SIZE_BYTES,
               "ChirpHeader_t must be exactly 16 bytes");

#endif /* CHIRP_HEADER_H */
