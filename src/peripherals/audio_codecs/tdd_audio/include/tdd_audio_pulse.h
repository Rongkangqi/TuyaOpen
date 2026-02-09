/**
 * @file tdd_audio_pulse.h
 * @brief Tuya Device Driver layer audio interface for PulseAudio.
 *
 * This file defines the device driver interface for audio functionality using PulseAudio
 * on Linux/Ubuntu platforms. It provides structures and functions for configuring
 * and registering PulseAudio audio devices with support for various audio parameters
 * including sample rates, data bits, channels, and buffer configuration.
 *
 * The TDD (Tuya Device Driver) layer acts as an abstraction between PulseAudio-specific
 * implementations and the higher-level TDL (Tuya Driver Layer) audio management system.
 *
 * This driver is only available when ENABLE_AUDIO_PULSE is enabled and the Ubuntu
 * platform is selected.
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __TDD_AUDIO_PULSE_H__
#define __TDD_AUDIO_PULSE_H__

#include "tuya_cloud_types.h"

#if defined(ENABLE_AUDIO_PULSE) && (ENABLE_AUDIO_PULSE == 1)
#include "tdl_audio_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************macro define************************
***********************************************************/

// PulseAudio audio data bits
typedef enum {
    TDD_PULSE_DATABITS_8 = 8,      /**< 8-bit unsigned integer */
    TDD_PULSE_DATABITS_16 = 16,    /**< 16-bit signed integer, little-endian */
    TDD_PULSE_DATABITS_24 = 24,    /**< 24-bit signed integer, little-endian */
    TDD_PULSE_DATABITS_32 = 32,    /**< 32-bit signed integer, little-endian */
    TDD_PULSE_DATABITS_FLOAT = 33, /**< 32-bit float, little-endian */
} TDD_PULSE_DATABITS_E;

/***********************************************************
***********************typedef define***********************
***********************************************************/
/**
 * @brief PulseAudio audio configuration structure
 *
 * This structure contains all the configuration parameters needed to initialize
 * a PulseAudio audio device for both capture (microphone) and playback (speaker).
 * PulseAudio automatically handles format conversion and device routing.
 */
typedef struct {
    // Application identification
    char *app_name;                    /**< Application name for PulseAudio (shown in audio mixer) */
    
    // Capture (microphone) settings
    char *capture_device;              /**< PulseAudio capture device name (e.g., "alsa_input.platform-es8388-sound.stereo-fallback", NULL for default) */
    uint32_t sample_rate;              /**< Audio sample rate in Hz (e.g., 16000, 44100) */
    TDD_PULSE_DATABITS_E data_bits;    /**< Number of bits per sample */
    uint8_t channels;                  /**< Number of audio channels (1=mono, 2=stereo) */

    // Playback (speaker) settings
    char *playback_device;             /**< PulseAudio playback device name (e.g., "alsa_output.platform-es8388-sound.stereo-fallback", NULL for default) */
    uint32_t spk_sample_rate;          /**< Speaker sample rate in Hz */

    // Buffer settings
    uint32_t buffer_frames;            /**< Buffer size in frames (used for internal buffering) */
    uint32_t period_frames;            /**< Period size in frames (processing block size) */
} TDD_AUDIO_PULSE_CFG_T;

/***********************************************************
***********************typedef define***********************
***********************************************************/
/**
 * @brief Audio Front-End (AFE) feed callback function type
 * 
 * This callback is used to feed audio data to Voice Activity Detection (VAD)
 * or Keyword Spotting (KWS) engines.
 * 
 * @param[in] mic_data  Microphone captured audio data (PCM int16)
 * @param[in] ref_data  Reference audio data, typically from speaker playback (can be NULL)
 * @param[in] out_data  Processed output data from AEC or other processing (can be NULL)
 * @param[in] frames    Number of audio frames in the buffers
 */
typedef void (*TDD_AUDIO_AFE_FEED_CB)(int16_t *mic_data, int16_t *ref_data, int16_t *out_data, uint32_t frames);

/***********************************************************
********************function declaration********************
***********************************************************/

/**
 * @brief Register a PulseAudio audio driver with the TDL audio management system
 *
 * This function creates and registers a PulseAudio audio driver instance with the
 * specified configuration. The driver will be available for use by applications
 * through the TDL audio management API.
 *
 * @param[in] name      Driver name (used to identify this audio device)
 * @param[in] cfg       PulseAudio configuration parameters
 *
 * @return OPERATE_RET
 * @retval OPRT_OK              Success
 * @retval OPRT_MALLOC_FAILED   Memory allocation failed
 * @retval OPRT_COM_ERROR       Registration failed
 */
OPERATE_RET tdd_audio_pulse_register(char *name, TDD_AUDIO_PULSE_CFG_T cfg);

/**
 * @brief Register a callback function for VAD (Voice Activity Detection) audio feed
 * 
 * The registered callback will be called with captured audio data during recording.
 * This allows VAD processing to run in parallel with normal audio capture.
 *
 * @param[in] cb    Callback function pointer, or NULL to clear the callback
 */
void tdd_audio_pulse_register_vad_feed_cb(TDD_AUDIO_AFE_FEED_CB cb);

/**
 * @brief Register a callback function for KWS (Keyword Spotting) audio feed
 * 
 * The registered callback will be called with captured audio data during recording.
 * This allows KWS processing to run in parallel with normal audio capture.
 *
 * @param[in] cb    Callback function pointer, or NULL to clear the callback
 */
void tdd_audio_pulse_register_kws_feed_cb(TDD_AUDIO_AFE_FEED_CB cb);

/**
 * @brief Unregister the VAD audio feed callback
 */
void tdd_audio_pulse_unregister_vad_feed_cb(void);

/**
 * @brief Unregister the KWS audio feed callback
 */
void tdd_audio_pulse_unregister_kws_feed_cb(void);

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_AUDIO_PULSE */

#endif /* __TDD_AUDIO_PULSE_H__ */