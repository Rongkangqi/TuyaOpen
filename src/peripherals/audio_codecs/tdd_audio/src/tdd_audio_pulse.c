/**
 * @file tdd_audio_pulse.c
 * @brief Implementation of Tuya Device Driver layer audio interface for PulseAudio.
 *
 * This file implements the device driver interface for audio functionality using
 * PulseAudio on Linux/Ubuntu platforms. It provides the implementation for 
 * audio device initialization, configuration, and audio data handling.
 *
 * Key functionalities include:
 * - PulseAudio device registration and initialization
 * - Microphone data capture with threaded callback mechanism
 * - Speaker playback with PulseAudio simple API
 * - Volume control through PulseAudio/PipeWire
 * - Frame-based audio data processing
 * - Proper resource cleanup and error handling
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 *
 */

#include "tuya_cloud_types.h"

#if defined(ENABLE_AUDIO_PULSE) && (ENABLE_AUDIO_PULSE == 1)

#include <pulse/simple.h>
#include <pulse/error.h>
#include <pthread.h>

#include "tal_log.h"
#include "tal_memory.h"
#include "tal_thread.h"

#include "tdd_audio_pulse.h"

/***********************************************************
************************macro define************************
***********************************************************/
#define AUDIO_PCM_FRAME_MS       10
#define PULSE_CAPTURE_THREAD_STACK_SIZE (4096)
#define PULSE_CAPTURE_THREAD_PRIORITY   (THREAD_PRIO_2)

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef struct {
    TDD_AUDIO_PULSE_CFG_T cfg;
    TDL_AUDIO_MIC_CB mic_cb;

    // PulseAudio handles
    pa_simple *capture_handle;
    pa_simple *playback_handle;

    // Capture thread
    pthread_t capture_thread;
    volatile uint8_t capture_running;

    // Thread synchronization for ref_buffer
    pthread_mutex_t ref_buffer_mutex;
    // Thread synchronization for playback
    pthread_mutex_t playback_mutex;

    // Playback settings
    uint8_t play_volume;

    // Buffer
    uint8_t *capture_buffer;
    uint32_t capture_buffer_size;

    uint8_t *ref_buffer;
    uint32_t ref_buffer_size;
    uint8_t *out_buffer;
    uint32_t out_buffer_size;

    // PulseAudio sample spec
    pa_sample_spec sample_spec;
} TDD_AUDIO_PULSE_HANDLE_T;

/***********************************************************
********************function declaration********************
***********************************************************/
static void *__pulse_capture_thread(void *arg);
static OPERATE_RET __pulse_setup_capture(TDD_AUDIO_PULSE_HANDLE_T *hdl);
static OPERATE_RET __pulse_setup_playback(TDD_AUDIO_PULSE_HANDLE_T *hdl);

// Public API for VAD/KWS callback registration
void tdd_audio_pulse_register_vad_feed_cb(TDD_AUDIO_AFE_FEED_CB cb);
void tdd_audio_pulse_register_kws_feed_cb(TDD_AUDIO_AFE_FEED_CB cb);
void tdd_audio_pulse_unregister_vad_feed_cb(void);
void tdd_audio_pulse_unregister_kws_feed_cb(void);

/***********************************************************
***********************variable define**********************
***********************************************************/
static TDD_AUDIO_AFE_FEED_CB __s_vad_feed_cb = NULL;
static TDD_AUDIO_AFE_FEED_CB __s_kws_feed_cb = NULL;

/***********************************************************
***********************function define**********************
***********************************************************/

/**
 * @brief Convert bits per sample to PulseAudio format
 */
static pa_sample_format_t __get_pulse_format(TDD_PULSE_DATABITS_E bits)
{
    switch (bits) {
    case TDD_PULSE_DATABITS_8:
        return PA_SAMPLE_U8;
    case TDD_PULSE_DATABITS_16:
        return PA_SAMPLE_S16LE;
    case TDD_PULSE_DATABITS_24:
        return PA_SAMPLE_S24LE;
    case TDD_PULSE_DATABITS_32:
        return PA_SAMPLE_S32LE;
    case TDD_PULSE_DATABITS_FLOAT:
        return PA_SAMPLE_FLOAT32LE;
    default:
        return PA_SAMPLE_S16LE;
    }
}

/**
 * @brief Calculate buffer size based on configuration
 */
static uint32_t __calculate_buffer_size(TDD_AUDIO_PULSE_HANDLE_T *hdl)
{
    uint32_t bytes_per_sample = 0;
    
    switch (hdl->cfg.data_bits) {
    case TDD_PULSE_DATABITS_8:
        bytes_per_sample = 1;
        break;
    case TDD_PULSE_DATABITS_16:
        bytes_per_sample = 2;
        break;
    case TDD_PULSE_DATABITS_24:
        bytes_per_sample = 3;
        break;
    case TDD_PULSE_DATABITS_32:
    case TDD_PULSE_DATABITS_FLOAT:
        bytes_per_sample = 4;
        break;
    default:
        bytes_per_sample = 2;
        break;
    }
    
    return hdl->cfg.period_frames * hdl->cfg.channels * bytes_per_sample;
}

/**
 * @brief Setup PulseAudio capture device
 */
static OPERATE_RET __pulse_setup_capture(TDD_AUDIO_PULSE_HANDLE_T *hdl)
{
    int error;
    
    // Setup sample specification
    hdl->sample_spec.format = __get_pulse_format(hdl->cfg.data_bits);
    hdl->sample_spec.rate = hdl->cfg.sample_rate;
    hdl->sample_spec.channels = hdl->cfg.channels;
    
    // Setup buffer attributes
    pa_buffer_attr ba = {
        .maxlength = (uint32_t)-1,
        .tlength = (uint32_t)-1,
        .prebuf = (uint32_t)-1,
        .minreq = (uint32_t)-1,
        .fragsize = __calculate_buffer_size(hdl)
    };
    
    // Open PulseAudio capture device
    hdl->capture_handle = pa_simple_new(
        NULL,                   // Use default PulseAudio server
        hdl->cfg.app_name,      // Application name
        PA_STREAM_RECORD,       // Stream direction
        hdl->cfg.capture_device, // Device name (NULL for default)
        "Tuya Audio Capture",   // Stream description
        &hdl->sample_spec,      // Sample specification
        NULL,                   // Channel map (NULL for default)
        &ba,                    // Buffer attributes
        &error                  // Error code
    );
    
    if (!hdl->capture_handle) {
        PR_WARN("PulseAudio capture device '%s' not available: %s", 
                hdl->cfg.capture_device ? hdl->cfg.capture_device : "default",
                pa_strerror(error));
        return OPRT_COM_ERROR;
    }
    
    // Calculate buffer size
    hdl->capture_buffer_size = __calculate_buffer_size(hdl);
    hdl->ref_buffer_size = hdl->capture_buffer_size;
    hdl->out_buffer_size = hdl->capture_buffer_size;
    
    // Allocate reference buffer
    hdl->ref_buffer = (uint8_t *)tal_malloc(hdl->ref_buffer_size);
    if (NULL == hdl->ref_buffer) {
        PR_ERR("Cannot allocate reference buffer");
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
        return OPRT_MALLOC_FAILED;
    }
    memset(hdl->ref_buffer, 0, hdl->ref_buffer_size);
    
    // Initialize mutex for ref_buffer protection
    if (pthread_mutex_init(&hdl->ref_buffer_mutex, NULL) != 0) {
        PR_ERR("Cannot initialize ref_buffer mutex");
        tal_free(hdl->ref_buffer);
        hdl->ref_buffer = NULL;
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
        return OPRT_COM_ERROR;
    }
    
    // Allocate output buffer
    hdl->out_buffer = (uint8_t *)tal_malloc(hdl->out_buffer_size);
    if (NULL == hdl->out_buffer) {
        PR_ERR("Cannot allocate output buffer");
        pthread_mutex_destroy(&hdl->ref_buffer_mutex);
        tal_free(hdl->ref_buffer);
        hdl->ref_buffer = NULL;
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
        return OPRT_MALLOC_FAILED;
    }
    memset(hdl->out_buffer, 0, hdl->out_buffer_size);
    
    // Allocate capture buffer
    hdl->capture_buffer = (uint8_t *)tal_malloc(hdl->capture_buffer_size);
    if (NULL == hdl->capture_buffer) {
        PR_ERR("Cannot allocate capture buffer");
        tal_free(hdl->out_buffer);
        hdl->out_buffer = NULL;
        pthread_mutex_destroy(&hdl->ref_buffer_mutex);
        tal_free(hdl->ref_buffer);
        hdl->ref_buffer = NULL;
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
        return OPRT_MALLOC_FAILED;
    }
    
    // Initialize playback mutex
    if (pthread_mutex_init(&hdl->playback_mutex, NULL) != 0) {
        PR_ERR("Cannot initialize playback mutex");
        tal_free(hdl->capture_buffer);
        hdl->capture_buffer = NULL;
        tal_free(hdl->out_buffer);
        hdl->out_buffer = NULL;
        pthread_mutex_destroy(&hdl->ref_buffer_mutex);
        tal_free(hdl->ref_buffer);
        hdl->ref_buffer = NULL;
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
        return OPRT_COM_ERROR;
    }
    
    PR_INFO("PulseAudio capture device setup: %s, rate=%d, channels=%d, bits=%d", 
            hdl->cfg.capture_device ? hdl->cfg.capture_device : "default",
            hdl->cfg.sample_rate, hdl->cfg.channels, hdl->cfg.data_bits);
    
    return OPRT_OK;
}

/**
 * @brief Setup PulseAudio playback device
 */
static OPERATE_RET __pulse_setup_playback(TDD_AUDIO_PULSE_HANDLE_T *hdl)
{
    int error;
    
    // Setup sample specification for playback (use same as capture or custom)
    pa_sample_spec playback_ss = {
        .format = __get_pulse_format(hdl->cfg.data_bits),
        .rate = hdl->cfg.spk_sample_rate,
        .channels = hdl->cfg.channels
    };
    
    // Setup buffer attributes for playback
    pa_buffer_attr ba = {
        .maxlength = (uint32_t)-1,
        .tlength = (uint32_t)-1,
        .prebuf = (uint32_t)-1,
        .minreq = (uint32_t)-1,
        .fragsize = __calculate_buffer_size(hdl)
    };
    
    // Open PulseAudio playback device
    hdl->playback_handle = pa_simple_new(
        NULL,                   // Use default PulseAudio server
        hdl->cfg.app_name,      // Application name
        PA_STREAM_PLAYBACK,     // Stream direction
        hdl->cfg.playback_device, // Device name (NULL for default)
        "Tuya Audio Playback",  // Stream description
        &playback_ss,           // Sample specification
        NULL,                   // Channel map (NULL for default)
        &ba,                    // Buffer attributes
        &error                  // Error code
    );
    
    if (!hdl->playback_handle) {
        PR_WARN("PulseAudio playback device '%s' not available: %s", 
                hdl->cfg.playback_device ? hdl->cfg.playback_device : "default",
                pa_strerror(error));
        return OPRT_COM_ERROR;
    }
    
    PR_INFO("PulseAudio playback device setup: %s, rate=%d, channels=%d, bits=%d", 
            hdl->cfg.playback_device ? hdl->cfg.playback_device : "default",
            hdl->cfg.spk_sample_rate, hdl->cfg.channels, hdl->cfg.data_bits);
    
    return OPRT_OK;
}

/**
 * @brief Audio capture thread
 */
static void *__pulse_capture_thread(void *arg)
{
    TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)arg;
    int error;
    
    PR_INFO("PulseAudio capture thread started");
    
    while (hdl->capture_running) {
        // Read audio frames from PulseAudio
        if (pa_simple_read(hdl->capture_handle, hdl->capture_buffer, 
                          hdl->capture_buffer_size, &error) < 0) {
            PR_ERR("PulseAudio capture error: %s", pa_strerror(error));
            break;
        }
        
        // Call callback with captured data
        if (hdl->mic_cb && hdl->cfg.period_frames > 0) {
            hdl->mic_cb(TDL_AUDIO_FRAME_FORMAT_PCM, TDL_AUDIO_STATUS_RECEIVING, 
                       hdl->capture_buffer, hdl->capture_buffer_size);
        }
        
        // Feed data to VAD/KWS if registered
        if ((__s_vad_feed_cb || __s_kws_feed_cb) && hdl->cfg.period_frames > 0) {
            int16_t *mic_data = (int16_t *)hdl->capture_buffer;
            int16_t *ref_data = (int16_t *)hdl->ref_buffer;
            int16_t *out_data = (int16_t *)hdl->out_buffer;
            
            // Safely read ref_buffer with mutex protection
            pthread_mutex_lock(&hdl->ref_buffer_mutex);
            
            if (__s_vad_feed_cb) {
                __s_vad_feed_cb(mic_data, ref_data, out_data, hdl->cfg.period_frames);
            }
            if (__s_kws_feed_cb) {
                __s_kws_feed_cb(mic_data, ref_data, out_data, hdl->cfg.period_frames);
            }
            
            // Clear ref_buffer after use
            memset(hdl->ref_buffer, 0, hdl->ref_buffer_size);
            pthread_mutex_unlock(&hdl->ref_buffer_mutex);
        }
    }
    
    PR_INFO("PulseAudio capture thread stopped");
    return NULL;
}

/**
 * @brief Open audio device
 */
static OPERATE_RET __tdd_audio_pulse_open(TDD_AUDIO_HANDLE_T handle, TDL_AUDIO_MIC_CB mic_cb)
{
    OPERATE_RET rt = OPRT_OK;
    TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)handle;
    
    if (NULL == hdl) {
        return OPRT_COM_ERROR;
    }
    
    hdl->mic_cb = mic_cb;
    
    // Setup capture device
    rt = __pulse_setup_capture(hdl);
    if (OPRT_OK != rt) {
        PR_ERR("Failed to setup PulseAudio capture device");
        return rt;
    }
    
    // Setup playback device (lazy open, will open on first play)
    // We don't open playback here to save resources
    
    // Set initial volume
    hdl->play_volume = 80; // Default volume 80%
    
    // Start capture thread
    hdl->capture_running = 1;
    int err = pthread_create(&hdl->capture_thread, NULL, __pulse_capture_thread, hdl);
    if (err != 0) {
        PR_ERR("Failed to create capture thread: %d", err);
        hdl->capture_running = 0;
        // Cleanup
        if (hdl->capture_handle) {
            pa_simple_free(hdl->capture_handle);
            hdl->capture_handle = NULL;
        }
        tal_free(hdl->capture_buffer);
        hdl->capture_buffer = NULL;
        return OPRT_COM_ERROR;
    }
    
    PR_INFO("PulseAudio audio device opened successfully");
    
    return rt;
}

/**
 * @brief Play audio data
 */
static OPERATE_RET __tdd_audio_pulse_play(TDD_AUDIO_HANDLE_T handle, uint8_t *data, uint32_t len)
{
    OPERATE_RET rt = OPRT_OK;
    TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)handle;
    
    TUYA_CHECK_NULL_RETURN(hdl, OPRT_COM_ERROR);
    
    if (NULL == data || len == 0) {
        PR_ERR("Play data is NULL or empty");
        return OPRT_COM_ERROR;
    }
    
    pthread_mutex_lock(&hdl->playback_mutex);
    
    // Lazy open playback device
    if (!hdl->playback_handle) {
        rt = __pulse_setup_playback(hdl);
        if (OPRT_OK != rt) {
            PR_ERR("Failed to open PulseAudio playback device");
            pthread_mutex_unlock(&hdl->playback_mutex);
            return rt;
        }
    }
    
    // --- Debug: Save playback audio to file ---
    static FILE *pulse_save_fp = NULL;
    static int saved_len = 0;
    if (saved_len < 5 * 1024 * 1024) { 
        if (pulse_save_fp == NULL) {
            pulse_save_fp = fopen("pulse_playback.pcm", "wb");
        }
        if (pulse_save_fp) {
            fwrite(data, 1, len, pulse_save_fp);
            fflush(pulse_save_fp);
            saved_len += len;
        }
    } else if (pulse_save_fp) {
        fclose(pulse_save_fp);
        pulse_save_fp = NULL;
        PR_INFO("Pulse debug file saved to pulse_playback.pcm");
    }
    // ------------------------------------------
    
    // Copy playback data to ref_buffer for AEC reference
    if (hdl->ref_buffer && hdl->ref_buffer_size > 0) {
        pthread_mutex_lock(&hdl->ref_buffer_mutex);
        uint32_t copy_size = (len < hdl->ref_buffer_size) ? len : hdl->ref_buffer_size;
        memcpy(hdl->ref_buffer, data, copy_size);
        pthread_mutex_unlock(&hdl->ref_buffer_mutex);
    }
    
    // Write audio frames to PulseAudio
    int error;
    if (pa_simple_write(hdl->playback_handle, data, len, &error) < 0) {
        PR_ERR("PulseAudio playback error: %s", pa_strerror(error));
        rt = OPRT_COM_ERROR;
    } else {
        // Wait for playback to complete
        if (pa_simple_drain(hdl->playback_handle, &error) < 0) {
            PR_ERR("PulseAudio drain error: %s", pa_strerror(error));
        }
        
        // Debug info
        int16_t *pcm = (int16_t *)data;
        uint32_t samples = len / sizeof(int16_t);
        int16_t max = 0;
        for (uint32_t i = 0; i < samples; i++) {
            int16_t v = pcm[i] >= 0 ? pcm[i] : -pcm[i];
            if (v > max) max = v;
        }
        PR_DEBUG("Pulse play len=%u, max_pcm=%d", len, max);
    }
    
    pthread_mutex_unlock(&hdl->playback_mutex);
    
    return rt;
}

/**
 * @brief Set playback volume using PulseAudio/PipeWire
 */
static OPERATE_RET __tdd_audio_pulse_set_volume(TDD_AUDIO_HANDLE_T handle, uint8_t volume)
{
    TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)handle;
    
    TUYA_CHECK_NULL_RETURN(hdl, OPRT_COM_ERROR);
    
    if (volume > 100) {
        volume = 100;
    }
    
    hdl->play_volume = volume;
    
    // Note: PulseAudio simple API doesn't support volume control directly
    // You can use pactl command or implement PA context API for volume control
    // For now, we just store the volume setting
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "pactl set-sink-volume @DEFAULT_SINK@ %d%%", volume);
    
    int ret = system(cmd);
    if (ret != 0) {
        PR_WARN("Failed to set volume via pactl, command: %s", cmd);
        // Try alternative method
        snprintf(cmd, sizeof(cmd), "amixer set Master %d%%", volume);
        system(cmd);
    }
    
    PR_DEBUG("Volume set to %d%%", volume);
    
    return OPRT_OK;
}

/**
 * @brief Audio configuration command handler
 */
static OPERATE_RET __tdd_audio_pulse_config(TDD_AUDIO_HANDLE_T handle, TDD_AUDIO_CMD_E cmd, void *args)
{
    OPERATE_RET rt = OPRT_OK;
    
    TUYA_CHECK_NULL_RETURN(handle, OPRT_COM_ERROR);
    
    switch (cmd) {
    case TDD_AUDIO_CMD_SET_VOLUME: {
        TUYA_CHECK_NULL_GOTO(args, __EXIT);
        uint8_t volume = *(uint8_t *)args;
        TUYA_CALL_ERR_GOTO(__tdd_audio_pulse_set_volume(handle, volume), __EXIT);
    } break;
    
    case TDD_AUDIO_CMD_PLAY_STOP: {
        TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)handle;
        pthread_mutex_lock(&hdl->playback_mutex);
        
        if (hdl->playback_handle) {
            // Drain any pending playback
            int error;
            pa_simple_drain(hdl->playback_handle, &error);
            pa_simple_free(hdl->playback_handle);
            hdl->playback_handle = NULL;
        }
        
        // Clear ref_buffer when playback stops
        if (hdl->ref_buffer && hdl->ref_buffer_size > 0) {
            pthread_mutex_lock(&hdl->ref_buffer_mutex);
            memset(hdl->ref_buffer, 0, hdl->ref_buffer_size);
            pthread_mutex_unlock(&hdl->ref_buffer_mutex);
        }
        
        pthread_mutex_unlock(&hdl->playback_mutex);
    } break;
    
    default:
        rt = OPRT_INVALID_PARM;
        break;
    }
    
__EXIT:
    return rt;
}

/**
 * @brief Close audio device
 */
static OPERATE_RET __tdd_audio_pulse_close(TDD_AUDIO_HANDLE_T handle)
{
    OPERATE_RET rt = OPRT_OK;
    TDD_AUDIO_PULSE_HANDLE_T *hdl = (TDD_AUDIO_PULSE_HANDLE_T *)handle;
    
    TUYA_CHECK_NULL_RETURN(hdl, OPRT_COM_ERROR);
    
    // Stop capture thread
    if (hdl->capture_running) {
        hdl->capture_running = 0;
        pthread_join(hdl->capture_thread, NULL);
    }
    
    // Close PulseAudio handles
    if (hdl->capture_handle) {
        pa_simple_free(hdl->capture_handle);
        hdl->capture_handle = NULL;
    }
    
    if (hdl->playback_handle) {
        pa_simple_free(hdl->playback_handle);
        hdl->playback_handle = NULL;
    }
    
    // Free buffers
    if (hdl->capture_buffer) {
        tal_free(hdl->capture_buffer);
        hdl->capture_buffer = NULL;
    }
    
    if (hdl->ref_buffer) {
        tal_free(hdl->ref_buffer);
        hdl->ref_buffer = NULL;
    }
    
    if (hdl->out_buffer) {
        tal_free(hdl->out_buffer);
        hdl->out_buffer = NULL;
    }
    
    // Destroy mutexes
    pthread_mutex_destroy(&hdl->ref_buffer_mutex);
    pthread_mutex_destroy(&hdl->playback_mutex);
    
    PR_INFO("PulseAudio audio device closed");
    
    return rt;
}

/**
 * @brief Register PulseAudio audio driver
 */
OPERATE_RET tdd_audio_pulse_register(char *name, TDD_AUDIO_PULSE_CFG_T cfg)
{
    OPERATE_RET rt = OPRT_OK;
    TDD_AUDIO_PULSE_HANDLE_T *_hdl = NULL;
    TDD_AUDIO_INTFS_T intfs = {0};
    TDD_AUDIO_INFO_T info = {0};
    
    // Allocate handle
    _hdl = (TDD_AUDIO_PULSE_HANDLE_T *)tal_malloc(sizeof(TDD_AUDIO_PULSE_HANDLE_T));
    TUYA_CHECK_NULL_RETURN(_hdl, OPRT_MALLOC_FAILED);
    memset(_hdl, 0, sizeof(TDD_AUDIO_PULSE_HANDLE_T));
    
    // Copy configuration
    memcpy(&_hdl->cfg, &cfg, sizeof(TDD_AUDIO_PULSE_CFG_T));
    
    info.sample_rate   = cfg.sample_rate;
    info.sample_ch_num = cfg.channels;
    info.sample_bits   = cfg.data_bits;
    info.sample_tm_ms  = AUDIO_PCM_FRAME_MS;
    
    // Setup interface functions
    intfs.open = __tdd_audio_pulse_open;
    intfs.play = __tdd_audio_pulse_play;
    intfs.config = __tdd_audio_pulse_config;
    intfs.close = __tdd_audio_pulse_close;
    
    // Register with TDL audio management
    TUYA_CALL_ERR_GOTO(tdl_audio_driver_register(name, (TDD_AUDIO_HANDLE_T)_hdl, &intfs, &info), __ERR);
    
    PR_INFO("PulseAudio audio driver registered: %s", name);
    
    return rt;
    
__ERR:
    if (_hdl) {
        tal_free(_hdl);
        _hdl = NULL;
    }
    
    return rt;
}

/**
 * @brief Register VAD audio feed callback
 */
void tdd_audio_pulse_register_vad_feed_cb(TDD_AUDIO_AFE_FEED_CB cb)
{
    __s_vad_feed_cb = cb;
    PR_INFO("VAD feed callback %s", cb ? "registered" : "cleared");
}

/**
 * @brief Register KWS audio feed callback
 */
void tdd_audio_pulse_register_kws_feed_cb(TDD_AUDIO_AFE_FEED_CB cb)
{
    __s_kws_feed_cb = cb;
    PR_INFO("KWS feed callback %s", cb ? "registered" : "cleared");
}

/**
 * @brief Unregister VAD audio feed callback
 */
void tdd_audio_pulse_unregister_vad_feed_cb(void)
{
    __s_vad_feed_cb = NULL;
    PR_INFO("VAD feed callback unregistered");
}

/**
 * @brief Unregister KWS audio feed callback
 */
void tdd_audio_pulse_unregister_kws_feed_cb(void)
{
    __s_kws_feed_cb = NULL;
    PR_INFO("KWS feed callback unregistered");
}

#endif /* ENABLE_AUDIO_PULSE */