/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "audio_hw_qap"
//#define LOG_NDEBUG 0
#define LOG_NDDEBUG 0
//#define VERY_VERY_VERBOSE_LOGGING
#ifdef VERY_VERY_VERBOSE_LOGGING
#define DEBUG_MSG_VV DEBUG_MSG
#else
#define DEBUG_MSG_VV(a...) do { } while(0)
#endif

#define DEBUG_MSG(arg,...) ALOGD("%s: %d:  " arg, __func__, __LINE__, ##__VA_ARGS__)
#define ERROR_MSG(arg,...) ALOGE("%s: %d:  " arg, __func__, __LINE__, ##__VA_ARGS__)

#define COMPRESS_OFFLOAD_NUM_FRAGMENTS 2
#define COMPRESS_PASSTHROUGH_DDP_FRAGMENT_SIZE 4608

#define QAP_DEFAULT_COMPR_AUDIO_HANDLE 1001
#define QAP_DEFAULT_COMPR_PASSTHROUGH_HANDLE 1002
#define QAP_DEFAULT_PASSTHROUGH_HANDLE 1003

#define COMPRESS_OFFLOAD_PLAYBACK_LATENCY 300
#define MS12_LATENCY 12960 //frames for 270ms

#define MIN_PCM_OFFLOAD_FRAGMENT_SIZE 512
#define MAX_PCM_OFFLOAD_FRAGMENT_SIZE (240 * 1024)

#define DIV_ROUND_UP(x, y) (((x) + (y) - 1)/(y))
#define ALIGN(x, y) ((y) * DIV_ROUND_UP((x), (y)))

/* Pcm input node buffer size is 6144 bytes, i.e, 32msec for 48000 samplerate */
#define QAP_MODULE_PCM_INPUT_BUFFER_LATENCY 32

#define MS12_PCM_OUT_FRAGMENT_SIZE 1536 //samples
#define MS12_PCM_IN_FRAGMENT_SIZE 1536 //samples

#define DD_FRAME_SIZE 1536
#define DDP_FRAME_SIZE DD_FRAME_SIZE
/*
 * DD encoder output size for one frame.
 */
#define DD_ENCODER_OUTPUT_SIZE 2560
/*
 * DDP encoder output size for one frame.
 */
#define DDP_ENCODER_OUTPUT_SIZE 4608
#define AUDIO_OUTPUT_SINK_CAPABILITY_DECTECTION 0x20000

/*********TODO Need to get correct values.*************************/

#define DTS_PCM_OUT_FRAGMENT_SIZE 1024 //samples

#define DTS_FRAME_SIZE 1536
#define DTSHD_FRAME_SIZE DTS_FRAME_SIZE
/*
 * DTS encoder output size for one frame.
 */
#define DTS_ENCODER_OUTPUT_SIZE 2560
/*
 * DTSHD encoder output size for one frame.
 */
#define DTSHD_ENCODER_OUTPUT_SIZE 4608
/******************************************************************/

/*
 * QAP Latency to process buffers since out_write from primary HAL
 */
#define QAP_COMPRESS_OFFLOAD_PROCESSING_LATENCY 18
#define QAP_PCM_OFFLOAD_PROCESSING_LATENCY 48

//TODO: Need to handle for DTS
#define QAP_DEEP_BUFFER_OUTPUT_PERIOD_SIZE 1536

#define MS12_ATMOS_LOCK_MASK  2
#define MS12_CHMOD_LOCK_MASK  3

#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <dlfcn.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <cutils/properties.h>
#include <cutils/str_parms.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include "audio_utils/primitives.h"
#include "audio_hw.h"
#include "platform_api.h"
#include <platform.h>
#include <system/thread_defs.h>
#include <cutils/sched_policy.h>
#include "audio_extn.h"
#include <qti_audio.h>
#include <qap_api.h>
#include "sound/compress_params.h"
#include "ip_hdlr_intf.h"
#include "dolby_ms12.h"

#ifdef DYNAMIC_LOG_ENABLED
#include <log_xml_parser.h>
#define LOG_MASK HAL_MOD_FILE_QAF
#include <log_utils.h>
#endif

//TODO: Need to remove this.
#define QAP_OUTPUT_SAMPLING_RATE 48000

#ifdef QAP_DUMP_ENABLED
FILE *fp_output_writer_hdmi = NULL;
#endif

qap_output_delay_t main_delay_event_data = {0};
bool delay_event_fired = false;

//Types of MM module, currently supported by QAP.
typedef enum {
    MS12,
    DTS_M8,
    MAX_MM_MODULE_TYPE,
    INVALID_MM_MODULE
} mm_module_type;

typedef enum {
    QAP_OUT_TRANSCODE_PASSTHROUGH = 0, /* Transcode passthrough via MM module*/
    QAP_OUT_OFFLOAD_MCH, /* Multi-channel PCM offload*/
    QAP_OUT_OFFLOAD, /* PCM offload */

    MAX_QAP_MODULE_OUT
} mm_module_output_type;

typedef enum {
    QAP_IN_MAIN = 0, /* Single PID Main/Primary or Dual-PID stream */
    QAP_IN_ASSOC,    /* Associated/Secondary stream */
    QAP_IN_PCM,      /* PCM stream. */
    QAP_IN_MAIN_2,   /* Single PID Main2 stream */
    MAX_QAP_MODULE_IN
} mm_module_input_type;

typedef enum {
    STOPPED,    /*Stream is in stop state. */
    STOPPING,   /*Stream is stopping, waiting for EOS. */
    RUN,        /*Stream is in run state. */
    MAX_STATES
} qap_stream_state;

struct qap_module {
    audio_session_handle_t session_handle;
    void *qap_lib;
    void *qap_handle;

    /*Input stream of MM module */
    struct stream_out *stream_in[MAX_QAP_MODULE_IN];
    pthread_mutex_t qap_stream_in_lock[MAX_QAP_MODULE_IN];
    /*Output Stream from MM module */
    struct stream_out *stream_out[MAX_QAP_MODULE_OUT];

    /*Media format associated with each output id raised by mm module. */
    qap_session_outputs_config_t session_outputs_config;
    /*Flag is set if media format is changed for an mm module output. */
    bool is_media_fmt_changed[MAX_QAP_MODULE_OUT];
    /*Index to be updated in session_outputs_config array for a new mm module output. */
    int new_out_format_index;

    //BT session handle.
    void *bt_hdl;

    float vol_left;
    float vol_right;
    bool is_vol_set;
    qap_stream_state stream_state[MAX_QAP_MODULE_IN];
    bool is_session_closing;
    bool is_session_output_active;
    unsigned long long qap_output_bytes_written[MAX_QAP_MODULE_OUT];
    pthread_cond_t session_output_cond;
    pthread_mutex_t session_output_lock;
    pthread_cond_t drain_output_cond;
    int  interpolation;
    bool pause;
};

struct qap {
    struct audio_device *adev;

    pthread_mutex_t lock;

    bool bt_connect;
    bool hdmi_connect;
    char ms12_out_format[4];
    int hdmi_sink_channels;
    //bit 0: atmos_lock, bit 1: chmod_lock, bit 2: request for atmos_lock, bit 3: request for chmod_
    char ms12_lock;

    //Flag to indicate if QAP transcode output stream is enabled from any mm module.
    bool passthrough_enabled;
    //Flag to indicate if QAP mch pcm output stream is enabled from any mm module.
    bool mch_pcm_hdmi_enabled;

    //Flag to indicate if msmd is supported.
    bool qap_msmd_enabled;
    bool bypass_enable;
    struct audio_stream_out hal_stream_ops;

    bool qap_output_block_handling;
    int qap_active_api_count;
    //Handle of QAP input stream, which is routed as QAP passthrough.
    struct stream_out *passthrough_in;
    //Handle of QAP passthrough stream.
    struct stream_out *passthrough_out;

    struct qap_module qap_mod[MAX_MM_MODULE_TYPE];
    bool wait_on_cond;
};

#define MAX_OUTPUTS  2
#define MAX_OUT_BUFFER   (20*1024)
//Global handle of QAP. Access to this should be protected by mutex lock.
static struct qap *p_qap = NULL;

struct data_cb_data {
    void * buff_ptr;
    int buff_size;
    uint32_t dev_id;
};
static struct data_cb_data cb_data_array[MAX_OUTPUTS];
static int no_of_outputs  = 0;

#define ID_RIFF 0x46464952
#define ID_WAVE 0x45564157
#define ID_FMT  0x20746d66
#define ID_DATA 0x61746164
#define WAVE_FORMAT_PCM 0x0001

struct wav_header {
   uint32_t riff_id;
   uint32_t riff_sz;
   uint32_t riff_fmt;

   uint32_t fmt_id;
   uint32_t fmt_sz;
   uint16_t audio_format;
   uint16_t num_channels;
   uint32_t sample_rate;
   uint32_t byte_rate;       /* sample_rate * num_channels * bps / 8 */
   uint16_t block_align;     /* num_channels * bps / 8 */
   uint16_t bits_per_sample;

   uint32_t data_id;
   uint32_t data_sz;
};

struct ecref {
    char file_dump[40];
    FILE *proxy_out_ptr;
    struct wav_header hdr;
    long long total_bytes_written;
    int ch;
};

static struct ecref ec;
static int get_input_stream_index_l(struct stream_out *out);
static struct qap_module* get_qap_module_for_input_stream_l(struct stream_out *out);

static void insert_wav_header(FILE *fp, struct audio_config *config) {
    struct wav_header *hdr = &ec.hdr;
    int bps = 16;
    int channels = 0;

     if (config) {
         channels = audio_channel_count_from_out_mask(config->channel_mask);
         hdr->riff_id = ID_RIFF;
         hdr->riff_sz = 0;
         hdr->riff_fmt = ID_WAVE;

         hdr->fmt_id = ID_FMT;
         hdr->fmt_sz = 16;
         hdr->audio_format = WAVE_FORMAT_PCM;
         hdr->num_channels = channels;
         hdr->sample_rate = config->sample_rate;
         hdr->byte_rate = hdr->sample_rate * hdr->num_channels * (bps/8);
         hdr->block_align = hdr->num_channels * (bps/8);
         hdr->bits_per_sample = bps;

         hdr->data_id = ID_DATA;
         hdr->data_sz = 0;
     }

     hdr->data_sz = ec.total_bytes_written;
     hdr->riff_sz = 4 + (8 + hdr->fmt_sz) + (8 + hdr->data_sz);
     fseek(fp, 0, SEEK_SET);
     fwrite(hdr, 1, sizeof(*hdr), fp);
}

static void lock_session_output(struct qap_module *qap_mod)
{
    int ret = 0;

    do {
       usleep(1000);
       ret = pthread_mutex_trylock(&qap_mod->session_output_lock);
    } while(ret != 0);
    DEBUG_MSG_VV("Session Output lock acquired %p", qap_mod);

    return;
}

static void unlock_session_output(struct qap_module *qap_mod)
{
    pthread_mutex_unlock(&qap_mod->session_output_lock);
    DEBUG_MSG_VV("Session Output lock released");
    return;
}

static void lock_all_qap_stream_in(struct qap_module *qap_mod)
{
    int i;
    for (i = QAP_IN_MAIN; i < MAX_QAP_MODULE_IN; i++) {
        pthread_mutex_lock(&qap_mod->qap_stream_in_lock[i]);
        DEBUG_MSG_VV("Qap stream in lock acquired %p for index=%d",
                      &qap_mod->qap_stream_in_lock[i], i);
    }
    return;
 }

static void unlock_all_qap_stream_in(struct qap_module *qap_mod)
{
    int i;
    for (i = QAP_IN_MAIN; i < MAX_QAP_MODULE_IN; i++) {
        pthread_mutex_unlock(&qap_mod->qap_stream_in_lock[i]);
        DEBUG_MSG_VV("Qap stream in lock released %p for index=%d",
                      &qap_mod->qap_stream_in_lock[i], i);
    }
    return;
}

static void lock_qap_stream_in(struct stream_out *out)
{
    int index = -1;
    struct qap_module *qap_mod = NULL;

    qap_mod = get_qap_module_for_input_stream_l(out);
    index = get_input_stream_index_l(out);
    pthread_mutex_lock(&qap_mod->qap_stream_in_lock[index]);
    DEBUG_MSG_VV("Qap stream in lock acquired %p for index =%d",
                  &qap_mod->qap_stream_in_lock[index], index);

    return;
}

static void unlock_qap_stream_in(struct stream_out *out)
{
    int index = -1;
    struct qap_module *qap_mod = NULL;

    qap_mod = get_qap_module_for_input_stream_l(out);
    index = get_input_stream_index_l(out);

    pthread_mutex_unlock(&qap_mod->qap_stream_in_lock[index]);
    DEBUG_MSG_VV("Qap stream in lock released %p for index=%d",
                  &qap_mod->qap_stream_in_lock[index], index);
    return;
}

static void check_and_activate_output_thread(bool awake)
{
    struct qap_module *qap_mod = &(p_qap->qap_mod[MS12]);

    lock_session_output(qap_mod);

    if (awake) {
        if (qap_mod->is_session_output_active == false && !p_qap->qap_active_api_count) {
            pthread_cond_signal(&qap_mod->session_output_cond);
        }
        p_qap->qap_active_api_count++;
    } else {
        p_qap->qap_active_api_count--;
    }
    unlock_session_output(qap_mod);

    return;
}

/* Gets the pointer to qap module for the qap input stream. */
static struct qap_module* get_qap_module_for_input_stream_l(struct stream_out *out)
{
    struct qap_module *qap_mod = NULL;
    int i, j;
    if (!p_qap) return NULL;

    for (i = 0; i < MAX_MM_MODULE_TYPE; i++) {
        for (j = 0; j < MAX_QAP_MODULE_IN; j++) {
            if (p_qap->qap_mod[i].stream_in[j] == out) {
                qap_mod = &(p_qap->qap_mod[i]);
                break;
            }
        }
    }

    return qap_mod;
}

/* Finds the mm module input stream index for the QAP input stream. */
static int get_input_stream_index_l(struct stream_out *out)
{
    int index = -1, j;
    struct qap_module* qap_mod = NULL;

    qap_mod = get_qap_module_for_input_stream_l(out);
    if (!qap_mod) return index;

    for (j = 0; j < MAX_QAP_MODULE_IN; j++) {
        if (qap_mod->stream_in[j] == out) {
            index = j;
            break;
        }
    }

    return index;
}

static void set_stream_state_l(struct stream_out *out, int state)
{
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);
    int index = get_input_stream_index_l(out);
    if (qap_mod && index >= 0) qap_mod->stream_state[index] = state;
}

static bool check_stream_state_l(struct stream_out *out, int state)
{
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);
    int index = get_input_stream_index_l(out);
    if (qap_mod && index >= 0) return ((int)qap_mod->stream_state[index] == state);
    return false;
}

/* Finds the right mm module for the QAP input stream format. */
static mm_module_type get_mm_module_for_format_l(audio_format_t format)
{
    int j;

    DEBUG_MSG("Format 0x%x", format);

    if (format == AUDIO_FORMAT_PCM_16_BIT) {
        //If dts is not supported then alway support pcm with MS12
        if (!property_get_bool("vendor.audio.qap.dts_m8", false)) { //TODO: Need to add this property for DTS.
            return MS12;
        }

        //If QAP passthrough is active then send the PCM stream to primary HAL.
        if (!p_qap->passthrough_out) {
            /* Iff any stream is active in MS12 module then route PCM stream to it. */
            for (j = 0; j < MAX_QAP_MODULE_IN; j++) {
                if (p_qap->qap_mod[MS12].stream_in[j]) {
                    return MS12;
                }
            }
        }
        return INVALID_MM_MODULE;
    }

    switch (format & AUDIO_FORMAT_MAIN_MASK) {
        case AUDIO_FORMAT_AC3:
        case AUDIO_FORMAT_E_AC3:
        case AUDIO_FORMAT_E_AC3_JOC:
        case AUDIO_FORMAT_AAC:
        case AUDIO_FORMAT_AAC_ADTS:
        case AUDIO_FORMAT_AC4:
            return MS12;
        case AUDIO_FORMAT_DTS:
        case AUDIO_FORMAT_DTS_HD:
            return DTS_M8;
        default:
            return INVALID_MM_MODULE;
    }
}

//Checks if any main or pcm stream is running in the session.
static bool is_any_stream_running_l(struct qap_module* qap_mod)
{
    //Not checking associated stream.
    struct stream_out *out = qap_mod->stream_in[QAP_IN_MAIN];
    struct stream_out *out_pcm = qap_mod->stream_in[QAP_IN_PCM];
    struct stream_out *out_main2 = qap_mod->stream_in[QAP_IN_MAIN_2];

    if ((out == NULL || (out != NULL && check_stream_state_l(out, STOPPED)))
        && (out_main2 == NULL || (out_main2 != NULL && check_stream_state_l(out_main2, STOPPED)))
        && (out_pcm == NULL || (out_pcm != NULL && check_stream_state_l(out_pcm, STOPPED)))) {
        return false;
    }
    return true;
}

/* Gets the pcm output buffer size(in samples) for the mm module. */
static uint32_t get_pcm_output_buffer_size_samples_l(struct qap_module *qap_mod)
{
    uint32_t pcm_output_buffer_size = 0;

    if (qap_mod == &p_qap->qap_mod[MS12]) {
        pcm_output_buffer_size = MS12_PCM_OUT_FRAGMENT_SIZE;
    } else if (qap_mod == &p_qap->qap_mod[DTS_M8]) {
        pcm_output_buffer_size = DTS_PCM_OUT_FRAGMENT_SIZE;
    }

    return pcm_output_buffer_size;
}

static int get_media_fmt_array_index_for_output_id_l(
        struct qap_module* qap_mod,
        uint32_t output_id)
{
    int i;
    for (i = 0; i < MAX_SUPPORTED_OUTPUTS; i++) {
        if (qap_mod->session_outputs_config.output_config[i].id == output_id) {
            return i;
        }
    }
    return -1;
}

/* Acquire Mutex lock on output stream */
static void lock_output_stream_l(struct stream_out *out)
{
    pthread_mutex_lock(&out->pre_lock);
    pthread_mutex_lock(&out->lock);
    pthread_mutex_unlock(&out->pre_lock);
}

/* Release Mutex lock on output stream */
static void unlock_output_stream_l(struct stream_out *out)
{
    pthread_mutex_unlock(&out->lock);
}

/* Checks if stream can be routed as QAP passthrough or not. */
static bool audio_extn_qap_passthrough_enabled(struct stream_out *out)
{
    DEBUG_MSG("Format 0x%x", out->format);
    bool is_enabled = false;

    if (!p_qap) return false;

    if ((!property_get_bool("vendor.audio.qap.reencode", false))
        && property_get_bool("vendor.audio.qap.passthrough", false)) {

        if ((out->format == AUDIO_FORMAT_PCM_16_BIT) && (popcount(out->channel_mask) > 2)) {
            is_enabled = true;
        } else if (property_get_bool("vendor.audio.offload.passthrough", false)) {
            switch (out->format) {
                case AUDIO_FORMAT_AC3:
                case AUDIO_FORMAT_E_AC3:
                case AUDIO_FORMAT_E_AC3_JOC:
                case AUDIO_FORMAT_DTS:
                case AUDIO_FORMAT_DTS_HD:
                case AUDIO_FORMAT_DOLBY_TRUEHD:
                case AUDIO_FORMAT_IEC61937: {
                    is_enabled = true;
                    break;
                }
                default:
                    is_enabled = false;
                break;
            }
        }
    }

    return is_enabled;
}

/*Closes all pcm hdmi output from QAP. */
static void close_all_pcm_hdmi_output_l()
{
    int i;
    //Closing all the PCM HDMI output stream from QAP.
    for (i = 0; i < MAX_MM_MODULE_TYPE; i++) {
        if (p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD_MCH]) {
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                     (struct audio_stream_out *)(p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD_MCH]));
            p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD_MCH] = NULL;
        }

        if ((p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD])
            && (p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD]->devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                     (struct audio_stream_out *)(p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD]));
            p_qap->qap_mod[i].stream_out[QAP_OUT_OFFLOAD] = NULL;
        }
    }

    p_qap->mch_pcm_hdmi_enabled = 0;
}

static void close_all_hdmi_output_l()
{
    int k;
    for (k = 0; k < MAX_MM_MODULE_TYPE; k++) {
        if (p_qap->qap_mod[k].stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]) {
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                     (struct audio_stream_out *)(p_qap->qap_mod[k].stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]));
            p_qap->qap_mod[k].stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH] = NULL;
        }
    }
    p_qap->passthrough_enabled = 0;

    close_all_pcm_hdmi_output_l();
}

#define DSD_VOLUME_MIN_DB (-96)
static float AmpToDb(float amplification)
{
     float db = DSD_VOLUME_MIN_DB;
     if (amplification > 0) {
         db = 20 * log10(amplification);
         if(db < DSD_VOLUME_MIN_DB)
             return DSD_VOLUME_MIN_DB;
     }
     return db;
}

static int set_ms12_atmos_lock(int lock)
{
    int ret = 0;
    int32_t cmd_data[2] = {0};
    struct qap_module *qap_mod = &(p_qap->qap_mod[MS12]);

    DEBUG_MSG_VV("%s:%d Entry", __func__, __LINE__);
    if (p_qap) {
       if (!p_qap->qap_mod[MS12].session_handle) {
           p_qap->ms12_lock = p_qap->ms12_lock | lock;
           //bit 2: save request for atmos_lock before session open
           p_qap->ms12_lock = p_qap->ms12_lock | (1 << MS12_ATMOS_LOCK_MASK);
           ERROR_MSG("request recieved but session is not setup, caching request");
           return ret;
       }
    }

    if (!(p_qap->ms12_lock & (1 << MS12_ATMOS_LOCK_MASK)))
        pthread_mutex_lock(&p_qap->lock);

    if (!p_qap->bypass_enable) {
        cmd_data[0] = MS12_SESSION_CFG_ATMOS_LOCK;
        cmd_data[1] = lock;
        check_and_activate_output_thread(true);
        if (qap_mod->session_handle != NULL) {
            ret = qap_session_cmd(qap_mod->session_handle,
                  QAP_SESSION_CMD_SET_PARAM,
                  sizeof(cmd_data),
                  &cmd_data[0],
                  NULL,
                  NULL);

           if (ret != QAP_STATUS_OK) {
              ERROR_MSG("atmos_lock set failed");
           }
        } else
           DEBUG_MSG("qap module is not yet opened!!,atmos_lock cannot be configured");
        check_and_activate_output_thread(false);
    } else {
        ERROR_MSG("in bypass mode, atmos lock can't be enabled/disabled");
        ret = -1;
    }

    pthread_mutex_unlock(&p_qap->lock);
    DEBUG_MSG_VV("Exit");
    return ret;
}

static int set_ms12_channel_mode_lock(int lock)
{
    int ret = 0;
    int32_t cmd_data[2] = {0};
    struct qap_module *qap_mod = &(p_qap->qap_mod[MS12]);

    DEBUG_MSG_VV("%s:%d Entry", __func__, __LINE__);
    if (p_qap) {
       if (!p_qap->qap_mod[MS12].session_handle) {
           p_qap->ms12_lock = p_qap->ms12_lock | lock << 1;
           p_qap->ms12_lock = p_qap->ms12_lock | (1 << MS12_CHMOD_LOCK_MASK); //bit 2: save request for atmos_lock before session open
           ERROR_MSG("request recieved but session is not setup, caching request");
           return ret;
       }
    }

    if (!(p_qap->ms12_lock & (1 << MS12_CHMOD_LOCK_MASK)))
          pthread_mutex_lock(&p_qap->lock);

    if (!p_qap->bypass_enable) {
        cmd_data[0] = MS12_SESSION_CFG_CHMOD_LOCKING;
        cmd_data[1] = lock;

        check_and_activate_output_thread(true);
        if (qap_mod->session_handle != NULL) {
            ret = qap_session_cmd(qap_mod->session_handle,
                  QAP_SESSION_CMD_SET_PARAM,
                  sizeof(cmd_data),
                  &cmd_data[0],
                  NULL,
                  NULL);
           if (ret != QAP_STATUS_OK) {
               ERROR_MSG("channel mode lock set failed");
           }
        } else {
           DEBUG_MSG("qap module is not yet opened!! channel_mod_lock cannot be configured");
        }
        check_and_activate_output_thread(false);
    } else {
        ERROR_MSG("in bypass mode, channel mode lock can't be enabled/disabled");
        ret = -1;
    }
    pthread_mutex_unlock(&p_qap->lock);
    DEBUG_MSG_VV("Exit");
    return ret;
}

static int qap_ms12_volume_easing_cmd(struct stream_out *out, float left, __unused float right, int32_t b_pause)
{
    int ret = 0;
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);
    int32_t cmd_data[4] = {0};

    if (is_offload_usecase(out->usecase))
        cmd_data[0] = MS12_SESSION_CFG_MAIN1_MIXING_GAIN;
    else if ((out->usecase == USECASE_AUDIO_PLAYBACK_LOW_LATENCY) ||
             (out->usecase == USECASE_AUDIO_PLAYBACK_DEEP_BUFFER)) {
             DEBUG_MSG("Request for volume set for %s usecase not supported",
                      (out->usecase == USECASE_AUDIO_PLAYBACK_LOW_LATENCY) ? "low_latency":"deepbuffer");
             unlock_qap_stream_in(out);
             return -ENOSYS;
     }

     /*take left as default level and MS12 doenst support left and right seperately*/
     cmd_data[1] = AmpToDb(left);
     if(!b_pause)
        cmd_data[2] = property_get_int32("vendor.audio.qap.volumeramp.duration", 20); /* default duration 20ms */
     else
        cmd_data[2] = property_get_int32("vendor.audio.qap.pauseramp.duration", 20); /* default duration 20ms */
     cmd_data[3] = qap_mod->interpolation; /* apply gain linearly*/

     check_and_activate_output_thread(true);
     if (qap_mod->session_handle != NULL) {
         ret = qap_session_cmd(qap_mod->session_handle,
                QAP_SESSION_CMD_SET_PARAM,
                sizeof(cmd_data),
                &cmd_data[0],
                NULL,
                NULL);
          if (ret != QAP_STATUS_OK) {
              ERROR_MSG("vol set failed");
          }
     } else
        DEBUG_MSG("qap module is not yet opened!!, vol cannot be applied");
     check_and_activate_output_thread(false);

     return ret;
}

/*
* get the MS12 o/p stream and update the volume
*/
static int qap_set_stream_volume(struct audio_stream_out *stream, float left, float right)
{
    int ret = 0;
    struct stream_out *out = (struct stream_out *)stream;
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);

    DEBUG_MSG("Left %f, Right %f", left, right);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
        qap_ms12_volume_easing_cmd(out, left, right, 0);
        if (!qap_mod->pause)
             qap_mod->vol_left = left;
    } else {
        ret = p_qap->hal_stream_ops.set_volume(stream, left, right);
    }

    unlock_qap_stream_in(out);
    DEBUG_MSG("Exit");
    return ret;
}

static int qap_out_callback(stream_callback_event_t event, void *param __unused, void *cookie)
{
    struct stream_out *out = (struct stream_out *)cookie;

    out->client_callback(event, NULL, out->client_cookie);
    return 0;
}

/* Creates the QAP passthrough output stream. */
static int create_qap_passthrough_stream_l()
{
    DEBUG_MSG("Entry");

    int ret = 0;
    struct stream_out *out = p_qap->passthrough_in;

    if (!out) return -EINVAL;

    pthread_mutex_lock(&p_qap->lock);
    lock_output_stream_l(out);

    //Creating QAP passthrough output stream.
    if (NULL == p_qap->passthrough_out) {
        audio_output_flags_t flags;
        struct audio_config config;
        audio_devices_t devices;

        config.sample_rate = config.offload_info.sample_rate = out->sample_rate;
        config.offload_info.version = AUDIO_INFO_INITIALIZER.version;
        config.offload_info.size = AUDIO_INFO_INITIALIZER.size;
        config.offload_info.format = out->format;
        config.offload_info.bit_width = out->bit_width;
        config.format = out->format;
        config.offload_info.channel_mask = config.channel_mask = out->channel_mask;

        //Device is copied from the QAP passthrough input stream.
        devices = out->devices;
        flags = out->flags;

        ret = adev_open_output_stream((struct audio_hw_device *)p_qap->adev,
                                      QAP_DEFAULT_PASSTHROUGH_HANDLE,
                                      devices,
                                      flags,
                                      &config,
                                      (struct audio_stream_out **)&(p_qap->passthrough_out),
                                      NULL);
        if (ret < 0) {
            ERROR_MSG("adev_open_output_stream failed with ret = %d!", ret);
            unlock_output_stream_l(out);
            return ret;
        }
        p_qap->passthrough_in = out;
        p_qap->passthrough_out->stream.set_callback((struct audio_stream_out *)p_qap->passthrough_out,
                                                    (stream_callback_t) qap_out_callback, out);
    }

    unlock_output_stream_l(out);

    //Since QAP-Passthrough is created, close other HDMI outputs.
    close_all_hdmi_output_l();

    pthread_mutex_unlock(&p_qap->lock);
    return ret;
}


/* Stops a QAP module stream.*/
static int audio_extn_qap_stream_stop(struct stream_out *out)
{
    int ret = 0;
    DEBUG_MSG("Output Stream 0x%x", (int)out);

    if (!check_stream_state_l(out, RUN)) return ret;

    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);

    if (!qap_mod || !qap_mod->session_handle|| !out->qap_stream_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p) strm hndl(%p)",
                                qap_mod, qap_mod->session_handle, out->qap_stream_handle);
        return -EINVAL;
    }

    check_and_activate_output_thread(true);
    ret = qap_module_cmd(out->qap_stream_handle,
                            QAP_MODULE_CMD_STOP,
                            sizeof(QAP_MODULE_CMD_STOP),
                            NULL,
                            NULL,
                            NULL);
    if (QAP_STATUS_OK != ret) {
        ERROR_MSG("stop failed %d", ret);
        ret = -EINVAL;
    }
    check_and_activate_output_thread(false);
    return ret;
}

static int qap_out_drain(struct audio_stream_out* stream, audio_drain_type_t type)
{
    struct stream_out *out = (struct stream_out *)stream;
    int status = 0;
    struct qap_module *qap_mod = NULL;

    qap_mod = get_qap_module_for_input_stream_l(out);
    DEBUG_MSG("Output Stream %p", out);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
       lock_output_stream_l(out);

       //If QAP passthrough is enabled then block the drain on module stream.
       if (p_qap->passthrough_out) {
          pthread_mutex_lock(&p_qap->lock);
          //If drain is received for QAP passthorugh stream then call the primary HAL api.
          if (p_qap->passthrough_in == out) {
              status = p_qap->passthrough_out->stream.drain(
                      (struct audio_stream_out *)p_qap->passthrough_out, type);
          }
          pthread_mutex_unlock(&p_qap->lock);
       } else if (!is_any_stream_running_l(qap_mod)) {
          //If stream is already stopped then send the drain ready.
          DEBUG_MSG("All streams are in stopped state, return drain ready event");
          out->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out->client_cookie);
          set_stream_state_l(out, STOPPED);
       } else {
          qap_audio_buffer_t *buffer;
          buffer = (qap_audio_buffer_t *) calloc(1, sizeof(qap_audio_buffer_t));
          buffer->common_params.offset = 0;
          buffer->common_params.data = buffer;
          buffer->common_params.size = 0;
          buffer->buffer_parms.input_buf_params.flags = QAP_BUFFER_EOS;
          DEBUG_MSG("Queing EOS buffer %p flags %d size %d",
                    buffer, buffer->buffer_parms.input_buf_params.flags,
                    buffer->common_params.size);
          check_and_activate_output_thread(true);
          status = qap_module_process(out->qap_stream_handle, buffer);
          check_and_activate_output_thread(false);
          if (QAP_STATUS_OK != status) {
             ERROR_MSG("EOS buffer queing failed%d", status);
             unlock_output_stream_l(out);
             unlock_qap_stream_in(out);
             return -EINVAL;
          }

          //Drain the module input stream.
          /* Stream stop will trigger EOS and on EOS_EVENT received
             from callback DRAIN_READY command is sent */
          status = audio_extn_qap_stream_stop(out);

          if (status == 0) {
             //Setting state to stopping as client is expecting drain_ready event.
             set_stream_state_l(out, STOPPING);
          }
       }

       unlock_output_stream_l(out);
    } else {
       status = p_qap->hal_stream_ops.drain(stream, type);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG_VV("Exit");
    return status;
}


/* Flush the QAP module input stream. */
static int audio_extn_qap_stream_flush(struct stream_out *out)
{
    DEBUG_MSG("Output Stream %p", out);
    int ret = -EINVAL;
    struct qap_module *qap_mod = NULL;

    qap_mod = get_qap_module_for_input_stream_l(out);
    if (!qap_mod || !qap_mod->session_handle|| !out->qap_stream_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p) strm hndl(%p)",
                                qap_mod, qap_mod->session_handle, out->qap_stream_handle);
        return -EINVAL;
    }

    check_and_activate_output_thread(true);
    ret = qap_module_cmd(out->qap_stream_handle,
                            QAP_MODULE_CMD_FLUSH,
                            sizeof(QAP_MODULE_CMD_FLUSH),
                            NULL,
                            NULL,
                            NULL);
    if (QAP_STATUS_OK != ret) {
        ERROR_MSG("flush failed %d", ret);
        ret = -EINVAL;
    }
    check_and_activate_output_thread(false);

    DEBUG_MSG_VV("Exit");
    return ret;
}


/* Pause the QAP module input stream. */
static int qap_stream_pause_l(struct stream_out *out)
{
    struct qap_module *qap_mod = NULL;
    int ret = -EINVAL;
    float left = 0.0f;

    qap_mod = get_qap_module_for_input_stream_l(out);
    if (!qap_mod || !qap_mod->session_handle|| !out->qap_stream_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p) strm hndl(%p)",
            qap_mod, qap_mod->session_handle, out->qap_stream_handle);
        return -EINVAL;
    }

    qap_ms12_volume_easing_cmd(out, left, left, 1);

    check_and_activate_output_thread(true);
    ret = qap_module_cmd(out->qap_stream_handle,
                            QAP_MODULE_CMD_PAUSE,
                            sizeof(QAP_MODULE_CMD_PAUSE),
                            NULL,
                            NULL,
                            NULL);
    if (QAP_STATUS_OK != ret) {
        ERROR_MSG("pause failed %d", ret);
        return -EINVAL;
    }
    qap_mod->pause = true;
    check_and_activate_output_thread(false);

    return ret;
}


/* Flush the QAP input stream. */
static int qap_out_flush(struct audio_stream_out* stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    int status = 0;

    DEBUG_MSG("Output Stream %p", out);
    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
       lock_output_stream_l(out);

       if (!out->standby) {
           //If QAP passthrough is active then block the flush on module input streams.
           if (p_qap->passthrough_out) {
              pthread_mutex_lock(&p_qap->lock);
              //If flush is received for the QAP passthrough stream then call the primary HAL api.
              if (p_qap->passthrough_in == out) {
                 status = p_qap->passthrough_out->stream.flush(
                         (struct audio_stream_out *)p_qap->passthrough_out);
                 out->offload_state = OFFLOAD_STATE_IDLE;
              }
              pthread_mutex_unlock(&p_qap->lock);
           } else {
              //Flush the module input stream.
              status = audio_extn_qap_stream_flush(out);
           }
       }
       unlock_output_stream_l(out);
    } else {
        status = p_qap->hal_stream_ops.flush(stream);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG("Exit");
    return status;
}


/* Pause a QAP input stream. */
static int qap_out_pause(struct audio_stream_out* stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    int status = 0;
    DEBUG_MSG("Output Stream %p", out);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
       lock_output_stream_l(out);

       //If QAP passthrough is enabled then block the pause on module stream.
       if (p_qap->passthrough_out) {
          pthread_mutex_lock(&p_qap->lock);
          //If pause is received for QAP passthorugh stream then call the primary HAL api.
          if (p_qap->passthrough_in == out) {
              status = p_qap->passthrough_out->stream.pause(
                      (struct audio_stream_out *)p_qap->passthrough_out);
              out->offload_state = OFFLOAD_STATE_PAUSED;
          }
          pthread_mutex_unlock(&p_qap->lock);
       } else {
          //Pause the module input stream.
          status = qap_stream_pause_l(out);
       }

       unlock_output_stream_l(out);
    } else {
       status = p_qap->hal_stream_ops.pause(stream);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG_VV("Exit");
    return status;
}

static void close_qap_passthrough_stream_l()
{
    if (p_qap->passthrough_out != NULL) { //QAP pasthroug is enabled. Close it.
        pthread_mutex_lock(&p_qap->lock);
        adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                 (struct audio_stream_out *)(p_qap->passthrough_out));
        p_qap->passthrough_out = NULL;
        pthread_mutex_unlock(&p_qap->lock);

        if (p_qap->passthrough_in->qap_stream_handle) {
            qap_out_pause((struct audio_stream_out*)p_qap->passthrough_in);
            qap_out_flush((struct audio_stream_out*)p_qap->passthrough_in);
            qap_out_drain((struct audio_stream_out*)p_qap->passthrough_in,
                          (audio_drain_type_t)STREAM_CBK_EVENT_DRAIN_READY);
        }
    }
}

static int qap_out_standby(struct audio_stream *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct qap_module *qap_mod = NULL;
    int status = 0;
    int i;

    DEBUG_MSG("enter: stream (%p) usecase(%d: %s)",
          stream, out->usecase, use_case_table[out->usecase]);

    lock_qap_stream_in(out);
    qap_mod = get_qap_module_for_input_stream_l(out);
    if (!p_qap->bypass_enable) {
       lock_output_stream_l(out);
       DEBUG_MSG("Total bytes consumed %llu[frames] for usecase %s stream (%p) by MM Module",
                (unsigned long long)out->written, use_case_table[out->usecase], stream);

       //If QAP passthrough is active then block standby on all the input streams of QAP mm modules.
       if (p_qap->passthrough_out) {
          //If standby is received on QAP passthrough stream then forward it to primary HAL.
          if (p_qap->passthrough_in == out) {
              status = p_qap->passthrough_out->stream.common.standby(
                      (struct audio_stream *)p_qap->passthrough_out);
          }
       } else if (check_stream_state_l(out, RUN)) {
          //If QAP passthrough stream is not active then stop the QAP module stream.
          status = audio_extn_qap_stream_stop(out);

          if (status == 0) {
             //Setting state to stopped as client not expecting drain_ready event.
             set_stream_state_l(out, STOPPED);
          }
       } else if (check_stream_state_l(out, STOPPING)) {
          /* Handle scenario where out_drain followed by out_standby
             is invoked. Wait for change in stream state to make sure
             drain ready event is sent to client. */
          do {
             if (check_stream_state_l(out, STOPPED))
                break;
            /*If hdmi is disconnected and stream is stopped state
              release the lock and wait on drain_output_cond,
              qap_session callback acquires lock and moves stream
              to stopped state.
             */
             p_qap->wait_on_cond = 1;
             pthread_cond_wait(&qap_mod->drain_output_cond, &out->lock);
             p_qap->wait_on_cond = 0;
          } while(1);
       }

       if (p_qap->qap_output_block_handling) {
           for (i = 0; i < MAX_QAP_MODULE_IN; i++) {
               if (qap_mod->stream_in[i] != NULL &&
                   check_stream_state_l(qap_mod->stream_in[i], RUN)) {
                   break;
               }
           }
           if (i != MAX_QAP_MODULE_IN) {
               DEBUG_MSG("[%s] stream is still active.", use_case_table[qap_mod->stream_in[i]->usecase]);
           } else {
               lock_session_output(qap_mod);
               if (!p_qap->wait_on_cond)
                   qap_mod->is_session_output_active = false;
               unlock_session_output(qap_mod);
               DEBUG_MSG(" all the input streams are either closed or stopped(standby) block the MM module output");
           }
       }

       if (!out->standby) {
          out->standby = true;
       }

       unlock_output_stream_l(out);
    } else {
        status = p_qap->hal_stream_ops.common.standby(stream);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG_VV("Exit");
    return status;
}

/* Starts a QAP module stream. */
static int qap_stream_start_l(struct stream_out *out)
{
    int ret = 0;
    struct qap_module *qap_mod = NULL;

    DEBUG_MSG("Output Stream = %p", out);

    qap_mod = get_qap_module_for_input_stream_l(out);
    if ((!qap_mod) || (!qap_mod->session_handle)) {
        ERROR_MSG("QAP mod is not inited (%p) or session is not yet opened (%p) ",
            qap_mod, qap_mod->session_handle);
        return -EINVAL;
    }
    if (out->qap_stream_handle) {
        ret = qap_module_cmd(out->qap_stream_handle,
                             QAP_MODULE_CMD_START,
                             sizeof(QAP_MODULE_CMD_START),
                             NULL,
                             NULL,
                             NULL);
        if (ret != QAP_STATUS_OK) {
            ERROR_MSG("start failed");
            ret = -EINVAL;
        }
    } else
        ERROR_MSG("QAP stream not yet opened, drop this cmd");

    /* apply default volume at start of the qap stream */
    qap_ms12_volume_easing_cmd(out, 1.0, 1.0, 0);

    DEBUG_MSG("exit");
    return ret;

}

static int qap_start_output_stream(struct stream_out *out)
{
    int ret = 0;
    struct audio_device *adev = out->dev;

    if ((out->usecase < 0) || (out->usecase >= AUDIO_USECASE_MAX)) {
        ret = -EINVAL;
        DEBUG_MSG("Use case out of bounds sleeping for 500ms");
        usleep(50000);
        return ret;
    }

    ALOGD("%s: enter: stream(%p)usecase(%d: %s) devices(%#x)",
          __func__, &out->stream, out->usecase, use_case_table[out->usecase],
          out->devices);

    if (CARD_STATUS_OFFLINE == out->card_status ||
        CARD_STATUS_OFFLINE == adev->card_status) {
        ALOGE("%s: sound card is not active/SSR returning error", __func__);
        ret = -EIO;
        usleep(50000);
        return ret;
    }

    return qap_stream_start_l(out);
}

/* Sends input buffer to the QAP MM module. */
static int qap_module_write_input_buffer(struct stream_out *out, const void *buffer, int bytes)
{
    int ret = -EINVAL;
    struct qap_module *qap_mod = NULL;
    qap_audio_buffer_t buff;

    qap_mod = get_qap_module_for_input_stream_l(out);
    if ((!qap_mod) || (!qap_mod->session_handle) || (!out->qap_stream_handle)) {
        return ret;
    }

    //If data received on associated stream when all other stream are stopped then drop the data.
    if (out == qap_mod->stream_in[QAP_IN_ASSOC] && !is_any_stream_running_l(qap_mod))
        return bytes;

    memset(&buff, 0, sizeof(buff));
    buff.common_params.offset = 0;
    buff.common_params.size = bytes;
    buff.common_params.data = (void *) buffer;
    buff.common_params.timestamp = QAP_BUFFER_NO_TSTAMP;
    buff.buffer_parms.input_buf_params.flags = QAP_BUFFER_NO_TSTAMP;
    DEBUG_MSG_VV("calling module process for usecase %s stream %p with bytes %d %p",
               use_case_table[out->usecase], out, bytes, buffer);
    ret = qap_module_process(out->qap_stream_handle, &buff);

    if(ret > 0) set_stream_state_l(out, RUN);

    return ret;
}

static ssize_t qap_out_write(struct audio_stream_out *stream, const void *buffer, size_t bytes)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct audio_device *adev = out->dev;
    ssize_t ret = 0;
    struct qap_module *qap_mod = NULL;

    DEBUG_MSG_VV("bytes = %d, usecase[%s] and flags[%x] for handle[%p] bypass mode[%d]",
          (int)bytes, use_case_table[out->usecase], out->flags, out, p_qap->bypass_enable);


    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
        lock_output_stream_l(out);
         check_and_activate_output_thread(true);

        // If QAP passthrough is active then block writing data to QAP mm module.
        if (p_qap->passthrough_out) {
           //If write is received for the QAP passthrough stream then send the buffer to primary HAL.
           if (p_qap->passthrough_in == out) {
               ret = p_qap->passthrough_out->stream.write(
                    (struct audio_stream_out *)(p_qap->passthrough_out),
                    buffer,
                    bytes);
               if (ret > 0) out->standby = false;
           }
           unlock_output_stream_l(out);
           unlock_qap_stream_in(out);
           return ret;
        } else if (out->standby) {
           qap_mod = get_qap_module_for_input_stream_l(out);
           pthread_mutex_lock(&adev->lock);
           ret = qap_start_output_stream(out);
           pthread_mutex_unlock(&adev->lock);
           if (ret == 0) {
              out->standby = false;
              if(p_qap->qap_output_block_handling) {
                 lock_session_output(qap_mod);
                 if (!qap_mod->is_session_output_active && !p_qap->qap_active_api_count) {
                     pthread_cond_signal(&qap_mod->session_output_cond);
                 }
                 qap_mod->is_session_output_active = true;
                 unlock_session_output(qap_mod);
              }
           } else {
              goto exit;
           }
        }

        if ((adev->is_channel_status_set == false) && (out->devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)) {
           audio_utils_set_hdmi_channel_status(out, (char *)buffer, bytes);
           adev->is_channel_status_set = true;
        }

        ret = qap_module_write_input_buffer(out, buffer, bytes);
        DEBUG_MSG_VV("Bytes consumed [%d] by MM Module for usecase %s stream %p",
                    (int)ret, use_case_table[out->usecase], out);

        if (ret >= 0) {
           out->written += ret / ((popcount(out->channel_mask) * sizeof(short)));
        }

        exit:
           unlock_output_stream_l(out);
           check_and_activate_output_thread(false);

           if (ret < 0) {
              if (ret == -EAGAIN) {
                 DEBUG_MSG_VV("No space available to consume bytes, post msg to cb thread");
                 bytes = 0;
              } else if (ret == -ENOMEM || ret == -EPERM) {
                 if (out->pcm)
                    ERROR_MSG("error %d, %s", (int)ret, pcm_get_error(out->pcm));
                    qap_out_standby(&out->stream.common);
                    DEBUG_MSG("SLEEP for 100sec");
                    usleep(bytes * 1000000
                          / audio_stream_out_frame_size(stream)
                          / out->stream.common.get_sample_rate(&out->stream.common));
              }
           } else if (ret < (ssize_t)bytes) {
              //partial buffer copied to the module.
              DEBUG_MSG_VV("Not enough space available to consume all the bytes");
              bytes = ret;
           }
    } else {
        ret = p_qap->hal_stream_ops.write(stream, buffer, bytes);
        bytes = ret;
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG_VV("Exit");
    return bytes;
}

/* Gets PCM offload buffer size for a given config. */
static uint32_t qap_get_pcm_offload_buffer_size(audio_offload_info_t* info,
                                                uint32_t samples_per_frame)
{
    uint32_t fragment_size = 0;

    fragment_size = (samples_per_frame * (info->bit_width >> 3) * popcount(info->channel_mask));

    if (fragment_size < MIN_PCM_OFFLOAD_FRAGMENT_SIZE)
        fragment_size = MIN_PCM_OFFLOAD_FRAGMENT_SIZE;
    else if (fragment_size > MAX_PCM_OFFLOAD_FRAGMENT_SIZE)
        fragment_size = MAX_PCM_OFFLOAD_FRAGMENT_SIZE;

    // To have same PCM samples for all channels, the buffer size requires to
    // be multiple of (number of channels * bytes per sample)
    // For writes to succeed, the buffer must be written at address which is multiple of 32
    fragment_size = ALIGN(fragment_size,
                          ((info->bit_width >> 3) * popcount(info->channel_mask) * 32));

    ALOGI("Qap PCM offload Fragment size is %d bytes", fragment_size);

    return fragment_size;
}

static uint32_t qap_get_pcm_offload_input_buffer_size(audio_offload_info_t* info)
{
    return qap_get_pcm_offload_buffer_size(info, MS12_PCM_IN_FRAGMENT_SIZE);
}

static uint32_t qap_get_pcm_offload_output_buffer_size(struct qap_module *qap_mod,
                                                audio_offload_info_t* info)
{
    return qap_get_pcm_offload_buffer_size(info, get_pcm_output_buffer_size_samples_l(qap_mod));
}

/* Gets buffer latency in samples. */
static int get_buffer_latency(struct stream_out *out, uint32_t buffer_size, uint32_t *latency)
{
    unsigned long int samples_in_one_encoded_frame;
    unsigned long int size_of_one_encoded_frame;

    switch (out->format) {
        case AUDIO_FORMAT_AC3:
            samples_in_one_encoded_frame = DD_FRAME_SIZE;
            size_of_one_encoded_frame = DD_ENCODER_OUTPUT_SIZE;
        break;
        case AUDIO_FORMAT_E_AC3:
        case AUDIO_FORMAT_E_AC3_JOC:
            samples_in_one_encoded_frame = DDP_FRAME_SIZE;
            size_of_one_encoded_frame = DDP_ENCODER_OUTPUT_SIZE;
        break;
        case AUDIO_FORMAT_DTS:
            samples_in_one_encoded_frame = DTS_FRAME_SIZE;
            size_of_one_encoded_frame = DTS_ENCODER_OUTPUT_SIZE;
        break;
        case AUDIO_FORMAT_DTS_HD:
            samples_in_one_encoded_frame = DTSHD_FRAME_SIZE;
            size_of_one_encoded_frame = DTSHD_ENCODER_OUTPUT_SIZE;
        break;
        case AUDIO_FORMAT_PCM_16_BIT:
            samples_in_one_encoded_frame = 1;
            size_of_one_encoded_frame = ((out->bit_width) >> 3) * popcount(out->channel_mask);
        break;
        default:
            *latency = 0;
            return (-EINVAL);
    }

    *latency = ((buffer_size * samples_in_one_encoded_frame) / size_of_one_encoded_frame);
    return 0;
}

/* Returns the ms12 graph latency from lookup table in msec.
 * pass ms12_latency_value pointer to get ms12 latency
 */
int get_ms12_graph_latency(struct stream_out *out, int *ms12_graph_latency)
{
    int ret = 0;

    if (NULL == out || NULL == out->qap_stream_handle || NULL == ms12_graph_latency) {
        fprintf(stderr, "!!!! Error Stream config is NULL \n");
        return -EINVAL;
    }

    uint32_t param_id = MS12_STREAM_GET_LATENCY;

    ret = qap_module_cmd(out->qap_stream_handle, QAP_MODULE_CMD_GET_PARAM, sizeof(param_id), &param_id, NULL, ms12_graph_latency);

    return ret;
}

/* Returns the number of frames rendered to outside observer. */
static int qap_get_rendered_frames(struct stream_out *out, uint64_t *frames)
{
    int ret = 0, i;
    unsigned long long position = 0;
    int module_latency = 0;
    uint32_t kernel_latency = 0;
    uint32_t dsp_latency = 0;
    uint64_t signed_frames = 0;
    int ms12_latency = 0;
    struct qap_module *qap_mod = NULL;
    int ms12_latency_sample = 0;
    int ms12_latency_addon = 0;

    qap_mod = get_qap_module_for_input_stream_l(out);
    if (!qap_mod || !qap_mod->session_handle|| !out->qap_stream_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p) strm hndl(%p)",
            qap_mod, qap_mod->session_handle, out->qap_stream_handle);
        return -EINVAL;
    }

    if (property_get_bool("vendor.audio.qap.ms12_latency_realtime", false)) {
        ms12_latency_addon = property_get_int32("vendor.audio.qap.ms12_latency_addon", 0);

        get_ms12_graph_latency(out, &ms12_latency);
        ms12_latency_sample = 48 * (ms12_latency + ms12_latency_addon);
    } else {
        ms12_latency_sample = MS12_LATENCY;
    }

     module_latency = ms12_latency_sample;
    //Get kernel Latency
    for (i = MAX_QAP_MODULE_OUT - 1; i >= 0; i--) {
        if (qap_mod->stream_out[i] == NULL) {
            continue;
        } else {
            unsigned int num_fragments = qap_mod->stream_out[i]->compr_config.fragments;
            uint32_t fragment_size = qap_mod->stream_out[i]->compr_config.fragment_size;
            uint32_t kernel_buffer_size = num_fragments * fragment_size;
            get_buffer_latency(qap_mod->stream_out[i], kernel_buffer_size, &kernel_latency);
            break;
        }
    }

    if (i < 0) {
        ALOGI("%s: No streams are active, it seems qap is in standby mode", __func__);
        return -ENODATA;
    }

    //Get DSP latency
    if ((qap_mod->stream_out[QAP_OUT_OFFLOAD] != NULL)
        || (qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH] != NULL)) {
        unsigned int sample_rate = 0;
        audio_usecase_t platform_latency = 0;

        if (qap_mod->stream_out[QAP_OUT_OFFLOAD])
            sample_rate = qap_mod->stream_out[QAP_OUT_OFFLOAD]->sample_rate;
        else if (qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH])
            sample_rate = qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->sample_rate;

        if (qap_mod->stream_out[QAP_OUT_OFFLOAD])
            platform_latency =
                platform_render_latency(qap_mod->stream_out[QAP_OUT_OFFLOAD]->usecase);
        else
            platform_latency =
                platform_render_latency(qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->usecase);

        dsp_latency = (platform_latency * sample_rate) / 1000000LL;
    } else if (qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH] != NULL) {
        unsigned int sample_rate = 0;

        sample_rate = qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]->sample_rate; //TODO: How this sample rate can be used?
        dsp_latency = (COMPRESS_OFFLOAD_PLAYBACK_LATENCY * sample_rate) / 1000;
    }

    // MM Module Latency + Kernel Latency + DSP Latency
    if ( audio_extn_bt_hal_get_output_stream(qap_mod->bt_hdl) != NULL) {
        out->platform_latency = module_latency + audio_extn_bt_hal_get_latency(qap_mod->bt_hdl);
    } else {
        out->platform_latency = (uint32_t)module_latency + kernel_latency + dsp_latency;
    }

    DEBUG_MSG_VV("total platform latency %d msec MS12(%d)+kernel_latency(%d)+dsp_latency(%d)",
        out->platform_latency, module_latency, kernel_latency, dsp_latency);

    if (out->format == AUDIO_FORMAT_PCM_16_BIT) {
        *frames = 0;
        if(out->written > out->platform_latency) {
           signed_frames = out->written - out->platform_latency;
           *frames = signed_frames;
        }
    } else {
        uint32_t param_id = MS12_STREAM_GET_POSITION;
        check_and_activate_output_thread(true);
        ret = qap_module_cmd(out->qap_stream_handle,
                                    QAP_MODULE_CMD_GET_PARAM,
                                    sizeof(param_id),
                                    &param_id,
                                    NULL,
                                    &position);
        DEBUG_MSG_VV("Frames returned by MS12(%llu)", position);
        if (ret >= 0) {
            *frames = position;
            if(position > out->platform_latency) {
               signed_frames = position - out->platform_latency;
               *frames = signed_frames;
            } else
               *frames = 0;
        } else 
            ret = -EINVAL;
        check_and_activate_output_thread(false);
    }
    return ret;
}

static int qap_out_get_render_position(const struct audio_stream_out *stream,
                                   uint32_t *dsp_frames)
{
    struct stream_out *out = (struct stream_out *)stream;
    int ret = 0;
    uint64_t frames=0;
    struct qap_module* qap_mod = NULL;

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
        qap_mod = get_qap_module_for_input_stream_l(out);
        if (!qap_mod) {
            ret = out->stream.get_render_position(stream, dsp_frames);
            DEBUG_MSG("non qap_MOD DSP FRAMES %d", (int)dsp_frames);
            unlock_qap_stream_in(out);
            return ret;
        }

        if (p_qap->passthrough_out) {
            pthread_mutex_lock(&p_qap->lock);
            ret = p_qap->passthrough_out->stream.get_render_position((struct audio_stream_out *)p_qap->passthrough_out,
                          dsp_frames);
            pthread_mutex_unlock(&p_qap->lock);
            DEBUG_MSG("PASS THROUGH DSP FRAMES %p", dsp_frames);
            unlock_qap_stream_in(out);
            return ret;
        }
        frames=*dsp_frames;
        ret = qap_get_rendered_frames(out, &frames);
        *dsp_frames = (uint32_t)frames;
        DEBUG_MSG_VV("DSP FRAMES %ud for out(%p)", *dsp_frames, out);
    } else {
       ret = p_qap->hal_stream_ops.get_render_position(stream, dsp_frames);
    }
    unlock_qap_stream_in(out);
    return ret;
}

static int qap_out_get_presentation_position(const struct audio_stream_out *stream,
                                             uint64_t *frames,
                                             struct timespec *timestamp)
{
    struct stream_out *out = (struct stream_out *)stream;
    int ret = 0;

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
    //If QAP passthorugh output stream is active.
    if (p_qap->passthrough_out) {
        if (p_qap->passthrough_in == out) {
            //If api is called for QAP passthorugh stream then call the primary HAL api to get the position.
            pthread_mutex_lock(&p_qap->lock);
            ret = p_qap->passthrough_out->stream.get_presentation_position(
                    (struct audio_stream_out *)p_qap->passthrough_out,
                    frames,
                    timestamp);
            pthread_mutex_unlock(&p_qap->lock);
        } else {
            //If api is called for other stream then return zero frames.
            *frames = 0;
            clock_gettime(CLOCK_MONOTONIC, timestamp);
        }
        unlock_qap_stream_in(out);
        return ret;
    }

    ret = qap_get_rendered_frames(out, frames);
    clock_gettime(CLOCK_MONOTONIC, timestamp);

    DEBUG_MSG_VV("frames(%llu) for out(%p)", (unsigned long long)*frames, out);
    } else {
       ret = p_qap->hal_stream_ops.get_presentation_position(stream, frames, timestamp);
    }
    unlock_qap_stream_in(out);
    return ret;
}

static uint32_t qap_out_get_latency(const struct audio_stream_out *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    uint32_t latency = 0;
    struct qap_module *qap_mod = NULL;
    DEBUG_MSG_VV("Output Stream %p", out);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
        qap_mod = get_qap_module_for_input_stream_l(out);
        if (!qap_mod) {
            unlock_qap_stream_in(out);
            return 0;
        }

        //If QAP passthrough is active then block the get latency on module input streams.
        if (p_qap->passthrough_out) {
            pthread_mutex_lock(&p_qap->lock);
        //If get latency is called for the QAP passthrough stream then call the primary HAL api.
        if (p_qap->passthrough_in == out) {
            latency = p_qap->passthrough_out->stream.get_latency(
                      (struct audio_stream_out *)p_qap->passthrough_out);
        }
        pthread_mutex_unlock(&p_qap->lock);
        } else {
           if (is_offload_usecase(out->usecase)) {
               latency = COMPRESS_OFFLOAD_PLAYBACK_LATENCY;
           } else {
               uint32_t sample_rate = 0;
               latency = QAP_MODULE_PCM_INPUT_BUFFER_LATENCY; //Input latency

               if (qap_mod->stream_out[QAP_OUT_OFFLOAD])
                   sample_rate = qap_mod->stream_out[QAP_OUT_OFFLOAD]->sample_rate;
               else if (qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH])
                   sample_rate = qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->sample_rate;

               if (sample_rate) {
                   latency += (get_pcm_output_buffer_size_samples_l(qap_mod) * 1000) / out->sample_rate;
               }
           }

           if ( audio_extn_bt_hal_get_output_stream(qap_mod->bt_hdl) != NULL) {
               if (is_offload_usecase(out->usecase)) {
                   latency = audio_extn_bt_hal_get_latency(qap_mod->bt_hdl) +
                   QAP_COMPRESS_OFFLOAD_PROCESSING_LATENCY;
               } else {
                   latency = audio_extn_bt_hal_get_latency(qap_mod->bt_hdl) +
                   QAP_PCM_OFFLOAD_PROCESSING_LATENCY;
               }
           }
        }
    } else {
        latency = p_qap->hal_stream_ops.get_latency(stream);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG_VV("Latency %d", latency);
    return latency;
}

static int qap_session_cmd_l(audio_session_handle_t session_handle, qap_session_outputs_config_t *outputs_config){
    int status = 0;

    if (ec.ch){
        outputs_config->num_output += 1;
        outputs_config->output_config[outputs_config->num_output - 1].format = QAP_AUDIO_FORMAT_PCM_16_BIT;
        outputs_config->output_config[outputs_config->num_output - 1].id = AUDIO_DEVICE_OUT_PROXY;
        outputs_config->output_config[outputs_config->num_output - 1].channels = ec.ch;

        DEBUG_MSG(" Enabling PROXY(pcm out) from MS12 wrapper outputid = 0x%x",
                    outputs_config->output_config[outputs_config->num_output - 1].id);

    }

    no_of_outputs = outputs_config->num_output;
    check_and_activate_output_thread(true);
    status = qap_session_cmd(session_handle, QAP_SESSION_CMD_SET_OUTPUTS,
                             sizeof(qap_session_outputs_config_t),
                             outputs_config, NULL, NULL);
    check_and_activate_output_thread(false);

    return status;
}

static bool qap_check_and_get_compressed_device_format(int device, int *format)
{
    switch (device) {
        case (AUDIO_DEVICE_OUT_AUX_DIGITAL | QAP_AUDIO_FORMAT_AC3):
            *format = AUDIO_FORMAT_AC3;
            return true;
        case (AUDIO_DEVICE_OUT_AUX_DIGITAL | QAP_AUDIO_FORMAT_EAC3):
            *format = AUDIO_FORMAT_E_AC3;
            return true;
        case (AUDIO_DEVICE_OUT_AUX_DIGITAL | QAP_AUDIO_FORMAT_DTS):
            *format = AUDIO_FORMAT_DTS;
            return true;
        default:
            return false;
    }
}

static void set_out_stream_channel_map(struct stream_out *out, qap_output_config_t * media_fmt)
{
    if (media_fmt == NULL || out == NULL) {
        return;
    }
    struct audio_out_channel_map_param chmap = {0,{0}};
    int i = 0;
    chmap.channels = media_fmt->channels;
    for (i = 0; i < chmap.channels && i < AUDIO_CHANNEL_COUNT_MAX && i < AUDIO_QAF_MAX_CHANNELS;
            i++) {
        chmap.channel_map[i] = media_fmt->ch_map[i];
    }
    audio_extn_utils_set_channel_map(out, &chmap);
}

bool audio_extn_is_qap_enabled()
{
    bool prop_enabled = false;
    char value[PROPERTY_VALUE_MAX] = {0};
    property_get("vendor.audio.qap.enabled", value, NULL);
    prop_enabled = atoi(value) || !strncmp("true", value, 4);
    DEBUG_MSG("%d", prop_enabled);
    return (prop_enabled);
}

void static qap_close_all_output_streams(struct qap_module *qap_mod)
{
    int i =0;
    struct stream_out *stream_out = NULL;
    DEBUG_MSG("Entry");

    for (i = 0; i < MAX_QAP_MODULE_OUT; i++) {
        stream_out = qap_mod->stream_out[i];
        if (stream_out != NULL) {
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev, (struct audio_stream_out *)stream_out);
            DEBUG_MSG("Closed outputenum=%d session 0x%x %s, with total bytes of %llu consumed by hal",
                    i, (int)stream_out, use_case_table[stream_out->usecase],
                    qap_mod->qap_output_bytes_written[i]);
            qap_mod->stream_out[i] = NULL;
            qap_mod->qap_output_bytes_written[i] = 0;
        }
        memset(&qap_mod->session_outputs_config.output_config[i], 0, sizeof(qap_session_outputs_config_t));
        qap_mod->is_media_fmt_changed[i] = false;
        delay_event_fired = false;
    }
    if (ec.proxy_out_ptr) {
        insert_wav_header(ec.proxy_out_ptr, NULL);
        fclose(ec.proxy_out_ptr);
        DEBUG_MSG("File %s CLOSED, total bytes written %lld", ec.file_dump,
                       ec.total_bytes_written);
        ec.proxy_out_ptr = NULL;
        ec.total_bytes_written = 0;
    }
    p_qap->passthrough_enabled = false;
    p_qap->mch_pcm_hdmi_enabled = false;

    DEBUG_MSG("exit");
}

/* Call back function for mm module. */
static void qap_session_callback(qap_session_handle_t session_handle __unused,
                                  void *prv_data,
                                 qap_callback_event_t event_id,
                                  int size,
                                  void *data)
{

    /*
     For SPKR:
     1. Open pcm device if device_id passed to it SPKR and write the data to
     pcm device

     For HDMI
     1.Open compress device for HDMI(PCM or AC3) based on current hdmi o/p format and write
     data to the HDMI device.
     */
    int ret;
    int cb_array_index = 0;
    static int abs_index = 0;
    int i;
    uint32_t bytes_written = 0;
    audio_output_flags_t flags;
    struct qap_module* qap_mod = (struct qap_module*)prv_data;
    struct audio_stream_out *bt_stream = NULL;
    int format;
    int8_t *data_buffer_p = NULL;
    uint32_t buffer_size = 0;
    bool need_to_recreate_stream = false;
    struct audio_config config;
    qap_output_config_t *new_conf = NULL;
    qap_audio_buffer_t *buffer = (qap_audio_buffer_t *) data;
    uint32_t device = 0;

    if(p_qap->qap_output_block_handling) {
        lock_session_output(qap_mod);
        if (!qap_mod->is_session_output_active && !p_qap->qap_active_api_count) {
            qap_close_all_output_streams(qap_mod);
            DEBUG_MSG("disabling MM module output by blocking the output thread");
            /*is_session_output_active & qap_active_api_count used as predicate to safe gaurd against the spurious wakeups */
            while(!qap_mod->is_session_output_active && !p_qap->qap_active_api_count)
                pthread_cond_wait(&qap_mod->session_output_cond, &qap_mod->session_output_lock);
            DEBUG_MSG("MM module output Enabled, output thread active");
        }
        unlock_session_output(qap_mod);
    }

    if (qap_mod->is_session_closing) {
        DEBUG_MSG("Dropping event as session is closing."
                "Event = 0x%X, Bytes to write %d", event_id, size);
        return;
    }

    pthread_mutex_lock(&p_qap->lock);

    if (event_id == QAP_CALLBACK_EVENT_OUTPUT_CFG_CHANGE) {
        new_conf = &buffer->buffer_parms.output_buf_params.output_config;
        /* Assign QAP device id since new conf has ms12 deviceid */
        new_conf->id = buffer->buffer_parms.output_buf_params.output_id;
        qap_output_config_t *cached_conf = NULL;
        int index = -1;

        DEBUG_MSG("Received QAP_CALLBACK_EVENT_OUTPUT_CFG_CHANGE event for output id=0x%x",
                buffer->buffer_parms.output_buf_params.output_id);

        DEBUG_MSG("sample rate=%d bitwidth=%d format = %d channels =0x%x",
            new_conf->sample_rate,
            new_conf->bit_width,
            new_conf->format,
            new_conf->channels);

        if ( (uint32_t)size < sizeof(qap_output_config_t)) {
            ERROR_MSG("Size is not proper for the event AUDIO_OUTPUT_MEDIA_FORMAT_EVENT.");
            return ;
        }

        index = get_media_fmt_array_index_for_output_id_l(qap_mod, buffer->buffer_parms.output_buf_params.output_id);

        if (index >= 0) {
            cached_conf = &qap_mod->session_outputs_config.output_config[index];
        } else if (index < 0 && qap_mod->new_out_format_index < MAX_QAP_MODULE_OUT) {
            index = qap_mod->new_out_format_index;
            cached_conf = &qap_mod->session_outputs_config.output_config[index];
            qap_mod->new_out_format_index++;
        }

        if (cached_conf == NULL) {
            ERROR_MSG("Maximum output from a QAP module is reached. Can not process new output.");
            return ;
        }

        if (memcmp(cached_conf, new_conf, sizeof(qap_output_config_t)) != 0) {
            memcpy(cached_conf, new_conf, sizeof(qap_output_config_t));
            qap_mod->is_media_fmt_changed[index] = true;
        }
    } else if (event_id == QAP_CALLBACK_EVENT_DATA) {
        data_buffer_p = (int8_t*)buffer->common_params.data+buffer->common_params.offset;
        buffer_size = buffer->common_params.size;
        device = buffer->buffer_parms.output_buf_params.output_id;

        abs_index++;
        cb_array_index =abs_index%no_of_outputs;

        DEBUG_MSG_VV("Received QAP_CALLBACK_EVENT_DATA event buff size(%d) for outputid=0x%x",
            buffer_size, buffer->buffer_parms.output_buf_params.output_id);

        cb_data_array[abs_index-1].buff_size = buffer->common_params.size;
        memcpy(cb_data_array[abs_index-1].buff_ptr, (int8_t*)buffer->common_params.data+buffer->common_params.offset, buffer->common_params.size);
        cb_data_array[abs_index-1].dev_id = buffer->buffer_parms.output_buf_params.output_id;


        if(cb_array_index == 0) {
            DEBUG_MSG_VV("callback (%d) recieved out of (%d) data cached at %p", abs_index, no_of_outputs, cb_data_array[abs_index-1].buff_ptr);
            DEBUG_MSG_VV("all callbacks recieved process them in loop");
            abs_index = 0;
        } else {
            DEBUG_MSG_VV("callback (%d) recieved out of (%d)", cb_array_index, no_of_outputs);
            pthread_mutex_unlock(&p_qap->lock);
            return;
        }

        for(i=0; i<no_of_outputs; i++) {
            data_buffer_p = cb_data_array[i].buff_ptr ;
            buffer_size = cb_data_array[i].buff_size ;
            device = cb_data_array[i].dev_id;

            DEBUG_MSG_VV("callback (%d) buffer_size (%d) data_buffer_p(%p) device 0x%x",i, buffer_size, data_buffer_p, device);

            if (buffer && buffer->common_params.data) {
                int index = -1;
                /* Default config initialization. */
                config.sample_rate = config.offload_info.sample_rate = QAP_OUTPUT_SAMPLING_RATE;
                config.offload_info.version = AUDIO_INFO_INITIALIZER.version;
                config.offload_info.size = AUDIO_INFO_INITIALIZER.size;
                config.format = config.offload_info.format = AUDIO_FORMAT_PCM_16_BIT;
                config.offload_info.bit_width = CODEC_BACKEND_DEFAULT_BIT_WIDTH;
                config.offload_info.channel_mask = config.channel_mask = AUDIO_CHANNEL_OUT_STEREO;
                if (qap_check_and_get_compressed_device_format(device, &format))
                    config.offload_info.channel_mask = config.channel_mask = AUDIO_CHANNEL_OUT_5POINT1;

                need_to_recreate_stream = false;
                index = get_media_fmt_array_index_for_output_id_l(qap_mod, device);
                if (index > -1 && qap_mod->is_media_fmt_changed[index]) {
                    DEBUG_MSG("FORMAT changed of device 0x%x, recreate stream", device);
                    need_to_recreate_stream = true;
                    qap_mod->is_media_fmt_changed[index] = false;

                    qap_output_config_t *new_config = &qap_mod->session_outputs_config.output_config[index];

                    config.sample_rate = config.offload_info.sample_rate = new_config->sample_rate;
                    config.offload_info.version = AUDIO_INFO_INITIALIZER.version;
                    config.offload_info.size = AUDIO_INFO_INITIALIZER.size;
                    config.offload_info.bit_width = new_config->bit_width;

                    if (new_config->format == QAP_AUDIO_FORMAT_PCM_16_BIT) {
                        if (new_config->bit_width == 16)
                            config.format = config.offload_info.format = AUDIO_FORMAT_PCM_16_BIT;
                        else if (new_config->bit_width == 24)
                            config.format = config.offload_info.format = AUDIO_FORMAT_PCM_24_BIT_PACKED;
                        else
                            config.format = config.offload_info.format = AUDIO_FORMAT_PCM_32_BIT;
                    } else if (new_config->format  == QAP_AUDIO_FORMAT_AC3)
                        config.format = config.offload_info.format = AUDIO_FORMAT_AC3;
                    else if (new_config->format  == QAP_AUDIO_FORMAT_EAC3)
                        config.format = config.offload_info.format = AUDIO_FORMAT_E_AC3;
                    else if (new_config->format  == QAP_AUDIO_FORMAT_DTS)
                        config.format = config.offload_info.format = AUDIO_FORMAT_DTS;

                    device |= (new_config->format & AUDIO_FORMAT_MAIN_MASK);

                    config.channel_mask = audio_channel_out_mask_from_count(new_config->channels);
                    config.offload_info.channel_mask = config.channel_mask;
                    DEBUG_MSG("sample rate=%d bitwidth=%d format = %d channels=%d channel_mask=%d device =0x%x",
                        config.sample_rate,
                        config.offload_info.bit_width,
                        config.offload_info.format,
                        new_config->channels,
                        config.channel_mask,
                        device);
                    memcpy(&qap_mod->session_outputs_config.output_config[0],
                          &qap_mod->session_outputs_config.output_config[index],
                          sizeof(qap_mod->session_outputs_config.output_config[index]));
                    memset(&qap_mod->session_outputs_config.output_config[index], 0,
                          sizeof(qap_mod->session_outputs_config.output_config[index]));
                    qap_mod->new_out_format_index = 1;
                }
            }

            if (device == AUDIO_DEVICE_OUT_PROXY) {
                if (need_to_recreate_stream && ec.proxy_out_ptr) {
                    insert_wav_header(ec.proxy_out_ptr, NULL);
                    fclose(ec.proxy_out_ptr);
                    DEBUG_MSG("File %s CLOSED, total bytes written %lld", ec.file_dump, ec.total_bytes_written);
                    ec.total_bytes_written = 0;
                    ec.proxy_out_ptr = NULL;
                }
                if (!need_to_recreate_stream) {
                    config.channel_mask = audio_channel_out_mask_from_count(ec.ch);
                }
                if (ec.proxy_out_ptr == NULL && ec.ch) {
                    memset(ec.file_dump, 0, 40);
                    snprintf(ec.file_dump, 40, "/data/vendor/misc/audio/ecref%d.wav", audio_channel_count_from_out_mask(config.channel_mask));
                    ec.proxy_out_ptr = fopen(ec.file_dump, "w+");
                    if (!ec.proxy_out_ptr)
                        ERROR_MSG("unable to open proxy dump file %s", ec.file_dump);
                    else {
                        DEBUG_MSG("Proxy dump file opened successfully at %s", ec.file_dump);
                        insert_wav_header(ec.proxy_out_ptr, &config);
                    }
                }
                if(ec.proxy_out_ptr) {
                    bytes_written = fwrite(data_buffer_p, 1, buffer_size, ec.proxy_out_ptr);
                    if (bytes_written != buffer_size) {
                        ERROR_MSG("unable to write to proxy dump");
                        insert_wav_header(ec.proxy_out_ptr, NULL);
                        fclose(ec.proxy_out_ptr);
                        ec.proxy_out_ptr = NULL;
                    }
                    else
                        ec.total_bytes_written += bytes_written;
                }
                continue;
            }

            if (p_qap->passthrough_out != NULL) {
                //If QAP passthrough is active then all the module output will be dropped.
                pthread_mutex_unlock(&p_qap->lock);
                DEBUG_MSG("QAP-PSTH is active, DROPPING DATA!");
                return;
            }

            if (qap_check_and_get_compressed_device_format(device, &format)) {
                /*
                 * CASE 1: Transcoded output of mm module.
                 * If HDMI is not connected then drop the data.
                 * Only one HDMI output can be supported from all the mm modules of QAP.
                 * Multi-Channel PCM HDMI output streams will be closed from all the mm modules.
                 * If transcoded output of other module is already enabled then this data will be dropped.
                 */

                if (!p_qap->hdmi_connect) {
                    DEBUG_MSG("HDMI not connected, DROPPING DATA!");
                    close_all_hdmi_output_l();
                    close_qap_passthrough_stream_l();
                    pthread_mutex_unlock(&p_qap->lock);
                    return;
                }

                //Closing all the PCM HDMI output stream from QAP.
                close_all_pcm_hdmi_output_l();

                /* If Media format was changed for this stream then need to re-create the stream. */
                if (need_to_recreate_stream && qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]) {
                    DEBUG_MSG("closing Transcode Passthrough session ox%x",
                        (int)qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]);
                    adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                             (struct audio_stream_out *)(qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]));
                    qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH] = NULL;
                    p_qap->passthrough_enabled = false;
                }

                if (!p_qap->passthrough_enabled
                    && !(qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH])) {

                    audio_devices_t devices;

                    config.format = config.offload_info.format = format;

                    flags = (AUDIO_OUTPUT_FLAG_NON_BLOCKING
                             | AUDIO_OUTPUT_FLAG_COMPRESS_OFFLOAD
                             | AUDIO_OUTPUT_FLAG_DIRECT
                             | AUDIO_OUTPUT_FLAG_COMPRESS_PASSTHROUGH);
                    devices = AUDIO_DEVICE_OUT_AUX_DIGITAL;

                    DEBUG_MSG("Opening Transcode Passthrough out(outputenum=%d) session 0x%x with below params",
                            QAP_OUT_TRANSCODE_PASSTHROUGH,
                            (int)qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]);

                    DEBUG_MSG("sample rate=%d bitwidth=%d format = 0x%x channel mask=0x%x flags=0x%x device =0x%x",
                        config.sample_rate,
                        config.offload_info.bit_width,
                        config.offload_info.format,
                        config.offload_info.channel_mask,
                        flags,
                        devices);

                    ret = adev_open_output_stream((struct audio_hw_device *)p_qap->adev,
                                                  QAP_DEFAULT_COMPR_PASSTHROUGH_HANDLE,
                                                  devices,
                                                  flags,
                                                  &config,
                                                  (struct audio_stream_out **)&(qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]),
                                                  NULL);
                    if (ret < 0) {
                        ERROR_MSG("Failed opening Transcode Passthrough out(outputenum=%d) session 0x%x",
                                QAP_OUT_TRANSCODE_PASSTHROUGH,
                                (int)qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]);
                        pthread_mutex_unlock(&p_qap->lock);
                        return;
                    } else
                        DEBUG_MSG("Opened Transcode Passthrough out(outputenum=%d) session 0x%x",
                                QAP_OUT_TRANSCODE_PASSTHROUGH,
                                (int)qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]);


                    if (format == AUDIO_FORMAT_E_AC3) {
                        qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]->compr_config.fragment_size =
                                COMPRESS_PASSTHROUGH_DDP_FRAGMENT_SIZE;
                    }
                    qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]->compr_config.fragments =
                            COMPRESS_OFFLOAD_NUM_FRAGMENTS;

                    p_qap->passthrough_enabled = true;
                }

                if (qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]) {
                    DEBUG_MSG_VV("Writing Bytes(%d) to QAP_OUT_TRANSCODE_PASSTHROUGH output(%p) buff ptr(%p)",
                        buffer_size, qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH],
                        data_buffer_p);
                    ret = qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH]->stream.write(
                            (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_TRANSCODE_PASSTHROUGH],
                            data_buffer_p,
                            buffer_size);
                    if (ret > 0)
                       qap_mod->qap_output_bytes_written[QAP_OUT_TRANSCODE_PASSTHROUGH] += ret;
                }
            }
            else if ((device & AUDIO_DEVICE_OUT_AUX_DIGITAL)
                       && (p_qap->hdmi_connect)
                       && (p_qap->hdmi_sink_channels > 2)) {

                /* CASE 2: Multi-Channel PCM output to HDMI.
                 * If any other HDMI output is already enabled then this has to be dropped.
                 */

                if (p_qap->passthrough_enabled) {
                    //Closing all HDMI output stream from QAP.
                    close_all_hdmi_output_l();

                    //If passthrough is active then pcm hdmi output has to be dropped.
                    pthread_mutex_unlock(&p_qap->lock);
                    DEBUG_MSG("Compressed passthrough enabled, DROPPING DATA!");
                    return;
                }

                /* If Media format was changed for this stream then need to re-create the stream. */
                if (need_to_recreate_stream && qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]) {
                    DEBUG_MSG("closing MCH PCM session ox%x", (int)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]);
                    adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                             (struct audio_stream_out *)(qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]));
                    qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH] = NULL;
                    p_qap->mch_pcm_hdmi_enabled = false;
                }

                if (!p_qap->mch_pcm_hdmi_enabled && !(qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH])) {
                    audio_devices_t devices;

                    config.offload_info.format = config.format = AUDIO_FORMAT_PCM_16_BIT;
                    config.offload_info.channel_mask = config.channel_mask =
                                                               AUDIO_CHANNEL_OUT_5POINT1;

                    devices = AUDIO_DEVICE_OUT_AUX_DIGITAL;
                    flags = AUDIO_OUTPUT_FLAG_DIRECT;

                    DEBUG_MSG("Opening MCH PCM out(outputenum=%d) session ox%x with below params",
                        QAP_OUT_OFFLOAD_MCH,
                        (int)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]);

                    DEBUG_MSG("sample rate=%d bitwidth=%d format = 0x%x channel mask=0x%x flags=0x%x device =0x%x",
                        config.sample_rate,
                        config.offload_info.bit_width,
                        config.offload_info.format,
                        config.offload_info.channel_mask,
                        flags,
                        devices);

                    ret = adev_open_output_stream((struct audio_hw_device *)p_qap->adev,
                                                  QAP_DEFAULT_COMPR_AUDIO_HANDLE,
                                                  devices,
                                                  flags,
                                                  &config,
                                                  (struct audio_stream_out **)&(qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]),
                                                  NULL);
                    if (ret < 0) {
                        ERROR_MSG("Failed opening MCH PCM out(outputenum=%d) session ox%x",
                            QAP_OUT_OFFLOAD_MCH,
                            (int)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]);
                        pthread_mutex_unlock(&p_qap->lock);
                        return;
                        } else
                            DEBUG_MSG("Opened MCH PCM out(outputenum=%d) session ox%x",
                                QAP_OUT_OFFLOAD_MCH,
                                (int)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]);

                    set_out_stream_channel_map(qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH], new_conf);

                    qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->compr_config.fragments =
                            COMPRESS_OFFLOAD_NUM_FRAGMENTS;
                    qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->compr_config.fragment_size =
                            qap_get_pcm_offload_output_buffer_size(qap_mod, &config.offload_info);

                    p_qap->mch_pcm_hdmi_enabled = true;

                    if ((qap_mod->stream_in[QAP_IN_MAIN]
                        && qap_mod->stream_in[QAP_IN_MAIN]->client_callback != NULL) ||
                        (qap_mod->stream_in[QAP_IN_MAIN_2]
                        && qap_mod->stream_in[QAP_IN_MAIN_2]->client_callback != NULL)) {

                        if (qap_mod->stream_in[QAP_IN_MAIN]) {
                            qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->stream.set_callback(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH],
                                qap_mod->stream_in[QAP_IN_MAIN]->client_callback,
                                qap_mod->stream_in[QAP_IN_MAIN]->client_cookie);
                        }
                        if (qap_mod->stream_in[QAP_IN_MAIN_2]) {
                            qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->stream.set_callback(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH],
                                qap_mod->stream_in[QAP_IN_MAIN_2]->client_callback,
                                qap_mod->stream_in[QAP_IN_MAIN_2]->client_cookie);
                        }
                    } else if (qap_mod->stream_in[QAP_IN_PCM]
                               && qap_mod->stream_in[QAP_IN_PCM]->client_callback != NULL) {

                        qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->stream.set_callback(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH],
                                qap_mod->stream_in[QAP_IN_PCM]->client_callback,
                                qap_mod->stream_in[QAP_IN_PCM]->client_cookie);
                    }
                }
                if (qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]) {
                    DEBUG_MSG_VV("Writing Bytes(%d) to QAP_OUT_OFFLOAD_MCH output(%p) buff ptr(%p)",
                        buffer_size, qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH],
                        data_buffer_p);
                    ret = qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH]->stream.write(
                            (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD_MCH],
                            data_buffer_p,
                            buffer_size);
                    if (ret > 0)
                       qap_mod->qap_output_bytes_written[QAP_OUT_OFFLOAD_MCH] += ret;
                }
            }
            else {
                /* CASE 3: PCM output.
                 */

                if (!p_qap->hdmi_connect || p_qap->passthrough_enabled) {
                    close_all_hdmi_output_l();
                    close_qap_passthrough_stream_l();
                }

                /* If Media format was changed for this stream then need to re-create the stream. */
                if (need_to_recreate_stream && qap_mod->stream_out[QAP_OUT_OFFLOAD]) {
                    DEBUG_MSG("closing PCM session ox%x", (int)qap_mod->stream_out[QAP_OUT_OFFLOAD]);
                    adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                             (struct audio_stream_out *)(qap_mod->stream_out[QAP_OUT_OFFLOAD]));
                    qap_mod->stream_out[QAP_OUT_OFFLOAD] = NULL;
                }

                bt_stream = audio_extn_bt_hal_get_output_stream(qap_mod->bt_hdl);
                if (bt_stream != NULL) {
                    if (qap_mod->stream_out[QAP_OUT_OFFLOAD]) {
                        adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                                 (struct audio_stream_out *)(qap_mod->stream_out[QAP_OUT_OFFLOAD]));
                        qap_mod->stream_out[QAP_OUT_OFFLOAD] = NULL;
                    }

                    audio_extn_bt_hal_out_write(p_qap->bt_hdl, data_buffer_p, buffer_size);
                } else if (NULL == qap_mod->stream_out[QAP_OUT_OFFLOAD]) {
                    audio_devices_t devices;

                    if (qap_mod->stream_in[QAP_IN_MAIN])
                        devices = qap_mod->stream_in[QAP_IN_MAIN]->devices;
                    else
                        devices = qap_mod->stream_in[QAP_IN_PCM]->devices;

                    //If multi channel pcm or passthrough is already enabled then remove the hdmi flag from device.
                    if (p_qap->mch_pcm_hdmi_enabled || p_qap->passthrough_enabled) {
                        if (devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)
                            devices ^= AUDIO_DEVICE_OUT_AUX_DIGITAL;
                    }
                    if (devices == 0 || !p_qap->hdmi_connect) {
                        devices = device;
                    } else if (p_qap->hdmi_connect) {
                        devices = AUDIO_DEVICE_OUT_AUX_DIGITAL;
                    }

                    flags = AUDIO_OUTPUT_FLAG_DIRECT;


                    DEBUG_MSG("Opening Stereo PCM out(outputenum=%d) session ox%x with below params",
                        QAP_OUT_OFFLOAD,
                        (int)qap_mod->stream_out[QAP_OUT_OFFLOAD]);


                    DEBUG_MSG("sample rate=%d bitwidth=%d format = 0x%x channel mask=0x%x flags=0x%x device =0x%x",
                        config.sample_rate,
                        config.offload_info.bit_width,
                        config.offload_info.format,
                        config.offload_info.channel_mask,
                        flags,
                        devices);


                    /* TODO:: Need to Propagate errors to framework */
                    ret = adev_open_output_stream((struct audio_hw_device *)p_qap->adev,
                                                  QAP_DEFAULT_COMPR_AUDIO_HANDLE,
                                                  devices,
                                                  flags,
                                                  &config,
                                                  (struct audio_stream_out **)&(qap_mod->stream_out[QAP_OUT_OFFLOAD]),
                                                  NULL);
                    if (ret < 0) {
                        ERROR_MSG("Failed opening Stereo PCM out(outputenum=%d) session ox%x",
                            QAP_OUT_OFFLOAD,
                            (int)qap_mod->stream_out[QAP_OUT_OFFLOAD]);
                        pthread_mutex_unlock(&p_qap->lock);
                        return;
                    } else
                        DEBUG_MSG("Opened Stereo PCM out(outputenum=%d) session ox%x",
                            QAP_OUT_OFFLOAD,
                            (int)qap_mod->stream_out[QAP_OUT_OFFLOAD]);

                    set_out_stream_channel_map(qap_mod->stream_out[QAP_OUT_OFFLOAD], new_conf);

                    if ((qap_mod->stream_in[QAP_IN_MAIN]
                        && qap_mod->stream_in[QAP_IN_MAIN]->client_callback != NULL) ||
                        (qap_mod->stream_in[QAP_IN_MAIN_2]
                        && qap_mod->stream_in[QAP_IN_MAIN_2]->client_callback != NULL)) {

                        if (qap_mod->stream_in[QAP_IN_MAIN]) {
                            qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.set_callback(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD],
                                qap_mod->stream_in[QAP_IN_MAIN]->client_callback,
                                qap_mod->stream_in[QAP_IN_MAIN]->client_cookie);
                        }
                        if (qap_mod->stream_in[QAP_IN_MAIN_2]) {
                            qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.set_callback(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD],
                                qap_mod->stream_in[QAP_IN_MAIN_2]->client_callback,
                                qap_mod->stream_in[QAP_IN_MAIN_2]->client_cookie);
                        }
                    } else if (qap_mod->stream_in[QAP_IN_PCM]
                               && qap_mod->stream_in[QAP_IN_PCM]->client_callback != NULL) {

                        qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.set_callback(
                                                    (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD],
                                                    qap_mod->stream_in[QAP_IN_PCM]->client_callback,
                                                    qap_mod->stream_in[QAP_IN_PCM]->client_cookie);
                    }

                    qap_mod->stream_out[QAP_OUT_OFFLOAD]->compr_config.fragments =
                            COMPRESS_OFFLOAD_NUM_FRAGMENTS;
                    qap_mod->stream_out[QAP_OUT_OFFLOAD]->compr_config.fragment_size =
                            qap_get_pcm_offload_output_buffer_size(qap_mod, &config.offload_info);

                    if (qap_mod->is_vol_set) {
                        DEBUG_MSG("Setting Volume Left[%f], Right[%f]", qap_mod->vol_left, qap_mod->vol_right);
                        qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.set_volume(
                                (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD],
                                qap_mod->vol_left,
                                qap_mod->vol_right);
                    }
                }

                if (qap_mod->stream_out[QAP_OUT_OFFLOAD]) {
                    DEBUG_MSG_VV("Writing Bytes(%d) to QAP_OUT_OFFLOAD output(%p) buff ptr(%p)",
                        buffer_size, qap_mod->stream_out[QAP_OUT_OFFLOAD],
                        data_buffer_p);
                    ret = qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.write(
                            (struct audio_stream_out *)qap_mod->stream_out[QAP_OUT_OFFLOAD],
                            data_buffer_p,
                            buffer_size);
                    if (ret > 0)
                       qap_mod->qap_output_bytes_written[QAP_OUT_OFFLOAD] += ret;

                }
            }
            DEBUG_MSG_VV("Bytes consumed [%d] by Audio HAL", ret);
        }
    }
    else if (event_id == QAP_CALLBACK_EVENT_EOS
               || event_id == QAP_CALLBACK_EVENT_MAIN_2_EOS
               || event_id == QAP_CALLBACK_EVENT_EOS_ASSOC) {
        struct stream_out *out = qap_mod->stream_in[QAP_IN_MAIN];
        struct stream_out *out_pcm = qap_mod->stream_in[QAP_IN_PCM];
        struct stream_out *out_main2 = qap_mod->stream_in[QAP_IN_MAIN_2];
        struct stream_out *out_assoc = qap_mod->stream_in[QAP_IN_ASSOC];

        /**
         * TODO:: Only DD/DDP Associate Eos is handled, need to add support
         * for other formats.
         */
        if (event_id == QAP_CALLBACK_EVENT_EOS
                && (out_pcm != NULL)
                && (check_stream_state_l(out_pcm, STOPPING))) {

            lock_output_stream_l(out_pcm);
            out_pcm->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out_pcm->client_cookie);
            set_stream_state_l(out_pcm, STOPPED);
            unlock_output_stream_l(out_pcm);
            pthread_cond_signal(&qap_mod->drain_output_cond);
            DEBUG_MSG("sent pcm DRAIN_READY");
        } else if ( event_id == QAP_CALLBACK_EVENT_EOS_ASSOC
                && (out_assoc != NULL)
                && (check_stream_state_l(out_assoc, STOPPING))) {

            lock_output_stream_l(out_assoc);
            out_assoc->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out_assoc->client_cookie);
            set_stream_state_l(out_assoc, STOPPED);
            unlock_output_stream_l(out_assoc);
            pthread_cond_signal(&qap_mod->drain_output_cond);
            DEBUG_MSG("sent associated DRAIN_READY");
        } else if (event_id == QAP_CALLBACK_EVENT_MAIN_2_EOS
                && (out_main2 != NULL)
                && (check_stream_state_l(out_main2, STOPPING))) {

            lock_output_stream_l(out_main2);
            out_main2->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out_main2->client_cookie);
            set_stream_state_l(out_main2, STOPPED);
            unlock_output_stream_l(out_main2);
            pthread_cond_signal(&qap_mod->drain_output_cond);
            DEBUG_MSG("sent main2 DRAIN_READY");
        } else if ((out != NULL) && (check_stream_state_l(out, STOPPING))) {
            lock_output_stream_l(out);
            out->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out->client_cookie);
            set_stream_state_l(out, STOPPED);
            unlock_output_stream_l(out);
            pthread_cond_signal(&qap_mod->drain_output_cond);
            DEBUG_MSG("sent main DRAIN_READY");
        }
    }
    else if (event_id == QAP_CALLBACK_EVENT_EOS || event_id == QAP_CALLBACK_EVENT_EOS_ASSOC) {
        struct stream_out *out = NULL;

        if (event_id == QAP_CALLBACK_EVENT_EOS) {
            out = qap_mod->stream_in[QAP_IN_MAIN];
        } else {
            out = qap_mod->stream_in[QAP_IN_ASSOC];
        }

        if ((out != NULL) && (check_stream_state_l(out, STOPPING))) {
            lock_output_stream_l(out);
            out->client_callback(STREAM_CBK_EVENT_DRAIN_READY, NULL, out->client_cookie);
            set_stream_state_l(out, STOPPED);
            unlock_output_stream_l(out);
            DEBUG_MSG("sent DRAIN_READY");
        }
    } else if (QAP_CALLBACK_EVENT_DELAY == event_id) {
        qap_output_delay_t *qap_delay = (qap_output_delay_t*) buffer;
        main_delay_event_data.algo_delay = qap_delay->algo_delay;
        main_delay_event_data.buffering_delay = qap_delay->buffering_delay;
        main_delay_event_data.non_main_data_length = qap_delay->non_main_data_length;
        main_delay_event_data.non_main_data_offset = qap_delay->non_main_data_offset;
        if (delay_event_fired == false) {
            delay_event_fired = true;
            DEBUG_MSG("QAP_CALLBACK_EVENT_DELAY active");
            DEBUG_MSG("MS12 Latency/Delay event data alog delay %d, buffering delay %d non_main_data_lenght %d non_main_data_offset %d",
                main_delay_event_data.algo_delay,
                main_delay_event_data.buffering_delay,
                main_delay_event_data.non_main_data_length,
                main_delay_event_data.non_main_data_offset);
        }
    }

    pthread_mutex_unlock(&p_qap->lock);
    return;
}

static int qap_sess_close(struct qap_module* qap_mod)
{
    int j;
    int ret = -EINVAL;

    DEBUG_MSG("Closing Session.");

    //Check if all streams are closed or not.
    for (j = 0; j < MAX_QAP_MODULE_IN; j++) {
        if (qap_mod->stream_in[j] != NULL) {
            break;
        }
    }
    if (j != MAX_QAP_MODULE_IN) {
        DEBUG_MSG("Some stream is still active, Can not close session.");
        return 0;
    }

    qap_mod->is_session_closing = true;
    if(p_qap->qap_output_block_handling) {
        lock_session_output(qap_mod);
        if (!qap_mod->is_session_output_active && !p_qap->qap_active_api_count) {
            qap_mod->is_session_output_active = true;
            pthread_cond_signal(&qap_mod->session_output_cond);
        }
        unlock_session_output(qap_mod);
    }
    pthread_mutex_lock(&p_qap->lock);

    if (!qap_mod || !qap_mod->session_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p)",
            qap_mod, qap_mod->session_handle);
        return -EINVAL;
    }

    ret = qap_session_close(qap_mod->session_handle);
    if (QAP_STATUS_OK != ret) {
        ERROR_MSG("close session failed %d", ret);
        return -EINVAL;
    } else
        DEBUG_MSG("Closed QAP session 0x%x", (int)qap_mod->session_handle);

    qap_mod->session_handle = NULL;
    qap_mod->is_vol_set = false;
    memset(qap_mod->stream_state, 0, sizeof(qap_mod->stream_state));

    qap_close_all_output_streams(qap_mod);

    qap_mod->new_out_format_index = 0;

    pthread_mutex_unlock(&p_qap->lock);
    qap_mod->is_session_closing = false;
    DEBUG_MSG("Exit.");

    return 0;
}

static int qap_stream_close(struct stream_out *out)
{
    int ret = -EINVAL;
    struct qap_module *qap_mod = NULL;
    int index = -1;
    DEBUG_MSG("Flag [0x%x], Stream handle [%p]", out->flags, out->qap_stream_handle);

    qap_mod = get_qap_module_for_input_stream_l(out);
    index = get_input_stream_index_l(out);

    if (!qap_mod || !qap_mod->session_handle || (index < 0) || !out->qap_stream_handle) {
        ERROR_MSG("Wrong state to process qap_mod(%p) sess_hadl(%p) strm hndl(%p), index %d",
            qap_mod, qap_mod->session_handle, out->qap_stream_handle, index);
        return -EINVAL;
    }

    pthread_mutex_lock(&qap_mod->qap_stream_in_lock[index]);
    set_stream_state_l(out,STOPPED);

    check_and_activate_output_thread(true);
    ret = qap_module_deinit(out->qap_stream_handle);
    check_and_activate_output_thread(false);
    if (QAP_STATUS_OK != ret) {
        ERROR_MSG("deinit failed %d", ret);
        return -EINVAL;
    } else
        DEBUG_MSG("module(ox%x) closed successfully", (int)out->qap_stream_handle);
    qap_mod->stream_in[index] = NULL;

    out->qap_stream_handle = NULL;
    pthread_mutex_unlock(&qap_mod->qap_stream_in_lock[index]);

    //If all streams are closed then close the session.
    qap_sess_close(qap_mod);

    DEBUG_MSG("Exit");
    return ret;
}

#define MAX_INIT_PARAMS 6

static void update_qap_session_init_params(audio_session_handle_t session_handle) {
    DEBUG_MSG("Entry");
    qap_status_t ret = QAP_STATUS_OK;
    uint32_t cmd_data[MAX_INIT_PARAMS] = {0};

    /* all init params should be sent
     * together so gang them up.
     */
    cmd_data[0] = MS12_SESSION_CFG_MAX_CHS;
    cmd_data[1] = 6;/*5.1 channels*/

    cmd_data[2] = MS12_SESSION_CFG_BS_OUTPUT_MODE;
    cmd_data[3] = 3;/*DDP Re-encoding and DDP to DD Transcoding*/

    cmd_data[4] = MS12_SESSION_CFG_CHMOD_LOCKING;
    cmd_data[MAX_INIT_PARAMS - 1] = 1;/*Lock to 6 channel*/

    ret = qap_session_cmd(session_handle,
            QAP_SESSION_CMD_SET_PARAM,
            MAX_INIT_PARAMS * sizeof(uint32_t),
            &cmd_data[0],
            NULL,
            NULL);
    if (ret != QAP_STATUS_OK) {
        ERROR_MSG("session init params config failed");
    }
    DEBUG_MSG("Exit");
    return;
}

/* Query HDMI EDID and sets module output accordingly.*/
static int qap_set_hdmi_configuration_to_module()
{
    int ret = 0;
    int channels = 0;
    char prop_value[PROPERTY_VALUE_MAX] = {0};
    bool passth_support = false;
    qap_session_outputs_config_t session_outputs_config = {0};


    DEBUG_MSG("Entry");

    if (!p_qap) {
        return -EINVAL;
    }

    if (!p_qap->hdmi_connect) {
        return -EINVAL;
    }

    p_qap->hdmi_sink_channels = 0;

    if (!p_qap->qap_mod[MS12].session_handle && !p_qap->qap_mod[DTS_M8].session_handle) {
        DEBUG_MSG("HDMI connection comes even before session is setup");
        return -EINVAL;
    }

    session_outputs_config.num_output = 1;
    //QAP re-encoding and DSP offload passthrough is supported.
    if (property_get_bool("vendor.audio.offload.passthrough", false)
            && property_get_bool("vendor.audio.qap.reencode", false)) {

        if (p_qap->qap_mod[MS12].session_handle) {

            bool do_setparam = false;
            memcpy(prop_value, p_qap->ms12_out_format, sizeof(p_qap->ms12_out_format) -1);
            if (p_qap->ms12_out_format[0] == '\0') {
                property_get("vendor.audio.qap.hdmi.out", prop_value, NULL);
            }

            if (platform_is_edid_supported_format(p_qap->adev->platform, AUDIO_FORMAT_E_AC3)
                    && (strncmp(prop_value, "ddp", 3) == 0)) {
                do_setparam = true;
                session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_EAC3;
                session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_EAC3;
            } else if (platform_is_edid_supported_format(p_qap->adev->platform, AUDIO_FORMAT_AC3)) {
                do_setparam = true;
                session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_AC3;
                session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_AC3;
            }
            if (do_setparam) {
                DEBUG_MSG(" Enabling HDMI(Passthrough out) from MS12 wrapper outputid=0x%x",
                    session_outputs_config.output_config[0].id);

                ret = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                        &session_outputs_config);
                if (QAP_STATUS_OK != ret) {
                    ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_HDMI device with QAP %d", ret);
                    return ret;
                }
                passth_support = true;
            }

            if ((p_qap->ms12_lock & (1 << MS12_CHMOD_LOCK_MASK))) {
                if (!set_ms12_channel_mode_lock(((int)p_qap->ms12_lock)))
                    p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << MS12_CHMOD_LOCK_MASK)));
            }
        }

        if (p_qap->qap_mod[DTS_M8].session_handle) {

            bool do_setparam = false;
            if (platform_is_edid_supported_format(p_qap->adev->platform, AUDIO_FORMAT_DTS)) {
                do_setparam = true;
                session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_DTS;
                session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_DTS;
            }

            if (do_setparam) {
                ret = qap_session_cmd_l(p_qap->qap_mod[DTS_M8].session_handle,
                                        &session_outputs_config);
                if (QAP_STATUS_OK != ret) {
                    ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_HDMI device with QAP %d", ret);
                    return ret;
                }
                passth_support = true;
            }
        }
    }

    //Check for atmos lock request
    if ((p_qap->ms12_lock & (1 << MS12_ATMOS_LOCK_MASK))) {
        if (!set_ms12_atmos_lock((int)p_qap->ms12_lock))
            //on success clear atmos lock request bit.
            p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << MS12_ATMOS_LOCK_MASK)));
    }

    //Compressed passthrough is not enabled.
    if (!passth_support) {

        channels = platform_edid_get_max_channels(p_qap->adev->platform);
        session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_PCM_16_BIT;

        switch (channels) {
            case 8:
                DEBUG_MSG("Switching Qap output to 7.1 channels");
                session_outputs_config.output_config[0].channels = 8;
                if (!p_qap->qap_msmd_enabled)
                    session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_PCM_16_BIT;
                p_qap->hdmi_sink_channels = channels;
                break;
            case 6:
                DEBUG_MSG("Switching Qap output to 5.1 channels");
                session_outputs_config.output_config[0].channels = 6;
                if (!p_qap->qap_msmd_enabled)
                    session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_PCM_16_BIT;
                p_qap->hdmi_sink_channels = channels;
                break;
            default:
                DEBUG_MSG("Switching Qap output to default channels");
                session_outputs_config.output_config[0].channels = 2;
                if (!p_qap->qap_msmd_enabled)
                    session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_HDMI|QAP_AUDIO_FORMAT_PCM_16_BIT;
                p_qap->hdmi_sink_channels = 2;
                break;
        }

        if (p_qap->qap_mod[MS12].session_handle) {
            DEBUG_MSG(" Enabling HDMI(MCH PCM out) from MS12 wrapper outputid = %x", session_outputs_config.output_config[0].id);
            ret = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                    &session_outputs_config);
            if (QAP_STATUS_OK != ret) {
                ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_HDMI device with QAP %d", ret);
                return ret;
            }
        }
        if (p_qap->qap_mod[DTS_M8].session_handle) {
                ret = qap_session_cmd_l(p_qap->qap_mod[DTS_M8].session_handle,
                                        &session_outputs_config);
                if (QAP_STATUS_OK != ret) {
                    ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_HDMI device with QAP %d", ret);
                    return ret;
                }
            }

    }
    DEBUG_MSG("Exit");
    return ret;
}

static int set_ecref(const char *value) {

    int status = 0;
    long int channel = 0;
    qap_session_outputs_config_t session_outputs_config = {0};

    if (p_qap) {
       if (!p_qap->qap_mod[MS12].session_handle && !p_qap->qap_mod[DTS_M8].session_handle) {
           DEBUG_MSG("EC-ref request comes but session is not setup, caching request");
       }
    } else
       return -EINVAL;

    channel = strtol(value, NULL, 10);
    if (ec.ch == (int)channel) {
       DEBUG_MSG_VV("EC ref for channel %ld is already active", channel);
       return status;
    }

    if (channel > 0 && channel != 2 && channel != 6) {
       ERROR_MSG("Unsupported ec-ref channels %ld", channel);
       return -ENOTSUP;
    }

    pthread_mutex_lock(&p_qap->lock);
    ec.ch = (int)channel;
    if (ec.ch) {
        if (p_qap->qap_mod[MS12].session_handle) {
            if (p_qap->hdmi_connect) {
                status = qap_set_hdmi_configuration_to_module();
            } else {
                session_outputs_config.num_output = 1;
                session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_SPEAKER;
                session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_PCM_16_BIT;
                status = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                           &session_outputs_config);
            }
            if (QAP_STATUS_OK != status) {
                ERROR_MSG("Unable to register ECREF with QAP %d", status);
                ec.ch = 0;
            } else {
                DEBUG_MSG("EC reference is enabled.");
            }
        }
    } else {
        if(p_qap->qap_mod[MS12].session_handle) {
           if (p_qap->hdmi_connect) {
               status = qap_set_hdmi_configuration_to_module();
           } else {
               session_outputs_config.num_output = 1;
               session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_SPEAKER;
               session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_PCM_16_BIT;
               status = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                          &session_outputs_config);
           }
           if (ec.proxy_out_ptr) {
               insert_wav_header(ec.proxy_out_ptr, NULL);
               fclose(ec.proxy_out_ptr);
               DEBUG_MSG("File %s CLOSED, total bytes written %lld", ec.file_dump,
                              ec.total_bytes_written);
               ec.proxy_out_ptr = NULL;
               ec.total_bytes_written = 0;
           }
           DEBUG_MSG("EC reference is disabled.");
        }
    }
    pthread_mutex_unlock(&p_qap->lock);

    return status;
}

static void qap_set_default_configuration_to_module()
{
    qap_session_outputs_config_t session_outputs_config = {0};
    int ret = 0;

    DEBUG_MSG("Entry");

    if (!p_qap) {
        return;
    }

    if (!p_qap->bt_connect) {
        DEBUG_MSG("BT is not connected.");
    }

    //ms12 wrapper don't support bt, treat this as speaker and routign to bt
    //will take care as a part of data callback notifier


    session_outputs_config.num_output = 1;
    if(p_qap->bt_connect) {
        ALOGD("%s BT is connected, setting device to BT headset",__func__);
        session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_BLUETOOTH_A2DP;
    }else {
        session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_SPEAKER;
    }

    session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_PCM_16_BIT;

    if (p_qap->qap_mod[MS12].session_handle) {
        DEBUG_MSG(" Enabling speaker(PCM out) from MS12 wrapper outputid = %x", session_outputs_config.output_config[0].id);

        ret = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                            &session_outputs_config);
        if (QAP_STATUS_OK != ret) {
            ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_SPEAKER device with QAP %d", ret);
            return;
        }
    }
    if (p_qap->qap_mod[DTS_M8].session_handle) {
        ret = qap_session_cmd_l(p_qap->qap_mod[DTS_M8].session_handle,
                            &session_outputs_config);
        if (QAP_STATUS_OK != ret) {
            ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_SPEAKER device with QAP %d", ret);
            return;
        }
    }
}

static int set_ms12_output_format(const char *out_format) {

   int status = 0;

   if (out_format == NULL)
       return -EINVAL;

   if (p_qap) {
       if (!p_qap->qap_mod[MS12].session_handle && !p_qap->qap_mod[DTS_M8].session_handle) {
           ERROR_MSG("Set request recieved but session is not setup, caching request");
       }
   } else
       return -EINVAL;

   if (p_qap->ms12_out_format[0] != '\0' &&
      strncmp(p_qap->ms12_out_format, out_format, (sizeof(p_qap->ms12_out_format) - 1)) == 0) {
      DEBUG_MSG_VV("ms12 out format %s is already active", out_format);
      return status;
   }

   if (strncmp(out_format, "ddp", 3) == 0) {
        if (!(p_qap->hdmi_connect &&
              platform_is_edid_supported_format(p_qap->adev->platform, AUDIO_FORMAT_E_AC3))) {
              ERROR_MSG("Requested hdmi output format %s is not supported by sink", out_format);
              return -ENOTSUP;
        }
   } else if (strncmp(out_format, "dd", 2) == 0) {
        if (!(p_qap->hdmi_connect &&
             platform_is_edid_supported_format(p_qap->adev->platform, AUDIO_FORMAT_AC3))) {
             ERROR_MSG("Requested hdmi output format %s is not supported by sink", out_format);
             return -ENOTSUP;
        }
   } else if (strncmp(out_format, "pcm", 3) == 0) {
        property_set("vendor.audio.qap.reencode", "false");
   } else {
        ERROR_MSG("Not supported ms12 output format: %s", out_format);
        return -ENOTSUP;
   }

   memcpy(p_qap->ms12_out_format, out_format, sizeof(p_qap->ms12_out_format) - 1);

  if (p_qap->qap_mod[MS12].session_handle) {
      pthread_mutex_lock(&p_qap->lock);
      if (p_qap->hdmi_connect) {
          if (strncmp(p_qap->ms12_out_format, "pcm", 3) != 0)
              property_set("vendor.audio.qap.reencode", "true");

          status = qap_set_hdmi_configuration_to_module();
      } else
          qap_set_default_configuration_to_module();
      pthread_mutex_unlock(&p_qap->lock);
   }

   return status;
}

static int enable_qap_bypass(struct audio_device *adev, bool bypass_qap) {
    struct qap_module *qap_mod = NULL;
    struct stream_out *out = NULL;
    int i = 0;
    int status = 0;

    DEBUG_MSG_VV("enter");
    if (p_qap) {
        if (p_qap->bypass_enable == bypass_qap) {
            DEBUG_MSG_VV("Bypass %d request is already active", bypass_qap);
            return 0;
        }

        if (!p_qap->qap_mod[MS12].session_handle && !p_qap->qap_mod[DTS_M8].session_handle) {
            p_qap->bypass_enable = bypass_qap;
            ERROR_MSG("Set request recieved but session is not setup, caching request");
            return 0;
        }

        qap_mod = &p_qap->qap_mod[MS12];
        pthread_mutex_unlock(&adev->lock);
        lock_all_qap_stream_in(qap_mod);
        pthread_mutex_lock(&adev->lock);
        if (!bypass_qap) {
            /* Check if hal pcm active, move to standby */
            out = qap_mod->stream_in[QAP_IN_PCM];
            if (out != NULL && !out->standby) {
               pthread_mutex_unlock(&adev->lock);
               p_qap->hal_stream_ops.common.standby((struct audio_stream *)out);
               pthread_mutex_lock(&adev->lock);
            }
        } else {
            /* Check if any compress stream is active.
               If so, then reject the bypass request */
            for (i = QAP_IN_MAIN; i < MAX_QAP_MODULE_IN; i++) {
                 if (i == QAP_IN_PCM)
                    continue;

                 if (qap_mod->stream_in[i] != NULL &&
                     check_stream_state_l(qap_mod->stream_in[i], RUN)) {
                     ERROR_MSG("[%s] stream is still active in qap path, rejecting bypass request",
                               use_case_table[qap_mod->stream_in[i]->usecase]);
                     unlock_all_qap_stream_in(qap_mod);
                     return -ENOTSUP;
                 }
            }
            /* Check if pcm qap path is active, move to standby */
            if (qap_mod->stream_in[QAP_IN_PCM] != NULL &&
                check_stream_state_l(qap_mod->stream_in[QAP_IN_PCM], RUN)) {
                status = audio_extn_qap_stream_stop((struct stream_out *)qap_mod->stream_in[QAP_IN_PCM]);
                if (status == 0) {
                    set_stream_state_l((struct stream_out *)qap_mod->stream_in[QAP_IN_PCM], STOPPED);
                }
                qap_mod->stream_in[QAP_IN_PCM]->standby = true;
            }
            /* Set qap session output to inactive state */
            lock_session_output(qap_mod);
            qap_mod->is_session_output_active = false;
            unlock_session_output(qap_mod);
        }
        p_qap->bypass_enable = bypass_qap;
        unlock_all_qap_stream_in(qap_mod);
    } else
        return -EINVAL;

    DEBUG_MSG_VV("exit");
    return 0;
}

/* Open a MM module session with QAP. */
static int audio_extn_qap_session_open(mm_module_type mod_type, __unused struct stream_out *out)
{
    DEBUG_MSG("%s %d", __func__, __LINE__);
    int ret = 0;

    struct qap_module *qap_mod = NULL;

    if (mod_type >= MAX_MM_MODULE_TYPE)
        return -ENOTSUP; //Not supported by QAP module.

    pthread_mutex_lock(&p_qap->lock);

    qap_mod = &(p_qap->qap_mod[mod_type]);

    //If session is already opened then return.
    if (qap_mod->session_handle) {
        DEBUG_MSG("QAP Session is already opened.");
        pthread_mutex_unlock(&p_qap->lock);
        return 0;
    }

    if (MS12 == mod_type) {
        if (NULL == (qap_mod->session_handle = (void *)qap_session_open(QAP_SESSION_MS12_OTT, qap_mod->qap_lib))) {
            ERROR_MSG("Failed to open QAP session, lib_handle 0x%x", (int)qap_mod->qap_lib);
            ret = -EINVAL;
            goto exit;
        } else
            DEBUG_MSG("Opened QAP session 0x%x", (int)qap_mod->session_handle);

        update_qap_session_init_params(qap_mod->session_handle);
    }

    if (QAP_STATUS_OK != (qap_session_set_callback (qap_mod->session_handle, &qap_session_callback, (void *)qap_mod))) {
        ERROR_MSG("Failed to register QAP session callback");
        ret = -EINVAL;
        goto exit;
    }

    qap_mod->is_session_output_active = true;
    p_qap->qap_active_api_count = 0;

    if(p_qap->hdmi_connect)
        qap_set_hdmi_configuration_to_module();
    else
        qap_set_default_configuration_to_module();

exit:
    pthread_mutex_unlock(&p_qap->lock);
    return ret;
}



static int qap_map_input_format(audio_format_t audio_format, qap_audio_format_t *format)
{
    if (audio_format == AUDIO_FORMAT_AC3) {
        *format = QAP_AUDIO_FORMAT_AC3;
        DEBUG_MSG( "File Format is AC3!");
    } else if ((audio_format == AUDIO_FORMAT_E_AC3) ||
               (audio_format == AUDIO_FORMAT_E_AC3_JOC)) {
        *format = QAP_AUDIO_FORMAT_EAC3;
        DEBUG_MSG( "File Format is E_AC3!");
    } else if ((audio_format == AUDIO_FORMAT_AAC_ADTS_LC) ||
               (audio_format == AUDIO_FORMAT_AAC_ADTS_HE_V1) ||
               (audio_format == AUDIO_FORMAT_AAC_ADTS_HE_V2) ||
               (audio_format == AUDIO_FORMAT_AAC_LC) ||
               (audio_format == AUDIO_FORMAT_AAC_HE_V1) ||
               (audio_format == AUDIO_FORMAT_AAC_HE_V2) ||
               (audio_format == AUDIO_FORMAT_AAC_LATM_LC) ||
               (audio_format == AUDIO_FORMAT_AAC_LATM_HE_V1) ||
               (audio_format == AUDIO_FORMAT_AAC_LATM_HE_V2)) {
        *format = QAP_AUDIO_FORMAT_AAC_ADTS;
        DEBUG_MSG( "File Format is AAC!");
    } else if (audio_format == AUDIO_FORMAT_DTS) {
        *format = QAP_AUDIO_FORMAT_DTS;
        DEBUG_MSG( "File Format is DTS!");
    } else if (audio_format == AUDIO_FORMAT_DTS_HD) {
        *format = QAP_AUDIO_FORMAT_DTS_HD;
        DEBUG_MSG( "File Format is DTS_HD!");
    } else if (audio_format == AUDIO_FORMAT_PCM_16_BIT) {
        *format = QAP_AUDIO_FORMAT_PCM_16_BIT;
        DEBUG_MSG( "File Format is PCM_16!");
    } else if (audio_format == AUDIO_FORMAT_PCM_32_BIT) {
        *format = QAP_AUDIO_FORMAT_PCM_32_BIT;
        DEBUG_MSG( "File Format is PCM_32!");
    } else if (audio_format == AUDIO_FORMAT_PCM_24_BIT_PACKED) {
        *format = QAP_AUDIO_FORMAT_PCM_24_BIT_PACKED;
        DEBUG_MSG( "File Format is PCM_24!");
    } else if ((audio_format == AUDIO_FORMAT_PCM_8_BIT) ||
               (audio_format == AUDIO_FORMAT_PCM_8_24_BIT)) {
        *format = QAP_AUDIO_FORMAT_PCM_8_24_BIT;
        DEBUG_MSG( "File Format is PCM_8_24!");
    } else {
        ERROR_MSG( "File Format not supported!");
        return -EINVAL;
    }
    return 0;
}


void qap_module_callback(__unused qap_module_handle_t module_handle,
                         void* priv_data,
                         qap_module_callback_event_t event_id,
                         __unused int size,
                         __unused void *data)
{
    struct stream_out *out=(struct stream_out *)priv_data;

    DEBUG_MSG_VV("Entry");
    if (QAP_MODULE_CALLBACK_EVENT_SEND_INPUT_BUFFER == event_id) {
        DEBUG_MSG_VV("QAP_MODULE_CALLBACK_EVENT_SEND_INPUT_BUFFER for (%p)", out);
        if (out->client_callback) {
            out->client_callback(STREAM_CBK_EVENT_WRITE_READY, NULL, out->client_cookie);
        }
        else
            DEBUG_MSG("client has no callback registered, no action needed for this event %d",
                event_id);
    }
    else
        DEBUG_MSG("Un Recognized event %d", event_id);

    DEBUG_MSG_VV("exit");
    return;
}


/* opens a stream in QAP module. */
static int qap_stream_open(struct stream_out *out,
                           struct audio_config *config,
                           audio_output_flags_t flags,
                           audio_devices_t devices)
{
    int status = -EINVAL;
    mm_module_type mmtype = get_mm_module_for_format_l(config->format);
    struct qap_module* qap_mod = NULL;
    qap_module_config_t input_config = {0};

    DEBUG_MSG("Flags 0x%x, Device 0x%x for use case %s out 0x%x", flags, devices, use_case_table[out->usecase], (int)out);

    if (mmtype >= MAX_MM_MODULE_TYPE) {
        ERROR_MSG("Unsupported Stream");
        return -ENOTSUP;
    }

    //Open the module session, if not opened already.
    status = audio_extn_qap_session_open(mmtype, out);
    qap_mod = &(p_qap->qap_mod[mmtype]);

    if ((status != 0) || (!qap_mod->session_handle ))
        return status;

    input_config.sample_rate = config->sample_rate;
    input_config.channels = popcount(config->channel_mask);
    if (input_config.format != AUDIO_FORMAT_PCM_16_BIT) {
        input_config.format &= AUDIO_FORMAT_MAIN_MASK;
    }
    input_config.module_type = QAP_MODULE_DECODER;
    status = qap_map_input_format(config->format, &input_config.format);
    if (status == -EINVAL)
        return -EINVAL;

    DEBUG_MSG("qap_stream_open sample_rate(%d) channels(%d) devices(%#x) flags(%#x) format(%#x)",
              input_config.sample_rate, input_config.channels, devices, flags, input_config.format);

    check_and_activate_output_thread(true);

    if (input_config.format == QAP_AUDIO_FORMAT_PCM_16_BIT) {
        //If PCM stream is already opened then fail this stream open.
        if (qap_mod->stream_in[QAP_IN_PCM]) {
            ERROR_MSG("PCM input is already active.");
            status = -ENOTSUP;
            goto exit;
        }
        input_config.flags = QAP_MODULE_FLAG_SYSTEM_SOUND;
        status = qap_module_init(qap_mod->session_handle, &input_config, &out->qap_stream_handle);
        if (QAP_STATUS_OK != status) {
            ERROR_MSG("Unable to open PCM(QAP_MODULE_FLAG_SYSTEM_SOUND) QAP module %d", status);
            status = -EINVAL;
            goto exit;
        } else
            DEBUG_MSG("QAP_MODULE_FLAG_SYSTEM_SOUND, module(ox%x) opened successfully", (int)out->qap_stream_handle);

        qap_mod->stream_in[QAP_IN_PCM] = out;
    } else if (input_config.format == QAP_AUDIO_FORMAT_AC3 || input_config.format == QAP_AUDIO_FORMAT_EAC3) {
         /* Acquire all qap_stream_in locks here and in qap_stream_close api,
            so that both api are serialized and qap_stream_in array is mapped
            same as ms12 streams i.e QAP_IN_MAIN index should be mapped
            to MAIN1 primary stream id of ms12*/
         lock_all_qap_stream_in(qap_mod);
         input_config.flags = QAP_MODULE_FLAG_PRIMARY;
         status = qap_module_init(qap_mod->session_handle, &input_config, &out->qap_stream_handle);
         if (QAP_STATUS_OK != status) {
             ERROR_MSG("Unable to open QAP stream/module with QAP_MODULE_FLAG_PRIMARY flag %d", status);
             status = -EINVAL;
             goto exit;
             } else
                 DEBUG_MSG("QAP_MODULE_FLAG_PRIMARY, module opened successfully 0x%x", (int)out->qap_stream_handle);;

         if(qap_mod->stream_in[QAP_IN_MAIN]) {
             qap_mod->stream_in[QAP_IN_MAIN_2] = out;
         } else {
             qap_mod->stream_in[QAP_IN_MAIN] = out;
         }
         unlock_all_qap_stream_in(qap_mod);
    }
    if (out->qap_stream_handle) {
        status = qap_module_set_callback(out->qap_stream_handle, &qap_module_callback, out);
        if (QAP_STATUS_OK != status) {
            ERROR_MSG("Unable to register module callback %d", status);
            status = -EINVAL;
            goto exit;
        } else
            DEBUG_MSG("Module call back registered 0x%x cookie 0x%x", (int)out->qap_stream_handle, (int)out);
    }

    if (status != 0) {
        //If no stream is active then close the session.
        qap_sess_close(qap_mod);
        status = 0;
        goto exit;
    }

    //If Device is HDMI, QAP passthrough is enabled and there is no previous QAP passthrough input stream.
    if ((!p_qap->passthrough_in)
        && (devices & AUDIO_DEVICE_OUT_AUX_DIGITAL)
        && audio_extn_qap_passthrough_enabled(out)) {
        //Assign the QAP passthrough input stream.
        p_qap->passthrough_in = out;

        //If HDMI is connected and format is supported by HDMI then create QAP passthrough output stream.
        if (p_qap->hdmi_connect
            && platform_is_edid_supported_format(p_qap->adev->platform, out->format)) {
            status = create_qap_passthrough_stream_l();
            if (status < 0) {
                qap_stream_close(out);
                ERROR_MSG("QAP passthrough stream creation failed with error %d", status);
                goto exit;
            }
        }
        /*Else: since QAP passthrough input stream is already initialized,
         * when hdmi is connected
         * then qap passthrough output stream will be created.
         */
    }

exit:
    check_and_activate_output_thread(false);
    DEBUG_MSG();
    return status;
}

static int qap_out_resume(struct audio_stream_out* stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    int status = 0;
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);;
    DEBUG_MSG("Output Stream %p", out);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
       lock_output_stream_l(out);

       //If QAP passthrough is active then block the resume on module input streams.
       if (p_qap->passthrough_out) {
          //If resume is received for the QAP passthrough stream then call the primary HAL api.
          pthread_mutex_lock(&p_qap->lock);
          if (p_qap->passthrough_in == out) {
              status = p_qap->passthrough_out->stream.resume(
                       (struct audio_stream_out*)p_qap->passthrough_out);
              if (!status) out->offload_state = OFFLOAD_STATE_PLAYING;
          }
          pthread_mutex_unlock(&p_qap->lock);
       } else {
          qap_mod->pause = true;
          //Flush the module input stream.
          status = qap_stream_start_l(out);
       }

       unlock_output_stream_l(out);
    } else {
        status = p_qap->hal_stream_ops.resume(stream);
    }
    unlock_qap_stream_in(out);
    DEBUG_MSG();
    return status;
}

static int qap_out_set_parameters(struct audio_stream *stream, const char *kvpairs)
{
    struct str_parms *parms;
    char value[32];
    int val = 0;
    struct stream_out *out = (struct stream_out *)stream;
    int ret = 0;
    int err = 0;
    struct qap_module *qap_mod = NULL;

    DEBUG_MSG("usecase(%d: %s) kvpairs: %s", out->usecase, use_case_table[out->usecase], kvpairs);

    lock_qap_stream_in(out);
    if (!p_qap->bypass_enable) {
        parms = str_parms_create_str(kvpairs);
        err = str_parms_get_str(parms, AUDIO_PARAMETER_STREAM_ROUTING, value, sizeof(value));
        if (err < 0) {
           unlock_qap_stream_in(out);
           return err;
        }
        val = atoi(value);

        qap_mod = get_qap_module_for_input_stream_l(out);
        if (!qap_mod) {
            unlock_qap_stream_in(out);
            return (-EINVAL);
        }

        //TODO: HDMI is connected but user doesn't want HDMI output, close both HDMI outputs.

        /* Setting new device information to the mm module input streams.
         * This is needed if QAP module output streams are not created yet.
         */
        out->devices = val;

        check_and_activate_output_thread(true);
#ifndef SPLIT_A2DP_ENABLED
        if (val == AUDIO_DEVICE_OUT_BLUETOOTH_A2DP) {
           //If device is BT then open the BT stream if not already opened.
           if ( audio_extn_bt_hal_get_output_stream(qap_mod->bt_hdl) == NULL
              && audio_extn_bt_hal_get_device(qap_mod->bt_hdl) != NULL) {
              ret = audio_extn_bt_hal_open_output_stream(qap_mod->bt_hdl,
                                                        QAP_OUTPUT_SAMPLING_RATE,
                                                        AUDIO_CHANNEL_OUT_STEREO,
                                                        CODEC_BACKEND_DEFAULT_BIT_WIDTH);
              if (ret != 0) {
                  ERROR_MSG("BT Output stream open failure!");
              }
           }
        } else if (val != 0) {
           //If device is not BT then close the BT stream if already opened.
           if ( audio_extn_bt_hal_get_output_stream(qap_mod->bt_hdl) != NULL) {
              audio_extn_bt_hal_close_output_stream(qap_mod->bt_hdl);
           }
        }
        #endif

        if (p_qap->passthrough_in == out) { //Device routing is received for QAP passthrough stream.

           if (!(val & AUDIO_DEVICE_OUT_AUX_DIGITAL)) { //HDMI route is disabled.

              //If QAP pasthrough output is enabled. Close it.
              close_qap_passthrough_stream_l();

              //Send the routing information to mm module pcm output.
              if (qap_mod->stream_out[QAP_OUT_OFFLOAD]) {
                 ret = qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.common.set_parameters(
                        (struct audio_stream *)qap_mod->stream_out[QAP_OUT_OFFLOAD], kvpairs);
              }
              //else: device info is updated in the input streams.
           } else { //HDMI route is enabled.

              //create the QAf passthrough stream, if not created already.
              ret = create_qap_passthrough_stream_l();

              //If QAP passthrough out is enabled then send routing information.
              if (p_qap->passthrough_out != NULL) {
                 ret = p_qap->passthrough_out->stream.common.set_parameters(
                       (struct audio_stream *)p_qap->passthrough_out, kvpairs);
              }
           }
        } else {
           //Send the routing information to mm module pcm output.
           if (qap_mod->stream_out[QAP_OUT_OFFLOAD] && !p_qap->hdmi_connect) {
              ret = qap_mod->stream_out[QAP_OUT_OFFLOAD]->stream.common.set_parameters(
                   (struct audio_stream *)qap_mod->stream_out[QAP_OUT_OFFLOAD], kvpairs);
           }
        }
        str_parms_destroy(parms);
        check_and_activate_output_thread(false);
    } else {
        ret = p_qap->hal_stream_ops.common.set_parameters(stream, kvpairs);
    }
    unlock_qap_stream_in(out);
    return ret;
}

/* Checks if a stream is QAP stream or not. */
bool audio_extn_is_qap_stream(struct stream_out *out)
{
    struct qap_module *qap_mod = get_qap_module_for_input_stream_l(out);

    if (qap_mod) {
        return true;
    }
    return false;
}

int audio_extn_qap_open_output_stream(struct audio_hw_device *dev,
                                      audio_io_handle_t handle,
                                      audio_devices_t devices,
                                      audio_output_flags_t flags,
                                      struct audio_config *config,
                                      struct audio_stream_out **stream_out,
                                      const char *address)
{
    int ret = 0;
    struct stream_out *out;

    DEBUG_MSG("Entry");
    if (p_qap->bypass_enable &&
        (flags & (AUDIO_OUTPUT_FLAG_COMPRESS_OFFLOAD | AUDIO_OUTPUT_FLAG_DIRECT)) &&
        (!(flags & AUDIO_OUTPUT_SINK_CAPABILITY_DECTECTION))) {
        ERROR_MSG("Recieved compress path request when bypass mode is active, reject open request");
        return -EAGAIN;
    }

    ret = adev_open_output_stream(dev, handle, devices, flags, config, stream_out, address);
    if (*stream_out == NULL) {
        ERROR_MSG("Stream open failed %d", ret);
        return ret;
    }

#ifndef LINUX_ENABLED
//Bypass QAP for dummy PCM session opened by APM during boot time
    if(flags == 0) {
        ALOGD("bypassing QAP for flags is equal to none");
        return ret;
    }
#endif

    out = (struct stream_out *)*stream_out;

    DEBUG_MSG("%s 0x%x", use_case_table[out->usecase], (int)out);

    ret = qap_stream_open(out, config, flags, devices);
    if (ret < 0) {
        ERROR_MSG("Error opening QAP stream err[%d]", ret);
        //Stream not supported by QAP, Bypass QAP.
        return 0;
    }

    memcpy(&p_qap->hal_stream_ops, &out->stream, sizeof(struct audio_stream_out));
    /* Override function pointers based on qap definitions */
    out->stream.set_volume = qap_set_stream_volume;
    out->stream.pause = qap_out_pause;
    out->stream.resume = qap_out_resume;
    out->stream.drain = qap_out_drain;
    out->stream.flush = qap_out_flush;

    out->stream.common.standby = qap_out_standby;
    out->stream.common.set_parameters = qap_out_set_parameters;
    out->stream.get_latency = qap_out_get_latency;
    out->stream.get_render_position = qap_out_get_render_position;
    out->stream.write = qap_out_write;
    out->stream.get_presentation_position = qap_out_get_presentation_position;
    out->platform_latency = 0;

    /*TODO: Need to handle this for DTS*/
    if (out->usecase == USECASE_AUDIO_PLAYBACK_LOW_LATENCY) {
        out->usecase = USECASE_AUDIO_PLAYBACK_DEEP_BUFFER;
        out->config.period_size = QAP_DEEP_BUFFER_OUTPUT_PERIOD_SIZE;
        out->config.period_count = DEEP_BUFFER_OUTPUT_PERIOD_COUNT;
        out->config.start_threshold = QAP_DEEP_BUFFER_OUTPUT_PERIOD_SIZE / 4;
        out->config.avail_min = QAP_DEEP_BUFFER_OUTPUT_PERIOD_SIZE / 4;
    } else if(out->flags == AUDIO_OUTPUT_FLAG_DIRECT) {
        out->compr_config.fragment_size = qap_get_pcm_offload_input_buffer_size(&(config->offload_info));
    }

    *stream_out = &out->stream;

    DEBUG_MSG("Exit");
    return 0;
}

void audio_extn_qap_close_output_stream(struct audio_hw_device *dev,
                                        struct audio_stream_out *stream)
{
    struct stream_out *out = (struct stream_out *)stream;
    struct qap_module* qap_mod = get_qap_module_for_input_stream_l(out);

    DEBUG_MSG("%s 0x%x", use_case_table[out->usecase], (int)out);

    if (!qap_mod) {
        DEBUG_MSG("qap module is NULL, nothing to close");
        /*closing non-MS12/default output stream opened with qap */
        adev_close_output_stream(dev, stream);
        return;
    }

    DEBUG_MSG("stream_handle(%p) format = %x", out, out->format);

    //If close is received for QAP passthrough stream then close the QAP passthrough output.
    if (p_qap->passthrough_in == out) {
        if (p_qap->passthrough_out) {
            ALOGD("%s %d closing stream handle %p", __func__, __LINE__, p_qap->passthrough_out);
            pthread_mutex_lock(&p_qap->lock);
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                     (struct audio_stream_out *)(p_qap->passthrough_out));
            pthread_mutex_unlock(&p_qap->lock);
            p_qap->passthrough_out = NULL;
        }

        p_qap->passthrough_in = NULL;
    }

    qap_stream_close(out);

    adev_close_output_stream(dev, stream);

    DEBUG_MSG("Exit");
}

/* Check if QAP is supported or not. */
bool audio_extn_qap_is_enabled()
{
    bool prop_enabled = false;
    char value[PROPERTY_VALUE_MAX] = {0};
    property_get("vendor.audio.qap.enabled", value, NULL);
    prop_enabled = atoi(value) || !strncmp("true", value, 4);
    return (prop_enabled);
}

/* QAP set parameter function. For Device connect and disconnect. */
int audio_extn_qap_set_parameters(struct audio_device *adev, struct str_parms *parms)
{
    int status = 0, val = 0;
    char value[4] = {0};
    qap_session_outputs_config_t session_outputs_config = {0};

    if (!p_qap) {
        return -EINVAL;
    }

    DEBUG_MSG("Entry");

    status = str_parms_get_int(parms, "bypass_qap", &val);
    if (status >= 0) {
       status = enable_qap_bypass(adev, val ? true : false);
       DEBUG_MSG("Set param request to bypass qap %d is %s", val, status ? "failed" : "success");
       return status;
    }

    status = str_parms_get_str(parms, "ecref", value, sizeof(value));
    if (status >= 0) {
        status = set_ecref(value);
        DEBUG_MSG("Set ec ref to channel %ld is %s", strtol(value, NULL, 10), status ? "failed" : "success");
        return status;
    }
    status = str_parms_get_str(parms, "ms12_out_format", value, sizeof(value));
    if (status > 0) {
        status = set_ms12_output_format(value);
        DEBUG_MSG("Set ms12 output format to %s is %s", value, status ? "failed" : "success");
        return status;
    }
    status = str_parms_get_int(parms, AUDIO_PARAMETER_DEVICE_CONNECT, &val);

    if ((status >= 0) && audio_is_output_device(val)) {
        if (val & AUDIO_DEVICE_OUT_AUX_DIGITAL) { //HDMI is connected.
            DEBUG_MSG("AUDIO_DEVICE_OUT_AUX_DIGITAL connected");
            p_qap->hdmi_connect = 1;
            p_qap->hdmi_sink_channels = 0;

            if (p_qap->passthrough_in) { //If QAP passthrough is already initialized.
                lock_output_stream_l(p_qap->passthrough_in);
                if (platform_is_edid_supported_format(adev->platform,
                                                   p_qap->passthrough_in->format)) {
                    //If passthrough format is supported by HDMI then create the QAP passthrough output if not created already.
                    create_qap_passthrough_stream_l();
                   //Ignoring the returned error, If error then QAP passthrough is disabled.
                } else {
                   //If passthrough format is not supported by HDMI then close the QAP passthrough output if already created.
                   close_qap_passthrough_stream_l();
                }
                unlock_output_stream_l(p_qap->passthrough_in);
            }
            pthread_mutex_lock(&p_qap->lock);
            qap_set_hdmi_configuration_to_module();
            pthread_mutex_unlock(&p_qap->lock);
        } else if (val & AUDIO_DEVICE_OUT_BLUETOOTH_A2DP) {
              DEBUG_MSG("AUDIO_DEVICE_OUT_BLUETOOTH_A2DP connected");
              p_qap->bt_connect = 1;
              pthread_mutex_lock(&p_qap->lock);
              qap_set_default_configuration_to_module();
              pthread_mutex_unlock(&p_qap->lock);
#ifndef SPLIT_A2DP_ENABLED
              for (int k = 0; k < MAX_MM_MODULE_TYPE; k++) {
                   if (!p_qap->qap_mod[k].bt_hdl) {
                       DEBUG_MSG("Opening a2dp output...");
                       status = audio_extn_bt_hal_load(&p_qap->qap_mod[k].bt_hdl);
                       if (status != 0) {
                           ERROR_MSG("Error opening BT module");
                           return status;
                       }
                   }
              }
#endif
         }
        //TODO else if: Need to consider other devices.
    }

    status = str_parms_get_int(parms, AUDIO_PARAMETER_DEVICE_DISCONNECT, &val);
    if ((status >= 0) && audio_is_output_device(val)) {
        if (val & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
            DEBUG_MSG("AUDIO_DEVICE_OUT_AUX_DIGITAL disconnected");

            p_qap->hdmi_sink_channels = 0;

            p_qap->passthrough_enabled = 0;
            p_qap->mch_pcm_hdmi_enabled = 0;
            p_qap->hdmi_connect = 0;

            if (!p_qap->qap_mod[MS12].session_handle &&
                  !p_qap->qap_mod[DTS_M8].session_handle) {
                 DEBUG_MSG("HDMI disconnection comes even before session is setup");
                 return 0;
             }

             session_outputs_config.num_output = 1;

             session_outputs_config.output_config[0].id = AUDIO_DEVICE_OUT_SPEAKER;
             session_outputs_config.output_config[0].format = QAP_AUDIO_FORMAT_PCM_16_BIT;

             if (p_qap->qap_mod[MS12].session_handle) {
                 DEBUG_MSG("Enabling speaker(PCM out) from MS12 wrapper outputid = %x",
                             session_outputs_config.output_config[0].id);

                 pthread_mutex_lock(&p_qap->lock);
                 status = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                             &session_outputs_config);
                 pthread_mutex_unlock(&p_qap->lock);
                 if (QAP_STATUS_OK != status) {
                     ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_SPEAKER device with QAP %d",status);
                     return -EINVAL;
                 }
              }
              if (p_qap->qap_mod[DTS_M8].session_handle) {
                  pthread_mutex_lock(&p_qap->lock);
                  status = qap_session_cmd_l(p_qap->qap_mod[MS12].session_handle,
                                               &session_outputs_config);
                  pthread_mutex_unlock(&p_qap->lock);
                  if (QAP_STATUS_OK != status) {
                      ERROR_MSG("Unable to register AUDIO_DEVICE_OUT_SPEAKER device with QAP %d", status);
                      return -EINVAL;
                  }
              }
          } else if (val & AUDIO_DEVICE_OUT_BLUETOOTH_A2DP) {
               DEBUG_MSG("AUDIO_DEVICE_OUT_BLUETOOTH_A2DP disconnected");
               p_qap->bt_connect = 0;
                //reconfig HDMI as end device (if connected)
               if(p_qap->hdmi_connect) {
                  pthread_mutex_lock(&p_qap->lock);
                  qap_set_hdmi_configuration_to_module();
                  pthread_mutex_unlock(&p_qap->lock);
               }else {
                  qap_set_default_configuration_to_module();
               }
#ifndef SPLIT_A2DP_ENABLED
               DEBUG_MSG("Closing a2dp output...");
               for (int k = 0; k < MAX_MM_MODULE_TYPE; k++) {
                    if (p_qap->qap_mod[k].bt_hdl) {
                        audio_extn_bt_hal_unload(p_qap->qap_mod[k].bt_hdl);
                        p_qap->qap_mod[k].bt_hdl = NULL;
                    }
                }
#endif
           }
        //TODO else if: Need to consider other devices.
    }

    status = str_parms_get_int(parms, "ms12_atmos_lock", &val);
    if (status >= 0) {
        if (val)
            p_qap->ms12_lock = p_qap->ms12_lock | val;
        else
            p_qap->ms12_lock = (p_qap->ms12_lock & (~1));
        p_qap->ms12_lock = (p_qap->ms12_lock | (1 << MS12_ATMOS_LOCK_MASK));

        status = set_ms12_atmos_lock(val);
        if (!status) {
            //on success clear atmos lock request bit.
            p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << MS12_ATMOS_LOCK_MASK)));
        }
        DEBUG_MSG("Set ms12_atmos_lock to %d:%d is %s", val, p_qap->ms12_lock, status ? "failed" : "success");
        return status;
    }

    status = str_parms_get_int(parms, "ms12_chmod_lock", &val);
    if (status >= 0) {
        if (val)
            p_qap->ms12_lock = (p_qap->ms12_lock | (~0 & (val << 1))); //set
        else
            p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << 1))); //reset
        p_qap->ms12_lock = (p_qap->ms12_lock | (1 << MS12_CHMOD_LOCK_MASK));

        status = set_ms12_channel_mode_lock(val);
        if (!status)
            p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << MS12_CHMOD_LOCK_MASK)));
        DEBUG_MSG("Set ms12 channel_mode_lock to %d:%d is %s", val, p_qap->ms12_lock, status ? "failed" : "success");
        return status;
    }

    status = str_parms_get_int(parms, "ms12_volume_interpolation", &val);
    if (status >= 0) {
        pthread_mutex_lock(&p_qap->lock);
        p_qap->qap_mod[MS12].interpolation = val;
        pthread_mutex_unlock(&p_qap->lock);
        DEBUG_MSG("Set ms12 volume interpolation %d is %s", val, status ? "failed" : "success");
        return status;
    }

    status = str_parms_get_int(parms, "ms12_continuous_output", &val);
    if (status >= 0) {
        pthread_mutex_lock(&p_qap->lock);
        p_qap->qap_output_block_handling = val? 0 : 1;
        pthread_mutex_unlock(&p_qap->lock);
        DEBUG_MSG("Set ms12 continuous_output to %d is %s", val, status ? "failed" : "success");
        return status;
    }

    DEBUG_MSG("Exit");
    return status;
}

/* Create the QAP. */
int audio_extn_qap_init(struct audio_device *adev)
{
    char value[PROPERTY_VALUE_MAX] = {0};
    DEBUG_MSG("Entry");

    p_qap = calloc(1, sizeof(struct qap));
    if (p_qap == NULL) {
        ERROR_MSG("Out of memory");
        return -ENOMEM;
    }

    p_qap->adev = adev;
    p_qap->bypass_enable = false;

    if (property_get_bool("vendor.audio.qap.msmd", false)) {
        DEBUG_MSG("MSMD enabled.");
        p_qap->qap_msmd_enabled = 1;
    }

    if (property_get_bool("vendor.audio.qap.output.block.handling", false)) {
        DEBUG_MSG("out put thread blocking handling enabled.");
        p_qap->qap_output_block_handling = 1;
    }

    if (property_get("persist.vendor.audio.qap.ecref", value, NULL)) {
        set_ecref(value);
    }

    pthread_mutex_init(&p_qap->lock, (const pthread_mutexattr_t *) NULL);

    int i = 0;

    for (i = 0; i < MAX_MM_MODULE_TYPE; i++) {
        char lib_name[PROPERTY_VALUE_MAX] = {0};
        struct qap_module *qap_mod = &(p_qap->qap_mod[i]);

        if (i == MS12) {
            property_get("vendor.audio.qap.library", value, NULL);
            if (value[0] != 0) {
                snprintf(lib_name, PROPERTY_VALUE_MAX, "%s", value);

                DEBUG_MSG("Opening Ms12 library at %s", lib_name);
                qap_mod->qap_lib = ( void *) qap_load_library(lib_name);
                if (qap_mod->qap_lib == NULL) {
                    ERROR_MSG("qap load lib failed for MS12 %s", lib_name);
                    continue;
                }
                DEBUG_MSG("Loaded QAP lib at %s", lib_name);
                pthread_mutex_init(&qap_mod->session_output_lock, (const pthread_mutexattr_t *) NULL);
                pthread_cond_init(&qap_mod->session_output_cond, (const pthread_condattr_t *)NULL);
                pthread_cond_init(&qap_mod->drain_output_cond, (const pthread_condattr_t *)NULL);
            }
            //bit 0:atmos lock will be disabled by default
            p_qap->ms12_lock = p_qap->ms12_lock & (~1);
            //bit 2:reset atmos lock request bit
            p_qap->ms12_lock = (p_qap->ms12_lock & (~(1 << MS12_ATMOS_LOCK_MASK)));
            //default chmod_lock is enabled
            p_qap->ms12_lock = p_qap->ms12_lock | (1 << 1);
            p_qap->qap_output_block_handling = 1;

            qap_mod->interpolation = 0; /*apply gain linearly*/
            qap_mod->pause = false;
            p_qap->wait_on_cond = 0;
        } else if (i == DTS_M8) {
            property_get("vendor.audio.qap.m8.library", value, NULL);
            if (value[0] != 0) {
                snprintf(lib_name, PROPERTY_VALUE_MAX, "%s", value);
                qap_mod->qap_lib = dlopen(lib_name, RTLD_NOW);
                if (qap_mod->qap_lib == NULL) {
                    ERROR_MSG("DLOPEN failed for DTS M8 %s", lib_name);
                    continue;
                }
                DEBUG_MSG("DLOPEN successful for %s", lib_name);
                pthread_mutex_init(&qap_mod->session_output_lock, (const pthread_mutexattr_t *) NULL);
                pthread_cond_init(&qap_mod->session_output_cond, (const pthread_condattr_t *)NULL);
            }
        } else {
            continue;
        }
        for (i = QAP_IN_MAIN; i < MAX_QAP_MODULE_IN; i++) {
            pthread_mutex_init(&qap_mod->qap_stream_in_lock[i], (const pthread_mutexattr_t *) NULL);
        }
    }

    no_of_outputs = 0;

    for(i = 0; i<MAX_OUTPUTS;i++) {
        cb_data_array[i].buff_ptr = malloc(MAX_OUT_BUFFER);
        cb_data_array[i].buff_size = 0;
        cb_data_array[i].dev_id= 0;
        if (cb_data_array[i].buff_ptr != NULL)
            DEBUG_MSG("buffers allocated successfully %d %p %d", i, cb_data_array[i].buff_ptr, MAX_OUT_BUFFER);
    }

    DEBUG_MSG("Exit");
    return 0;
}

/* Tear down the qap extension. */
void audio_extn_qap_deinit()
{
    int i;
    DEBUG_MSG("Entry");
    char value[PROPERTY_VALUE_MAX] = {0};
    char lib_name[PROPERTY_VALUE_MAX] = {0};

    if (p_qap != NULL) {
        for (i = 0; i < MAX_MM_MODULE_TYPE; i++) {
            if (p_qap->qap_mod[i].session_handle != NULL)
                qap_sess_close(&p_qap->qap_mod[i]);

            if (p_qap->qap_mod[i].qap_lib != NULL) {
                if (i == MS12) {
                    property_get("vendor.audio.qap.library", value, NULL);
                    snprintf(lib_name, PROPERTY_VALUE_MAX, "%s", value);
                    DEBUG_MSG("lib_name %s", lib_name);
                    if (QAP_STATUS_OK != qap_unload_library(p_qap->qap_mod[i].qap_lib))
                        ERROR_MSG("Failed to unload MS12 library lib name %s", lib_name);
                    else
                        DEBUG_MSG("closed/unloaded QAP lib at %s", lib_name);
                    p_qap->qap_mod[i].qap_lib = NULL;
                } else {
                    dlclose(p_qap->qap_mod[i].qap_lib);
                    p_qap->qap_mod[i].qap_lib = NULL;
                }
                pthread_mutex_destroy(&p_qap->qap_mod[i].session_output_lock);
                pthread_cond_destroy(&p_qap->qap_mod[i].session_output_cond);
                pthread_cond_destroy(&p_qap->qap_mod[i].drain_output_cond);

                for (i = QAP_IN_MAIN; i < MAX_QAP_MODULE_IN; i++) {
                     pthread_mutex_destroy(&p_qap->qap_mod[i].qap_stream_in_lock[i]);
                }
            }
        }

        if (p_qap->passthrough_out) {
            adev_close_output_stream((struct audio_hw_device *)p_qap->adev,
                                     (struct audio_stream_out *)(p_qap->passthrough_out));
            p_qap->passthrough_out = NULL;
        }

        pthread_mutex_destroy(&p_qap->lock);
        free(p_qap);
        p_qap = NULL;
    }
    for(i = 0; i<MAX_OUTPUTS;i++) {
        free(cb_data_array[i].buff_ptr);
         DEBUG_MSG("buffers de-allocated successfully %d", i);
    }

    no_of_outputs = 0;
    if (property_get("persist.vendor.audio.qap.ecref", value, NULL)) {
        set_ecref(value);
    }

    DEBUG_MSG("Exit");
}
