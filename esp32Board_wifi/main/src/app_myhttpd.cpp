// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "app_myhttpd.hpp"

#include <list>
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_jpeg_enc.h"
#include "lwip/sockets.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "app_mymdns.h"
#include "sdkconfig.h"

#include "yahboom_camera.h"
#include "yahboom_performance.h"
#include "esp_timer.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "camera_httpd";
#endif

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static bool gReturnFB = true;

static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
#define STREAM_JPEG_QUALITY 60
#define STREAM_JPEG_CONGESTED_QUALITY 40
#define STREAM_JPEG_CONGESTION_US 100000
#define STREAM_JPEG_QUALITY_RECOVERY_FRAMES 12
#define STREAM_JPEG_BUFFER_SIZE (128 * 1024)
#define STREAM_PREVIEW_WIDTH 160
#define STREAM_PREVIEW_HEIGHT 120
#define STREAM_PREVIEW_BUFFER_SIZE (STREAM_PREVIEW_WIDTH * STREAM_PREVIEW_HEIGHT * 2)
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %lld.%06lld\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
httpd_handle_t webserver = NULL;

typedef struct
{
    jpeg_enc_handle_t handle;
    uint8_t *output;
    uint8_t *preview;
    uint8_t quality;
    uint8_t stable_frames;
} stream_jpeg_encoder_t;

static bool stream_jpeg_encoder_init(stream_jpeg_encoder_t *encoder)
{
    jpeg_enc_config_t config = DEFAULT_JPEG_ENC_CONFIG();
    config.width = STREAM_PREVIEW_WIDTH;
    config.height = STREAM_PREVIEW_HEIGHT;
    config.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    config.subsampling = JPEG_SUBSAMPLE_420;
    config.quality = STREAM_JPEG_QUALITY;
    config.rotate = JPEG_ROTATE_0D;
    config.task_enable = false;

    encoder->handle = NULL;
    encoder->quality = STREAM_JPEG_QUALITY;
    encoder->stable_frames = 0;
    encoder->preview = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, STREAM_PREVIEW_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    encoder->output = static_cast<uint8_t *>(
        heap_caps_malloc(STREAM_JPEG_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!encoder->preview || !encoder->output)
    {
        ESP_LOGE(TAG, "Fast JPEG preview buffer allocation failed");
        heap_caps_free(encoder->preview);
        heap_caps_free(encoder->output);
        encoder->preview = NULL;
        encoder->output = NULL;
        return false;
    }

    jpeg_error_t ret = jpeg_enc_open(&config, &encoder->handle);
    if (ret != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "Fast JPEG encoder initialization failed: %d", ret);
        heap_caps_free(encoder->preview);
        heap_caps_free(encoder->output);
        encoder->preview = NULL;
        encoder->output = NULL;
        return false;
    }

    ESP_LOGI(TAG, "Fast JPEG encoder ready: %dx%d color preview, quality %d",
             STREAM_PREVIEW_WIDTH, STREAM_PREVIEW_HEIGHT, STREAM_JPEG_QUALITY);
    return true;
}

static void stream_jpeg_encoder_deinit(stream_jpeg_encoder_t *encoder)
{
    if (encoder->handle)
    {
        jpeg_enc_close(encoder->handle);
        encoder->handle = NULL;
    }
    heap_caps_free(encoder->preview);
    heap_caps_free(encoder->output);
    encoder->preview = NULL;
    encoder->output = NULL;
}

// 本地识别保留 QVGA；网页预览保留亮度边缘，色度仍按 2x2 平均。
static bool stream_downsample_yuv422(const camera_fb_t *frame, uint8_t *preview)
{
    if (!frame || !preview || frame->format != PIXFORMAT_YUV422 ||
        frame->width != 320 || frame->height != 240 ||
        frame->len < (size_t)frame->width * frame->height * 2)
    {
        return false;
    }

    const size_t source_stride = frame->width * 2;
    const size_t preview_stride = STREAM_PREVIEW_WIDTH * 2;
    for (size_t y = 0; y < STREAM_PREVIEW_HEIGHT; ++y)
    {
        const uint8_t *top = frame->buf + (y * 2) * source_stride;
        const uint8_t *bottom = top + source_stride;
        uint8_t *destination = preview + y * preview_stride;

        for (size_t pair = 0; pair < STREAM_PREVIEW_WIDTH / 2; ++pair)
        {
            const uint8_t *top_pair = top + pair * 8;
            const uint8_t *bottom_pair = bottom + pair * 8;
            uint8_t *output_pair = destination + pair * 4;

            output_pair[0] = top_pair[0];
            output_pair[1] = (uint8_t)((top_pair[1] + top_pair[5] +
                                        bottom_pair[1] + bottom_pair[5]) / 4);
            output_pair[2] = top_pair[4];
            output_pair[3] = (uint8_t)((top_pair[3] + top_pair[7] +
                                        bottom_pair[3] + bottom_pair[7]) / 4);
        }
    }

    return true;
}

static bool stream_jpeg_encode(stream_jpeg_encoder_t *encoder, const camera_fb_t *frame,
                               uint8_t **jpg_buf, size_t *jpg_len)
{
    if (!encoder->handle || !encoder->output || !encoder->preview ||
        !stream_downsample_yuv422(frame, encoder->preview))
    {
        return false;
    }

    int encoded_size = 0;
    jpeg_error_t ret = jpeg_enc_process(encoder->handle, encoder->preview,
                                        STREAM_PREVIEW_BUFFER_SIZE,
                                        encoder->output, STREAM_JPEG_BUFFER_SIZE,
                                        &encoded_size);
    if (ret != JPEG_ERR_OK || encoded_size <= 0)
    {
        ESP_LOGE(TAG, "Fast JPEG compression failed: %d", ret);
        return false;
    }

    *jpg_buf = encoder->output;
    *jpg_len = static_cast<size_t>(encoded_size);
    return true;
}

static void stream_jpeg_adjust_quality(stream_jpeg_encoder_t *encoder, uint32_t send_us)
{
    uint8_t next_quality = encoder->quality;
    if (send_us > STREAM_JPEG_CONGESTION_US)
    {
        encoder->stable_frames = 0;
        if (next_quality > STREAM_JPEG_CONGESTED_QUALITY)
            next_quality = STREAM_JPEG_CONGESTED_QUALITY;
    }
    else if (next_quality < STREAM_JPEG_QUALITY)
    {
        encoder->stable_frames++;
        if (encoder->stable_frames >= STREAM_JPEG_QUALITY_RECOVERY_FRAMES)
        {
            next_quality += 5;
            if (next_quality > STREAM_JPEG_QUALITY)
                next_quality = STREAM_JPEG_QUALITY;
            encoder->stable_frames = 0;
        }
    }

    if (next_quality == encoder->quality)
        return;

    if (jpeg_enc_set_quality(encoder->handle, next_quality) == JPEG_ERR_OK)
    {
        encoder->quality = next_quality;
        ESP_LOGI(TAG, "Preview JPEG quality adjusted to %u", (unsigned)next_quality);
    }
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *frame = NULL;
    esp_err_t res = ESP_OK;

    if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
    {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

        char ts[32];
        snprintf(ts, 32, "%lld.%06ld", frame->timestamp.tv_sec, frame->timestamp.tv_usec);
        httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

        // size_t fb_len = 0;
        if (frame->format == PIXFORMAT_JPEG)
        {
                // fb_len = frame->len;
            res = httpd_resp_send(req, (const char *)frame->buf, frame->len);
        }
        else
        {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(frame, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            // fb_len = jchunk.len;
        }

        if (xQueueFrameO)
        {
            xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
        }
        else if (gReturnFB)
        {
            esp_camera_fb_return(frame);
        }
        else
        {
            free(frame);
        }
    }
    else
    {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *frame = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char part_buf[128];
    stream_jpeg_encoder_t encoder = {};
    bool fast_encoder_ready = stream_jpeg_encoder_init(&encoder);
    const int tcp_nodelay = 1;

    if (setsockopt(httpd_req_to_sockfd(req), IPPROTO_TCP, TCP_NODELAY,
                   &tcp_nodelay, sizeof(tcp_nodelay)) < 0)
    {
        ESP_LOGW(TAG, "Failed to enable TCP_NODELAY for stream");
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        stream_jpeg_encoder_deinit(&encoder);
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true)
    {
        bool jpg_buf_allocated = false;
        bool fallback_encoder_used = false;
        const int64_t queue_wait_start_us = esp_timer_get_time();

        if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
        {
            const uint32_t queue_wait_us =
                static_cast<uint32_t>(esp_timer_get_time() - queue_wait_start_us);
            _timestamp.tv_sec = frame->timestamp.tv_sec;
            _timestamp.tv_usec = frame->timestamp.tv_usec;
            const int64_t encode_start_us = esp_timer_get_time();

            if (frame->format == PIXFORMAT_JPEG)
            {
                _jpg_buf = frame->buf;
                _jpg_buf_len = frame->len;
            }
            else if (fast_encoder_ready && stream_jpeg_encode(&encoder, frame, &_jpg_buf, &_jpg_buf_len))
            {
            }
            else if (!frame2jpg(frame, STREAM_JPEG_QUALITY, &_jpg_buf, &_jpg_buf_len))
            {
                fallback_encoder_used = true;
                ESP_LOGE(TAG, "JPEG compression failed");
                res = ESP_FAIL;
            }
            else
            {
                fallback_encoder_used = true;
                jpg_buf_allocated = true;
            }
            const uint32_t encode_us =
                static_cast<uint32_t>(esp_timer_get_time() - encode_start_us);
            const size_t encoded_jpeg_size = _jpg_buf_len;

            // YUV/RGB 帧已编码到独立 JPEG 缓冲，可在网络发送前归还摄像头帧。
            if (res == ESP_OK && xQueueFrameO == NULL && gReturnFB && frame->format != PIXFORMAT_JPEG)
            {
                esp_camera_fb_return(frame);
                frame = NULL;
            }
            const int64_t send_start_us = esp_timer_get_time();
            if (res == ESP_OK)
                res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

            if (res == ESP_OK)
            {
                size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len,
                                       (long long)_timestamp.tv_sec, (long long)_timestamp.tv_usec);
                res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            }

            if (res == ESP_OK)
                res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);

            const uint32_t send_us =
                static_cast<uint32_t>(esp_timer_get_time() - send_start_us);
            yahboom_performance_record_stream(queue_wait_us, encode_us, send_us,
                                               encoded_jpeg_size, fallback_encoder_used);
            if (fast_encoder_ready)
                stream_jpeg_adjust_quality(&encoder, send_us);
        }
        else
        {
            res = ESP_FAIL;
        }

        if (jpg_buf_allocated)
        {
            free(_jpg_buf);
        }
        _jpg_buf = NULL;
        _jpg_buf_len = 0;

        if (frame)
        {
            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
            }
            else if (gReturnFB)
            {
                esp_camera_fb_return(frame);
            }
            else
            {
                free(frame);
            }
        }

        if (res != ESP_OK)
        {
            break;
        }
    }

    stream_jpeg_encoder_deinit(&encoder);

    return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    char *buf = NULL;
    size_t buf_len = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = (char *)malloc(buf_len);
        if (!buf)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char variable[32];
    char value[32];

    if (parse_get(req, &buf) != ESP_OK ||
        httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int val = atoi(value);
    ESP_LOGI(TAG, "%s = %d", variable, val);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize"))
    {
        if (s->pixformat == PIXFORMAT_JPEG)
        {
            res = s->set_framesize(s, (framesize_t)val);
            if (res == 0)
            {
                app_mymdns_update_framesize(val);
            }
        }
    }
    else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    else if (!strcmp(variable, "led_intensity"))
        led_duty = val;
#endif

    else if (!strcmp(variable, "face_detect"))
    {
        detection_enabled = val;
        if (!detection_enabled)
        {
            recognition_enabled = 0;
        }
    }
    else if (!strcmp(variable, "face_enroll"))
        is_enrolling = val;
    else if (!strcmp(variable, "face_recognize"))
    {
        recognition_enabled = val;
        if (recognition_enabled)
        {
            detection_enabled = val;
        }
    }
    else
    {
        res = -1;
    }

    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask)
{
    return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    sensor_t *s = esp_camera_sensor_get();
    char *p = json_response;
    *p++ = '{';

    if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID)
    {
        for (int reg = 0x3400; reg < 0x3406; reg += 2)
        {
            p += print_reg(p, s, reg, 0xFFF); //12 bit
        }
        p += print_reg(p, s, 0x3406, 0xFF);

        p += print_reg(p, s, 0x3500, 0xFFFF0); //16 bit
        p += print_reg(p, s, 0x3503, 0xFF);
        p += print_reg(p, s, 0x350a, 0x3FF);  //10 bit
        p += print_reg(p, s, 0x350c, 0xFFFF); //16 bit

        for (int reg = 0x5480; reg <= 0x5490; reg++)
        {
            p += print_reg(p, s, reg, 0xFF);
        }

        for (int reg = 0x5380; reg <= 0x538b; reg++)
        {
            p += print_reg(p, s, reg, 0xFF);
        }

        for (int reg = 0x5580; reg < 0x558a; reg++)
        {
            p += print_reg(p, s, reg, 0xFF);
        }
        p += print_reg(p, s, 0x558a, 0x1FF); //9 bit
    }
    else
    {
        p += print_reg(p, s, 0xd3, 0xFF);
        p += print_reg(p, s, 0x111, 0xFF);
        p += print_reg(p, s, 0x132, 0xFF);
    }

    p += sprintf(p, "\"board\":\"%s\",", CAMERA_MODULE_NAME);
    p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
    p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
    p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
    p += sprintf(p, ",\"face_detect\":%u", detection_enabled);
    p += sprintf(p, ",\"face_enroll\":%u,", is_enrolling);
    p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t mdns_handler(httpd_req_t *req)
{
    size_t json_len = 0;
    const char *json_response = app_mymdns_query(&json_len);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, json_len);
}

static esp_err_t xclk_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _xclk[32];

    if (parse_get(req, &buf) != ESP_OK ||
        httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int xclk = atoi(_xclk);
    ESP_LOGI(TAG, "Set XCLK: %d MHz", xclk);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _reg[32];
    char _mask[32];
    char _val[32];

    if (parse_get(req, &buf) != ESP_OK ||
        httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK ||
        httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK ||
        httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int reg = atoi(_reg);
    int mask = atoi(_mask);
    int val = atoi(_val);
    ESP_LOGI(TAG, "Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_reg(s, reg, mask, val);
    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _reg[32];
    char _mask[32];

    if (parse_get(req, &buf) != ESP_OK ||
        httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK ||
        httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int reg = atoi(_reg);
    int mask = atoi(_mask);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->get_reg(s, reg, mask);
    if (res < 0)
    {
        return httpd_resp_send_500(req);
    }
    ESP_LOGI(TAG, "Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

    char buffer[20];
    const char *val = itoa(res, buffer, 10);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def)
{
    char _int[16];
    if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK)
    {
        return def;
    }
    return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req)
{
    char *buf = NULL;

    if (parse_get(req, &buf) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int bypass = parse_get_var(buf, "bypass", 0);
    int mul = parse_get_var(buf, "mul", 0);
    int sys = parse_get_var(buf, "sys", 0);
    int root = parse_get_var(buf, "root", 0);
    int pre = parse_get_var(buf, "pre", 0);
    int seld5 = parse_get_var(buf, "seld5", 0);
    int pclken = parse_get_var(buf, "pclken", 0);
    int pclk = parse_get_var(buf, "pclk", 0);
    free(buf);

    ESP_LOGI(TAG, "Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req)
{
    char *buf = NULL;

    if (parse_get(req, &buf) != ESP_OK)
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int startX = parse_get_var(buf, "sx", 0);
    int startY = parse_get_var(buf, "sy", 0);
    int endX = parse_get_var(buf, "ex", 0);
    int endY = parse_get_var(buf, "ey", 0);
    int offsetX = parse_get_var(buf, "offx", 0);
    int offsetY = parse_get_var(buf, "offy", 0);
    int totalX = parse_get_var(buf, "tx", 0);
    int totalY = parse_get_var(buf, "ty", 0);
    int outputX = parse_get_var(buf, "ox", 0);
    int outputY = parse_get_var(buf, "oy", 0);
    bool scale = parse_get_var(buf, "scale", 0) == 1;
    bool binning = parse_get_var(buf, "binning", 0) == 1;
    free(buf);

    ESP_LOGI(TAG, "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    if (res)
    {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    extern const unsigned char index_ov2640_html_gz_start[] asm("_binary_index_ov2640_html_gz_start");
    extern const unsigned char index_ov2640_html_gz_end[] asm("_binary_index_ov2640_html_gz_end");
    size_t index_ov2640_html_gz_len = index_ov2640_html_gz_end - index_ov2640_html_gz_start;

    extern const unsigned char index_ov3660_html_gz_start[] asm("_binary_index_ov3660_html_gz_start");
    extern const unsigned char index_ov3660_html_gz_end[] asm("_binary_index_ov3660_html_gz_end");
    size_t index_ov3660_html_gz_len = index_ov3660_html_gz_end - index_ov3660_html_gz_start;

    extern const unsigned char index_ov5640_html_gz_start[] asm("_binary_index_ov5640_html_gz_start");
    extern const unsigned char index_ov5640_html_gz_end[] asm("_binary_index_ov5640_html_gz_end");
    size_t index_ov5640_html_gz_len = index_ov5640_html_gz_end - index_ov5640_html_gz_start;

    extern const unsigned char index_gc2145_html_gz_start[] asm("_binary_index_gc2145_html_gz_start");
    extern const unsigned char index_gc2145_html_gz_end[] asm("_binary_index_gc2145_html_gz_end");
    size_t index_gc2145_html_gz_len = index_gc2145_html_gz_end - index_gc2145_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        if (s->id.PID == OV3660_PID)
        {
            return httpd_resp_send(req, (const char *)index_ov3660_html_gz_start, index_ov3660_html_gz_len);
        }
        else if (s->id.PID == OV5640_PID)
        {
            return httpd_resp_send(req, (const char *)index_ov5640_html_gz_start, index_ov5640_html_gz_len);
        }
         else if (s->id.PID == GC2145_PID)
        {
            return httpd_resp_send(req, (const char *)index_gc2145_html_gz_start, index_gc2145_html_gz_len);
        }
        else
        {
            return httpd_resp_send(req, (const char *)index_ov2640_html_gz_start, index_ov2640_html_gz_len);
        }
    }
    else
    {
        ESP_LOGE(TAG, "Camera sensor not found");
        return httpd_resp_send_500(req);
    }
}

static esp_err_t monitor_handler(httpd_req_t *req)
{
    extern const unsigned char monitor_html_gz_start[] asm("_binary_monitor_html_gz_start");
    extern const unsigned char monitor_html_gz_end[] asm("_binary_monitor_html_gz_end");
    size_t monitor_html_gz_len = monitor_html_gz_end - monitor_html_gz_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)monitor_html_gz_start, monitor_html_gz_len);
}

httpd_config_t myconfig = HTTPD_DEFAULT_CONFIG();
void register_httpd(const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    gReturnFB = return_fb;

    
    myconfig.max_uri_handlers = 13;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL};

    httpd_uri_t cmd_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL};

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL};

    httpd_uri_t stream_uri = {
        .uri = "/stream",  //stream
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    httpd_uri_t xclk_uri = {
        .uri = "/xclk",
        .method = HTTP_GET,
        .handler = xclk_handler,
        .user_ctx = NULL};

    httpd_uri_t reg_uri = {
        .uri = "/reg",
        .method = HTTP_GET,
        .handler = reg_handler,
        .user_ctx = NULL};

    httpd_uri_t greg_uri = {
        .uri = "/greg",
        .method = HTTP_GET,
        .handler = greg_handler,
        .user_ctx = NULL};

    httpd_uri_t pll_uri = {
        .uri = "/pll",
        .method = HTTP_GET,
        .handler = pll_handler,
        .user_ctx = NULL};

    httpd_uri_t win_uri = {
        .uri = "/resolution",
        .method = HTTP_GET,
        .handler = win_handler,
        .user_ctx = NULL};

    httpd_uri_t mdns_uri = {
        .uri = "/mdns",
        .method = HTTP_GET,
        .handler = mdns_handler,
        .user_ctx = NULL};

    httpd_uri_t monitor_uri = {
        .uri = "/monitor",
        .method = HTTP_GET,
        .handler = monitor_handler,
        .user_ctx = NULL};


    ESP_LOGI(TAG, "Starting web server on port: '%d'", myconfig.server_port);
    if (httpd_start(&camera_httpd, &myconfig) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);

        httpd_register_uri_handler(camera_httpd, &xclk_uri);
        httpd_register_uri_handler(camera_httpd, &reg_uri);
        httpd_register_uri_handler(camera_httpd, &greg_uri);
        httpd_register_uri_handler(camera_httpd, &pll_uri);
        httpd_register_uri_handler(camera_httpd, &win_uri);

        httpd_register_uri_handler(camera_httpd, &mdns_uri);
        httpd_register_uri_handler(camera_httpd, &monitor_uri);
    }

    myconfig.server_port += 1;
    myconfig.ctrl_port += 1;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", myconfig.server_port);
    if (httpd_start(&stream_httpd, &myconfig) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
