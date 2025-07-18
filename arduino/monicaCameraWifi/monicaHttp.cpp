#include <esp_timer.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <fb_gfx.h>
#include <Arduino.h>

#include "monicaHttpIndex.h"
#include "monicaHttp.h"

typedef struct
{
    httpd_req_t *req;
    size_t len;

} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321" // A boundary used to split MIME streams
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

MonicaHttp::MonicaHttp()
{
}

void MonicaHttp::initialize()
{
    start_camera_server();
}

void MonicaHttp::start_camera_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

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
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};

    if (httpd_start(&camera_httpd_, &config) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd_, &index_uri);
        httpd_register_uri_handler(camera_httpd_, &status_uri);
        httpd_register_uri_handler(camera_httpd_, &cmd_uri);
        httpd_register_uri_handler(camera_httpd_, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&stream_httpd_, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd_, &stream_uri);
    }
}

esp_err_t MonicaHttp::index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t MonicaHttp::status_handler(httpd_req_t *req)
{
    static char json_response[1024];
    memset(json_response, 0, sizeof(json_response));

    sensor_t *camera = esp_camera_sensor_get();
    char *p = json_response;

    p += sprintf(p, "{");
    p += sprintf(p, "\"framesize\":%u,", camera->status.framesize);
    p += sprintf(p, "\"quality\":%u,", camera->status.quality);
    p += sprintf(p, "\"brightness\":%d,", camera->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", camera->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", camera->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", camera->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", camera->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", camera->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", camera->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", camera->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", camera->status.aec);
    p += sprintf(p, "\"aec2\":%u,", camera->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", camera->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", camera->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", camera->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", camera->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", camera->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", camera->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", camera->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", camera->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", camera->status.lenc);
    p += sprintf(p, "\"vflip\":%u,", camera->status.vflip);
    p += sprintf(p, "\"hmirror\":%u,", camera->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", camera->status.dcw);
    p += sprintf(p, "\"colorbar\":%u,", camera->status.colorbar);
    // p += sprintf(p, "\"face_detect\":%u,", detection_enabled);
    // p += sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    // p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    // p += sprintf(p, "\"fls\":%u", flashL);
    p += sprintf(p, "}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

esp_err_t MonicaHttp::cmd_handler(httpd_req_t *req)
{
    char *buffer;
    size_t length;
    char variable[32] = {
        0,
    };
    char value[32] = {
        0,
    };

    // 1. Get Data
    length = httpd_req_get_url_query_len(req) + 1;
    if (length > 1)
    {
        buffer = (char *)malloc(length);

        if (buffer == nullptr)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buffer, length) == ESP_OK)
        {
            // 2. Parsing Data
            if (httpd_query_key_value(buffer, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buffer, "val", value, sizeof(value)) == ESP_OK)
            {
                // Process the variable and value Later
            }
            else
            {
                free(buffer);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        }
        else
        {
            free(buffer);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buffer);
    }
    else
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // 3. Update the Data
    int select_value = atoi(value);
    String variable_str = String(variable);

    sensor_t *camera = esp_camera_sensor_get();
    int result = 0;

    if (variable_str == "framesize")
    {
        if (camera->pixformat == PIXFORMAT_JPEG)
        {
            result = camera->set_framesize(camera, (framesize_t)select_value);
        }
    }
    else if (variable_str == "quality")
    {
        result = camera->set_quality(camera, select_value);
    }
    else if (variable_str == "contrast")
    {
        result = camera->set_contrast(camera, select_value);
    }
    else if (variable_str == "brightness")
    {
        result = camera->set_brightness(camera, select_value);
    }
    else if (variable_str == "saturation")
    {
        result = camera->set_saturation(camera, select_value);
    }
    else if (variable_str == "gainceiling")
    {
        result = camera->set_gainceiling(camera, (gainceiling_t)select_value);
    }
    else if (variable_str == "colorbar")
    {
        result = camera->set_colorbar(camera, select_value);
    }
    else if (variable_str == "awb")
    {
        result = camera->set_whitebal(camera, select_value);
    }
    else if (variable_str == "agc")
    {
        result = camera->set_gain_ctrl(camera, select_value);
    }
    else if (variable_str == "aec")
    {
        result = camera->set_exposure_ctrl(camera, select_value);
    }
    else if (variable_str == "hmirror")
    {
        result = camera->set_hmirror(camera, select_value);
    }
    else if (variable_str == "vflip")
    {
        result = camera->set_vflip(camera, select_value);
    }
    else if (variable_str == "awb_gain")
    {
        result = camera->set_awb_gain(camera, select_value);
    }
    else if (variable_str == "agc_gain")
    {
        result = camera->set_agc_gain(camera, select_value);
    }
    else if (variable_str == "aec_value")
    {
        result = camera->set_aec_value(camera, select_value);
    }
    else if (variable_str == "aec2")
    {
        result = camera->set_aec2(camera, select_value);
    }
    else if (variable_str == "dcw")
    {
        result = camera->set_dcw(camera, select_value);
    }
    else if (variable_str == "bpc")
    {
        result = camera->set_bpc(camera, select_value);
    }
    else if (variable_str == "wpc")
    {
        result = camera->set_wpc(camera, select_value);
    }
    else if (variable_str == "raw_gma")
    {
        result = camera->set_raw_gma(camera, select_value);
    }
    else if (variable_str == "lenc")
    {
        result = camera->set_lenc(camera, select_value);
    }
    else if (variable_str == "special_effect")
    {
        result = camera->set_special_effect(camera, select_value);
    }
    else if (variable_str == "wb_mode")
    {
        result = camera->set_wb_mode(camera, select_value);
    }
    else if (variable_str == "ae_level")
    {
        result = camera->set_ae_level(camera, select_value);
    }
    else if (variable_str == "sharpness")
    {
        result = camera->set_sharpness(camera, select_value);
    }
    else
    {
        result = -1;
    }

    if (result)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t MonicaHttp::capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (fb == nullptr)
    {
        // Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (fb->width > 400)
    {
        size_t fb_len = 0;
        if (fb->format == PIXFORMAT_JPEG)
        {
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb_len);
        }
        else
        {
            jpg_chunking_t jchunk = {req, 0};
            res = (frame2jpg_cb(fb, 80, MonicaHttp::jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL);
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }

        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        // Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    // Skip Face Detection
    esp_camera_fb_return(fb);
    return res;
}

size_t MonicaHttp::jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (index == 0)
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

esp_err_t MonicaHttp::stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while (true)
    {
        fb = esp_camera_fb_get();
        if (fb == nullptr)
        {
            // Serial.println("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            if (fb->width > 400)
            {
                if (fb->format != PIXFORMAT_JPEG)
                {
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if (!jpeg_converted)
                    {
                        // Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                }
                else
                {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            }
        }
        
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            break;
        }
        // Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
    }
    return res;
}
