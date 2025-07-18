#ifndef _MONICA_HTTP_H_
#define _MONICA_HTTP_H_

#include <esp_http_server.h>

class MonicaHttp
{
public:
    MonicaHttp();

public:
    void initialize();

private:
    void start_camera_server();
    static esp_err_t index_handler(httpd_req_t *req);
    static esp_err_t status_handler(httpd_req_t *req);
    static esp_err_t cmd_handler(httpd_req_t *req);
    static esp_err_t capture_handler(httpd_req_t *req);
    static esp_err_t stream_handler(httpd_req_t *req);

    static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len);

private:
    httpd_handle_t camera_httpd_;
    httpd_handle_t stream_httpd_; 
};

#endif /*_MONICA_HTTP_H_*/
