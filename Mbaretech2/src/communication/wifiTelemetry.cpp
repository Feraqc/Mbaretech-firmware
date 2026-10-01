#include "firmwareConfig.h"
#if ENABLE_WIFI_TELEMETRY
#include "fsm/WifiTelemetry.h"
#include "telemetry/TelemetryService.h"
#if ENABLE_RECIPE_FSM
#include "fsm/ParameterService.h"
#endif
#include <WiFi.h>
#include <esp_http_server.h>
#include <cstdlib>
#include <cstring>

namespace fsm {
namespace {
struct SendJob { int fd; char text[512]; };
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
int clientFd = -1;
unsigned pendingSends = 0;
httpd_handle_t server = nullptr;

void sendWork(void* arg) {
    SendJob* job = static_cast<SendJob*>(arg);
    if (httpd_ws_get_fd_info(server, job->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
        httpd_ws_frame_t frame{};
        frame.type = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<uint8_t*>(job->text);
        frame.len = strlen(job->text);
        if (httpd_ws_send_frame_async(server, job->fd, &frame) != ESP_OK) {
            portENTER_CRITICAL(&wifiMux);
            if (clientFd == job->fd) clientFd = -1;
            portEXIT_CRITICAL(&wifiMux);
        }
    }
    portENTER_CRITICAL(&wifiMux);
    if (pendingSends) --pendingSends;
    portEXIT_CRITICAL(&wifiMux);
    free(job);
}

esp_err_t websocketHandler(httpd_req_t* request) {
    if (request->method == HTTP_GET) {
        const int nextFd = httpd_req_to_sockfd(request);
        portENTER_CRITICAL(&wifiMux);
        const int previousFd = clientFd;
        clientFd = nextFd;
        portEXIT_CRITICAL(&wifiMux);
        if (previousFd >= 0 && previousFd != nextFd) httpd_sess_trigger_close(server, previousFd);
        telemetryAwaitWifiHello();
        telemetryPublishHello();
        return ESP_OK;
    }
    // La tarea HTTP sólo encola; validación y aplicación corren fuera del callback.
    httpd_ws_frame_t frame{};
    if (httpd_ws_recv_frame(request, &frame, 0) != ESP_OK || frame.len >= 1024) return ESP_FAIL;
    if (frame.len) {
        uint8_t incoming[1024] = {};
        frame.payload = incoming;
        if (httpd_ws_recv_frame(request, &frame, sizeof(incoming)) != ESP_OK) return ESP_FAIL;
#if ENABLE_RECIPE_FSM
        if (frame.type == HTTPD_WS_TYPE_TEXT)
            fsm::parameterSubmitFrame(reinterpret_cast<const char*>(incoming), frame.len);
#endif
    }
    return ESP_OK;
}
}

bool sendWifiTelemetry(const char* text) {
    portENTER_CRITICAL(&wifiMux);
    const int fd = clientFd;
    const bool capacity = pendingSends < 8;
    if (fd >= 0 && capacity) ++pendingSends;
    portEXIT_CRITICAL(&wifiMux);
    if (fd < 0 || !server || !capacity) return false;
    const size_t length = strlen(text);
    SendJob* job = length < sizeof(SendJob::text) ? static_cast<SendJob*>(malloc(sizeof(SendJob))) : nullptr;
    if (!job) {
        portENTER_CRITICAL(&wifiMux); --pendingSends; portEXIT_CRITICAL(&wifiMux);
        return false;
    }
    job->fd = fd;
    memcpy(job->text, text, length + 1);
    if (httpd_queue_work(server, sendWork, job) == ESP_OK) return true;
    free(job);
    portENTER_CRITICAL(&wifiMux); --pendingSends; portEXIT_CRITICAL(&wifiMux);
    return false;
}

bool wifiTelemetryConnected() {
    portENTER_CRITICAL(&wifiMux);
    const bool connected = server && clientFd >= 0;
    portEXIT_CRITICAL(&wifiMux);
    return connected;
}

void startWifiTelemetry() {
#if WIFI_TELEMETRY_USE_STA
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_TELEMETRY_STA_SSID, WIFI_TELEMETRY_STA_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && uint32_t(millis() - started) < 10000) delay(50);
    if (WiFi.status() != WL_CONNECTED) {
        telemetryPublishText(TelemetryType::Error, "STA_CONNECT");
        return;
    }
#else
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_TELEMETRY_AP_SSID, WIFI_TELEMETRY_AP_PASSWORD)) {
        telemetryPublishText(TelemetryType::Error, "AP_START");
        return;
    }
#endif
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.task_priority = 1;
    config.stack_size = 6144;
    config.max_open_sockets = 3;
    if (httpd_start(&server, &config) != ESP_OK) {
        telemetryPublishText(TelemetryType::Error, "HTTP_START");
        return;
    }
    httpd_uri_t route{};
    route.uri = "/ws";
    route.method = HTTP_GET;
    route.handler = websocketHandler;
    route.is_websocket = true;
    if (httpd_register_uri_handler(server, &route) != ESP_OK) {
        httpd_stop(server); server = nullptr;
        telemetryPublishText(TelemetryType::Error, "WS_ROUTE");
    }
}
} // namespace fsm
#endif
