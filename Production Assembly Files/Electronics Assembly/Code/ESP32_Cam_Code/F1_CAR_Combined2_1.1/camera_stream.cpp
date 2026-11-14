// ---------------------------------------------------------------------
// MJPEG stream handler — port 81, "/stream"
//
// Lives in its own .cpp file, deliberately separate from the main .ino,
// so this file's esp_http_server.h include never shares a translation
// unit with ESPAsyncWebServer.h (see stream_server.h for why that
// matters — it's what was causing your build error).
//
// Same multipart/x-mixed-replace pattern the stock CameraWebServer
// example uses, trimmed down and running on its own dedicated httpd
// instance so it never waits on anything the control server (port 80,
// in the .ino) is doing.
// ---------------------------------------------------------------------

#include <Arduino.h>    // .cpp files (unlike .ino) don't get this auto-included — needed for Serial
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "stream_server.h"

static httpd_handle_t stream_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpg_buf_len = 0;
  uint8_t *jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        // Only hit when STREAM_GRAYSCALE is true — raw frame gets
        // software-encoded here (see stream_server.h note on cost).
        bool ok = frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg_buf, &jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!ok) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        jpg_buf_len = fb->len;
        jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
    } else if (jpg_buf) {
      free(jpg_buf);
    }
    jpg_buf = NULL;

    if (res != ESP_OK) break; // client disconnected — end this handler
  }
  return res;
}

// ---------------------------------------------------------------------
// Single-frame snapshot — port 81, "/capture"
//
// Added because the phone's browser was rendering the /stream multipart
// feed unreliably when embedded in an <img> (broken-image icon inline,
// yet the same URL worked fine opened directly in a tab) — a known
// inconsistency in how mobile Chromium/Blink handles
// multipart/x-mixed-replace specifically inside <img src>, vs. as a
// direct page load. controllerV3.html now polls this endpoint rapidly
// instead, which sidesteps that quirk entirely since it's just a plain
// JPEG file every time — no multipart parsing involved, so it renders
// consistently everywhere (including iOS Safari).
// ---------------------------------------------------------------------
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

  esp_err_t res;
  if (fb->format == PIXFORMAT_JPEG) {
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    // Only hit when STREAM_GRAYSCALE is true — see stream_server.h note.
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    if (frame2jpg(fb, STREAM_JPEG_QUALITY, &jpg_buf, &jpg_len)) {
      res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
      free(jpg_buf);
    } else {
      httpd_resp_send_500(req);
      res = ESP_FAIL;
    }
  }
  esp_camera_fb_return(fb);
  return res;
}

// ---------------------------------------------------------------------
// CORS preflight — OPTIONS handler for /stream and /capture
//
// Chromium-based browsers (Chrome, Samsung Internet, etc.) treat a
// script-initiated request (fetch()/XHR/Image()) to a private-network
// address like 192.168.4.1 as a "Private Network Access" request. Before
// letting the real GET through, the browser silently sends an OPTIONS
// preflight and checks for this exact header in the response. With no
// OPTIONS handler at all, there's nothing to answer that preflight, so
// the browser aborts and the page just sees a bare "Failed to fetch" —
// no CORS error, no network error, nothing actionable. Plain top-level
// navigation (typing the URL directly) is NOT subject to this check,
// which is why that always worked while the polling never did.
// ---------------------------------------------------------------------
static esp_err_t preflight_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
  httpd_resp_set_hdr(req, "Access-Control-Max-Age", "600");
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port    = 32769; // must differ from the control server's internal port
  config.max_open_sockets = 4;
  config.lru_purge_enable = true;

  httpd_uri_t stream_uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = stream_handler,
    .user_ctx = NULL
  };
  httpd_uri_t capture_uri = {
    .uri      = "/capture",
    .method   = HTTP_GET,
    .handler  = capture_handler,
    .user_ctx = NULL
  };
  httpd_uri_t stream_opts_uri = {
    .uri      = "/stream",
    .method   = HTTP_OPTIONS,
    .handler  = preflight_handler,
    .user_ctx = NULL
  };
  httpd_uri_t capture_opts_uri = {
    .uri      = "/capture",
    .method   = HTTP_OPTIONS,
    .handler  = preflight_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
    httpd_register_uri_handler(stream_httpd, &stream_opts_uri);
    httpd_register_uri_handler(stream_httpd, &capture_opts_uri);
    Serial.println("Stream server started on port 81 (/stream + /capture, with PNA preflight)");
  } else {
    Serial.println("Failed to start stream server");
  }
}
