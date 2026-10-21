#pragma once
// ---------------------------------------------------------------------
// Shared between F1_CAR_Combined.ino and camera_stream.cpp.
//
// Why this file exists: ESPAsyncWebServer.h and esp_http_server.h both
// declare their own HTTP_GET / HTTP_DELETE / etc. in the global
// namespace. If both headers ever land in the SAME .cpp translation
// unit, the compiler sees two conflicting declarations and refuses to
// build (this is the error you hit). Arduino compiles every .cpp file
// in a sketch folder separately, so the fix is to keep them apart:
//   - F1_CAR_Combined.ino includes ESPAsyncWebServer.h (control) and
//     esp_camera.h (camera init) — never esp_http_server.h.
//   - camera_stream.cpp includes esp_http_server.h (streaming) and
//     esp_camera.h — never ESPAsyncWebServer.h.
// This header is the only thing shared between them, and it doesn't
// touch either conflicting header itself beyond esp_camera.h, which
// has no relation to esp_http_server.h.
// ---------------------------------------------------------------------

#include "esp_camera.h" // for framesize_t

// --- Stream tuning: the main levers for FPS/latency ---
// Color, hardware-encoded JPEG is the fast path on this sensor (see
// the .ino header comment for why) — kept at false per your call.
#define STREAM_GRAYSCALE false
static const framesize_t STREAM_FRAMESIZE   = FRAMESIZE_QVGA; // 320x240. Drop to FRAMESIZE_QQVGA (160x120) if still laggy.
static const int         STREAM_JPEG_QUALITY = 15;            // 0-63, higher = smaller/faster/lower detail. Try 18-24 for even less latency.

// Implemented in camera_stream.cpp, called once from setup() in the .ino.
void startCameraServer();
