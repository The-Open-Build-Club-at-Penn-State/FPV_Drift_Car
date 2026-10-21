/*
  =====================================================================
  ESP32-CAM F1 RC Car — Access Point Control + Live MJPEG Stream
  =====================================================================
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

#include "esp_camera.h"     
#include "stream_server.h"  

// ------------------------------------------------------------------
// CAMERA PIN MAP — AI-Thinker ESP32-CAM
// ------------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ------------------------------------------------------------------
// USER CONFIG 
// ------------------------------------------------------------------
const char* AP_SSID     = "F1_CAR";
const char* AP_PASSWORD = "racecar1";

const int SERVO_CENTER    = 90;
const int SERVO_LEFT_MAX  = 45;
const int SERVO_RIGHT_MAX = 135;

const int PWM_FREQ = 5000;   
const int PWM_RES  = 8;      

const unsigned long COMMAND_TIMEOUT = 500;
const unsigned long FRAME_INTERVAL_MS = 100; 

// ------------------------------------------------------------------
// MOTOR / SERVO / LED PIN DEFINITIONS 
// ------------------------------------------------------------------
#define PWMA_PIN  12
#define MotorPin  14
#define SERVO_PIN 13
#define LED_PIN   2

// ------------------------------------------------------------------
// GLOBALS 
// ------------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile int steerValue    = 0;  
volatile int throttleValue = 0;  
unsigned long lastCommandTime = 0;
unsigned long lastFrameTime   = 0;

// ------------------------------------------------------------------
// FORWARD DECLARATIONS
// ------------------------------------------------------------------
void applyControls();
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
bool initCamera();

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  if (!initCamera()) {
    Serial.println("Camera init FAILED — check wiring/board pin map. Continuing without video.");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // General PWM Pin
  ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWMA_PIN, 0);

  // --- HARDWARE LEDC FOR SERVO AND ESC ---
  // Attach both to 50Hz with 16-bit resolution
  ledcAttach(SERVO_PIN, 50, 16);
  ledcAttach(MotorPin, 50, 16);

  // Write neutral positions (1500us) immediately so they don't runaway
  // 50Hz = 20,000us period. (1500 * 65535) / 20000 = 4915 duty cycle
  uint32_t centerDuty = (1500 * 65535) / 20000;
  ledcWrite(SERVO_PIN, centerDuty);
  ledcWrite(MotorPin, centerDuty);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); 

  Serial.print("AP started. Connect to WiFi \"");
  Serial.print(AP_SSID);
  Serial.println("\", then open controllerV5.html on your phone.");
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "F1_CAR ESP32 is running.\nControl + video: ws://192.168.4.1/ws");
  });
  server.begin();

  lastCommandTime = millis();
}

// ------------------------------------------------------------------
// LOOP 
// ------------------------------------------------------------------
void loop() {
  ws.cleanupClients();

  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (steerValue != 0 || throttleValue != 0) {
      steerValue = 0;
      throttleValue = 0;
      applyControls();
    }
  }

  if (ws.count() > 0 && millis() - lastFrameTime >= FRAME_INTERVAL_MS) {
    lastFrameTime = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG) {
        ws.binaryAll(fb->buf, fb->len);
      }
      esp_camera_fb_return(fb);
    }
  }
}

// ------------------------------------------------------------------
// CONTROLS (Hardware LEDC version)
// ------------------------------------------------------------------
void applyControls() {
  const int DEADBAND = 5; 

  // --- STEERING ---
  static int lastValidSteer = 0;
  static int lastAppliedServoUs = -999;

  // 1. Restore the UI glitch filter (ignores random 0s dropped by the web client/JSON)
  if (steerValue == 0 && abs(lastValidSteer) > 15) {
    steerValue = lastValidSteer; 
  } else {
    lastValidSteer = steerValue;
  }

  int currentSteer = steerValue;

  // 2. Apply center deadband (ignores micro-jitters near center)
  if (currentSteer > -DEADBAND && currentSteer < DEADBAND) {
    currentSteer = 0;
  }

  // 3. Map and write hardware PWM
  int servoUs = map(currentSteer, -100, 100, 1000, 2000); 
  if (servoUs != lastAppliedServoUs) {
    uint32_t duty = (servoUs * 65535) / 20000;
    ledcWrite(SERVO_PIN, duty);
    lastAppliedServoUs = servoUs;
  }

  // --- ESC / MOTOR ---
  static int lastValidThrottle = 0;
  static int lastAppliedEscUs = -999;

  // 1. Restore the UI glitch filter for the throttle
  if (throttleValue == 0 && abs(lastValidThrottle) > 15) {
    throttleValue = lastValidThrottle;
  } else {
    lastValidThrottle = throttleValue;
  }

  int currentThrottle = throttleValue;

  // 2. Apply center deadband
  if (currentThrottle > -DEADBAND && currentThrottle < DEADBAND) {
    currentThrottle = 0;
  }

  // 3. Map and write hardware PWM
  int escUs = map(currentThrottle, -100, 100, 1000, 2000);
  if (escUs != lastAppliedEscUs) {
    uint32_t duty = (escUs * 65535) / 20000;
    ledcWrite(MotorPin, duty);
    lastAppliedEscUs = escUs;
  }
}

// ------------------------------------------------------------------
// WEBSOCKET HANDLER
// ------------------------------------------------------------------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected\n", client->id());
    lastCommandTime = millis();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
    steerValue = 0;
    throttleValue = 0;
    applyControls();
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      StaticJsonDocument<128> doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (!err) {
        steerValue    = constrain((int)doc["s"], -100, 100);
        throttleValue = constrain((int)doc["t"], -100, 100);
        lastCommandTime = millis();
        applyControls();
      }
    }
  }
}

// ------------------------------------------------------------------
// CAMERA INIT
// ------------------------------------------------------------------
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = STREAM_GRAYSCALE ? PIXFORMAT_GRAYSCALE : PIXFORMAT_JPEG;
  config.frame_size   = STREAM_FRAMESIZE;
  config.jpeg_quality = STREAM_JPEG_QUALITY;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_DRAM;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count    = 2;
    config.grab_mode   = CAMERA_GRAB_LATEST; 
  } else if (STREAM_FRAMESIZE > FRAMESIZE_SVGA) {
    config.frame_size = FRAMESIZE_SVGA; 
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (!STREAM_GRAYSCALE) {
    s->set_framesize(s, STREAM_FRAMESIZE);
  }

  return true;
}