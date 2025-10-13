/*
 * ESP32 Classic BT (SPP) + Wi-Fi Provisioning for Flutter App
 * Online-only 1Hz Sampling (MQ9 AO + VOC AO + PMS7003) → Firebase RTDB
 * 
 * 통신 프로토콜:
 * Flutter → ESP32:
 *   - "SCAN" : WiFi 스캔 요청
 *   - "CONNECT:SSID|PASSWORD" : WiFi 연결 요청
 * 
 * ESP32 → Flutter:
 *   - "WIFI_LIST_START" : WiFi 목록 시작
 *   - "WIFI:인덱스:SSID:신호강도:암호화타입" : WiFi 네트워크 정보
 *   - "WIFI_LIST_END" : WiFi 목록 끝
 *   - "ip:주소" : 연결 성공, IP 주소
 *   - "ok:connected" : 연결 성공 확인
 *   - "error:메시지" : 에러 발생
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "esp_bt.h"
#include "esp_bt_main.h"

#define LOGI(...) \
  do { \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while (0)
#define LOGW(...) \
  do { \
    Serial.print("[W] "); \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while (0)
#define LOGE(...) \
  do { \
    Serial.print("[E] "); \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while (0)

/************** 유형 **************/
struct SampleItem {
  String key;
  String value;
};
struct PMSData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
};

/************** 설정 **************/
#define BT_NAME "green"

// Firebase RTDB
static const char* HOST_PRIMARY = "jabswosseo-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* HOST_BACKUP = "jabswosseo-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* PATH_BASE = "/sensor_data_2.json";

// MQ-9 / VOC 핀
#define PIN_MQ9_AO 34
#define PIN_VOC_AO 35

// PMS7003 UART
#define PMS_RX 16
#define PMS_TX 17
HardwareSerial pmsSerial(2);

// 연결 후 샘플링 시작까지 예열 지연(ms)
#define WARMUP_MS 3000

/************** 전역 **************/
BluetoothSerial SerialBT;
Preferences prefs;

enum ProvState { WAIT_COMMAND,
                 TRY_CONNECT,
                 ONLINE };
volatile ProvState state_ = WAIT_COMMAND;

String ssid_, pass_, rxBuf_;
volatile bool btEnabled = false;
unsigned long rxLastMs_ = 0;

// 최신 샘플 (MQ9/VOC)
volatile float lastMQ9_V = 0.0f;
volatile float lastVOC_V = 0.0f;
volatile bool haveSample = false;

// 최신 샘플 (PMS7003)
PMSData lastPMS;
volatile bool havePMS = false;

/************** 시간(KST) **************/
void initTimeKST() {
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  setenv("TZ", "KST-9", 1);
  tzset();
}

bool isTimeSet() {
  time_t now = 0;
  time(&now);
  return now > 1600000000;
}

void waitForTimeSyncAfterWiFi(uint32_t ms = 4000) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (isTimeSet()) return;
    delay(100);
  }
}

String nowIso8601KST() {
  time_t now;
  time(&now);
  struct tm lt;
  localtime_r(&now, &lt);
  char b[32];
  snprintf(b, sizeof(b), "%04d-%02d-%02dT%02d:%02d:%02d+09:00",
           lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(b);
}

String nowDateTimeKey() {
  time_t now;
  time(&now);
  struct tm lt;
  localtime_r(&now, &lt);
  char b[32];
  snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d",
           lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(b);
}

/************** BT **************/
void startClassicBT() {
  if (!SerialBT.begin(BT_NAME)) {
    LOGE("[BT] begin failed");
    btEnabled = false;
  } else {
    btEnabled = true;
    LOGI("[BT] SPP started: %s", BT_NAME);
  }
}

void stopClassicBT() {
  if (!btEnabled) return;
  SerialBT.end();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  btEnabled = false;
  LOGI("[BT] stopped");
}

/************** Wi-Fi **************/
bool tryWiFiConnectOnce(const String& s, const String& p, uint32_t ms = 15000) {
  LOGI("[WiFi] begin ssid=\"%s\" pw_len=%d", s.c_str(), p.length());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  delay(120);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(s.c_str(), p.c_str());

  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

inline bool onlineReady() {
  return (WiFi.status() == WL_CONNECTED) && isTimeSet();
}

/************** TLS **************/
WiFiClientSecure clientPri, clientBak;
bool priConnected = false, bakConnected = false;

bool ensureClient(WiFiClientSecure& c, bool& flag, const char* host) {
  if (flag && c.connected()) return true;
  c.stop();
  c.setInsecure();
  c.setTimeout(900);
  if (!c.connect(host, 443)) {
    flag = false;
    return false;
  }
  flag = true;
  return true;
}

/************** PATCH **************/
bool send_patch_keepalive(WiFiClientSecure& c, bool& flag, const char* host,
                          const char* path_json, const SampleItem& it) {
  if (!ensureClient(c, flag, host)) {
    LOGW("[HTTP] connect fail to %s", host);
    return false;
  }

  String body = "{\"" + it.key + "\":" + it.value + "}";
  c.printf("PATCH %s HTTP/1.1\r\n", path_json);
  c.printf("Host: %s\r\n", host);
  c.println("User-Agent: ESP32");
  c.println("Content-Type: application/json");
  c.printf("Content-Length: %d\r\n", body.length());
  c.println("Connection: keep-alive\r\n");
  c.print(body);

  uint32_t t0 = millis();
  while (c.connected() && (millis() - t0 < 600)) {
    String line = c.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  String resp;
  t0 = millis();
  while (c.connected() && (millis() - t0 < 250)) {
    while (c.available()) {
      char ch = c.read();
      resp += ch;
      if (resp.length() > 256) break;
    }
    if (resp.length() > 0) break;
    delay(1);
  }

  if (resp.length() == 0) {
    LOGW("[RESP] empty(timeout)");
    c.stop();
    flag = false;
    return false;
  }
  LOGI("[RESP] %s", resp.c_str());
  return true;
}

bool rtdb_patch_single_with_fallback(const SampleItem& it) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (send_patch_keepalive(clientPri, priConnected, HOST_PRIMARY, PATH_BASE, it))
    return true;
  LOGW("[HTTP] primary failed, try backup…");
  return send_patch_keepalive(clientBak, bakConnected, HOST_BACKUP, PATH_BASE, it);
}

/************** PMS7003 **************/
bool readPMS(PMSData& data) {
  if (pmsSerial.available() < 32) return false;
  if (pmsSerial.read() != 0x42) return false;
  if (pmsSerial.read() != 0x4D) return false;

  uint8_t buffer[30];
  if (pmsSerial.readBytes(buffer, 30) != 30) return false;

  uint16_t frameLen = (buffer[0] << 8) | buffer[1];
  if (frameLen != 28) return false;

  data.pm1_0 = (buffer[10] << 8) | buffer[11];
  data.pm2_5 = (buffer[12] << 8) | buffer[13];
  data.pm10 = (buffer[14] << 8) | buffer[15];

  return true;
}

/************** JSON **************/
String makeSampleValueJSON(float mq9_v, float voc_v, const PMSData& pms, bool havePMS) {
  String ts = nowIso8601KST();
  String v = "{";
  v += "\"timestamp\":\"" + ts + "\"";
  v += ",\"MQ9_V\":" + String(mq9_v, 3);
  v += ",\"VOC_V\":" + String(voc_v, 3);
  if (havePMS) {
    v += ",\"PM1_0\":" + String(pms.pm1_0);
    v += ",\"PM2_5\":" + String(pms.pm2_5);
    v += ",\"PM10\":" + String(pms.pm10);
  }
  v += "}";
  return v;
}

/************** 샘플링 태스크 **************/
void taskSample1Hz(void*) {
  LOGI("[TASK] Sample1Hz start (online-only)");
  const TickType_t period = pdMS_TO_TICKS(1000);
  TickType_t last;

  for (;;) {
    while (!onlineReady()) {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(WARMUP_MS));
    last = xTaskGetTickCount();

    while (onlineReady()) {
      const float ADC_REF = 3.3f;

      // MQ9 AO
      uint32_t acc1 = 0;
      for (int i = 0; i < 4; i++) {
        acc1 += analogRead(PIN_MQ9_AO);
        delay(1);
      }
      float mq9_v = (acc1 / 4.0f) * (ADC_REF / 4095.0f);

      // VOC AO
      uint32_t acc2 = 0;
      for (int i = 0; i < 4; i++) {
        acc2 += analogRead(PIN_VOC_AO);
        delay(1);
      }
      float voc_v = (acc2 / 4.0f) * (ADC_REF / 4095.0f);

      lastMQ9_V = mq9_v;
      lastVOC_V = voc_v;
      haveSample = true;

      PMSData tmp;
      if (readPMS(tmp)) {
        lastPMS = tmp;
        havePMS = true;
      }

      if (havePMS) {
        LOGI("[SAMPLE] MQ9_V=%.3f V | VOC_V=%.3f V | PM1.0=%u, PM2.5=%u, PM10=%u",
             mq9_v, voc_v, lastPMS.pm1_0, lastPMS.pm2_5, lastPMS.pm10);
      } else {
        LOGI("[SAMPLE] MQ9_V=%.3f V | VOC_V=%.3f V | PMS=NO DATA", mq9_v, voc_v);
      }

      vTaskDelayUntil(&last, period);
    }
  }
}

/************** 업로드 태스크 **************/
void taskUpload1Hz(void*) {
  LOGI("[TASK] Upload1Hz start");
  const TickType_t period = pdMS_TO_TICKS(1000);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    if (onlineReady() && haveSample) {
      SampleItem it;
      it.key = nowDateTimeKey();
      PMSData tmpPMS = lastPMS;
      it.value = makeSampleValueJSON(lastMQ9_V, lastVOC_V, tmpPMS, havePMS);
      bool ok = rtdb_patch_single_with_fallback(it);
      if (ok) LOGI("[UPLOAD] ok");
      else LOGW("[UPLOAD] fail");
    }
    vTaskDelayUntil(&last, period);
  }
}

/************** WiFi 스캔 및 전송 **************/
void doWiFiScanAndSend() {
  if (!btEnabled) return;

  LOGI("[SCAN] Starting WiFi scan...");
  SerialBT.println("WIFI_LIST_START");

  int n = WiFi.scanNetworks();
  LOGI("[SCAN] Found %d networks", n);

  if (n > 0) {
    for (int i = 0; i < n; i++) {
      String encType = "WPA2";
      wifi_auth_mode_t authMode = WiFi.encryptionType(i);
      if (authMode == WIFI_AUTH_OPEN) encType = "OPEN";
      else if (authMode == WIFI_AUTH_WEP) encType = "WEP";
      else if (authMode == WIFI_AUTH_WPA_PSK) encType = "WPA";
      else if (authMode == WIFI_AUTH_WPA2_PSK) encType = "WPA2";
      else if (authMode == WIFI_AUTH_WPA_WPA2_PSK) encType = "WPA/WPA2";

      // 형식: WIFI:인덱스:SSID:신호강도:암호화타입
      SerialBT.printf("WIFI:%d:%s:%d:%s\n",
                      i,
                      WiFi.SSID(i).c_str(),
                      WiFi.RSSI(i),
                      encType.c_str());
      delay(10);  // 전송 안정성
    }
  }

  SerialBT.println("WIFI_LIST_END");
  LOGI("[SCAN] WiFi list sent");
}

/************** 명령어 처리 **************/
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  LOGI("[CMD] Received: %s", cmd.c_str());

  // SCAN 명령
  if (cmd == "SCAN") {
    doWiFiScanAndSend();
  }
  // CONNECT 명령: CONNECT:SSID|PASSWORD
  else if (cmd.startsWith("CONNECT:")) {
    String data = cmd.substring(8);  // "CONNECT:" 제거
    int sepIndex = data.indexOf('|');

    if (sepIndex > 0) {
      ssid_ = data.substring(0, sepIndex);
      pass_ = data.substring(sepIndex + 1);

      LOGI("[CMD] Connect request - SSID: %s, Pass length: %d",
           ssid_.c_str(), pass_.length());

      state_ = TRY_CONNECT;
    } else {
      SerialBT.println("error:invalid connect format");
      LOGE("[CMD] Invalid CONNECT format");
    }
  } else {
    SerialBT.println("error:unknown command");
    LOGW("[CMD] Unknown command: %s", cmd.c_str());
  }
}

/************** setup / loop **************/
void setup() {
  Serial.begin(115200);
  delay(200);
  LOGI("[BOOT] setup() start, heap=%u", ESP.getFreeHeap());

  esp_bt_mem_release(ESP_BT_MODE_BLE);
  startClassicBT();
  initTimeKST();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ9_AO, ADC_11db);
  analogSetPinAttenuation(PIN_VOC_AO, ADC_11db);

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);

  xTaskCreatePinnedToCore(taskSample1Hz, "Sample1Hz", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskUpload1Hz, "Upload1Hz", 6144, nullptr, 1, nullptr, 1);

  LOGI("[TASK] Sample+Upload spawned");
  LOGI("[BOOT] setup() done");
}

void loop() {
  // Bluetooth 명령어 수신 처리
  if (btEnabled && state_ != TRY_CONNECT) {
    while (SerialBT.available()) {
      char c = (char)SerialBT.read();
      if (c == '\r') c = '\n';
      if (c == '\n') {
        handleCommand(rxBuf_);
        rxBuf_.clear();
      } else {
        rxBuf_ += c;
        if (rxBuf_.length() > 200) rxBuf_.clear();
      }
      rxLastMs_ = millis();
      yield();
    }

    // 타임아웃 처리
    if (rxBuf_.length() > 0 && (millis() - rxLastMs_ > 1200)) {
      handleCommand(rxBuf_);
      rxBuf_.clear();
    }
  }

  // WiFi 연결 시도
  static bool tried = false;
  if (state_ == TRY_CONNECT && !tried) {
    tried = true;
    LOGI("[WiFi] Connecting to %s...", ssid_.c_str());

    if (btEnabled) {
      SerialBT.printf("Connecting to %s...\n", ssid_.c_str());
    }

    bool ok = tryWiFiConnectOnce(ssid_, pass_, 15000);

    if (ok) {
      String ip = WiFi.localIP().toString();
      LOGI("[WiFi] Connected! IP=%s", ip.c_str());

      if (btEnabled) {
        SerialBT.printf("ip:%s\n", ip.c_str());
        SerialBT.println("ok:connected");
        delay(500);
      }

      // ✅ 여기에 아래 코드 추가
      WiFiClient test;
      if (test.connect("google.com", 80))
        Serial.println("Internet OK");
      else
        Serial.println("Internet FAIL");

      waitForTimeSyncAfterWiFi();
      state_ = ONLINE;

      if (btEnabled) {
        stopClassicBT();
      }

      LOGI("[NET] Online ready → sampling will start after warmup");
    } else {
      LOGW("[WiFi] Connection failed");

      if (btEnabled) {
        SerialBT.println("error:connection failed");
      }

      state_ = WAIT_COMMAND;
      tried = false;

      // 연결 실패 시 Bluetooth 유지
    }
  }

  // 온라인 상태 모니터링
  static uint32_t lastChk = 0;
  if (millis() - lastChk > 2000) {
    lastChk = millis();

    if (state_ == ONLINE && !onlineReady()) {
      LOGW("[NET] Offline detected");
      state_ = WAIT_COMMAND;
      tried = false;

      if (!btEnabled) {
        startClassicBT();
      }
    }
  }

  delay(2);
}