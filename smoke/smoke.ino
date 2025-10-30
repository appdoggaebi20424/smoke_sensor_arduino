/*
 * ESP32 Classic BT (SPP) + Wi-Fi Provisioning for Flutter App
 * Online-only 1Hz Sampling (MQ9 AO + VOC AO + PMS7003) → Firebase RTDB
 *
 * 변경점(안정화 B2 + 로그 C2 + 시간키 A1):
 * - 온라인 판정: Wi-Fi 연결만으로 OK (NTP 의존 제거)
 * - HTTPS: 요청당 새 TLS 연결, Connection: close
 * - 업로드: 3회 재시도 + 백업 호스트 시도
 * - DNS 안정 옵션 추가(주석 해제 시 사용)
 * - 시간키 방식 유지(A1). NTP 미동기화 시 안전키(u+millis) 대체(옵션)
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "esp_bt.h"
#include "esp_bt_main.h"

/************** 로깅 매크로 **************/
#define LOGI(...) do { Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#define LOGW(...) do { Serial.print("[W] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)
#define LOGE(...) do { Serial.print("[E] "); Serial.printf(__VA_ARGS__); Serial.println(); } while (0)

/************** 유형 **************/
struct SampleItem { String key; String value; };
struct PMSData { uint16_t pm1_0; uint16_t pm2_5; uint16_t pm10; };

/************** 설정 **************/
#define BT_NAME "green"
#define BOOT_PIN 0  // BOOT 버튼은 GPIO0

// Firebase RTDB (asia-southeast1)
static const char* HOST_PRIMARY = "jabswosseo-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* HOST_BACKUP  = "jabswosseo-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* PATH_BASE    = "/sensor_data_2.json";

// MQ-9 / VOC 핀
#define PIN_MQ9_AO 34
#define PIN_VOC_AO 35

// PMS7003 UART
#define PMS_RX 16
#define PMS_TX 17
HardwareSerial pmsSerial(2);

// 연결 후 샘플링 시작까지 예열(ms)
#define WARMUP_MS 3000

/************** 전역 **************/
BluetoothSerial SerialBT;
Preferences prefs;

enum ProvState { WAIT_COMMAND, TRY_CONNECT, ONLINE };
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

/************** NVS 키 **************/
static const char* PREF_NAMESPACE = "wifi";
static const char* KEY_SSID = "ssid";
static const char* KEY_PASS = "pass";

/************** 시간(KST) **************/
void initTimeKST() {
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  setenv("TZ", "KST-9", 1);
  tzset();
}
bool isTimeSet() {
  time_t now = 0; time(&now);
  return now > 1600000000; // 2020-09-13
}
String nowIso8601KST() {
  time_t now; time(&now);
  struct tm lt; localtime_r(&now, &lt);
  char b[32];
  snprintf(b, sizeof(b), "%04d-%02d-%02dT%02d:%02d:%02d+09:00",
           lt.tm_year+1900, lt.tm_mon+1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(b);
}

String nowDateTimeKey() {
  time_t now; time(&now);
  struct tm lt; localtime_r(&now, &lt);
  char b[32];
  snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d",
           lt.tm_year+1900, lt.tm_mon+1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
  return String(b);
}

/************** 온라인 판정(안정화) **************/
inline bool wifiReady()   { return WiFi.status() == WL_CONNECTED; }
// NTP는 참고용만
inline bool timeReady()   { return isTimeSet(); }
// 운영은 Wi-Fi만으로 진행
inline bool onlineReady() { return wifiReady(); }

/************** BT **************/
void startClassicBT() {
  if (btEnabled) return;
  if (!SerialBT.begin(BT_NAME)) {
    LOGE("[BT] begin failed"); btEnabled = false;
  } else { btEnabled = true; LOGI("[BT] SPP started: %s", BT_NAME); }
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
  WiFi.setSleep(false);

  // ▼ DNS 안정 옵션(필요 시 주석 해제)
  // WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(8,8,8,8));

  WiFi.begin(s.c_str(), p.c_str());
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

/************** Wi-Fi NVS **************/
void saveWiFi(const String& s, const String& p) {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString(KEY_SSID, s);
  prefs.putString(KEY_PASS, p);
  prefs.end();
  LOGI("[NVS] WiFi saved: ssid=\"%s\" (pw_len=%d)", s.c_str(), p.length());
}
bool loadSavedWiFi(String& s, String& p) {
  prefs.begin(PREF_NAMESPACE, true);
  s = prefs.getString(KEY_SSID, "");
  p = prefs.getString(KEY_PASS, "");
  prefs.end();
  return s.length() > 0;
}
void clearWiFi() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  LOGI("[NVS] WiFi cleared");
}

/************** HTTP(요청당 새 TLS 연결, close) **************/
bool send_rtdb_patch_once(const char* host, const char* path_json, const SampleItem& it) {
  WiFiClientSecure c;
  c.setInsecure();
  c.setTimeout(6000);

  if (!c.connect(host, 443)) {
    LOGW("[HTTP] connect fail to %s", host);
    return false;
  }

  String body = "{\"" + it.key + "\":" + it.value + "}";

  c.printf("PATCH %s HTTP/1.1\r\n", path_json);
  c.printf("Host: %s\r\n", host);
  c.println("User-Agent: ESP32");
  c.println("Content-Type: application/json");
  c.printf("Content-Length: %d\r\n", body.length());
  c.println("Connection: close");                 // ★ keep-alive 금지
  c.println();
  c.print(body);

  // 상태줄
  uint32_t t0 = millis();
  String status = c.readStringUntil('\n'); status.trim();
  while (status.length() == 0 && millis() - t0 < 2000) {
    delay(5);
    if (c.available()) { status = c.readStringUntil('\n'); status.trim(); }
  }
  LOGI("[HTTP] %s", status.c_str());
  if (!status.startsWith("HTTP/1.1 ")) return false;

  int code = status.substring(9).toInt(); // "200 OK" → 200

  // 헤더 스킵
  while (true) {
    String line = c.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
    if (millis() - t0 > 4000) break;
  }
  // 바디 스니핑(디버그용)
  String bodyResp;
  t0 = millis();
  while ((millis() - t0 < 1000) && (c.connected() || c.available())) {
    while (c.available()) {
      char ch = c.read();
      if (bodyResp.length() < 256) bodyResp += ch;
    }
    delay(1);
  }
  if (bodyResp.length()) LOGI("[RESP] %s", bodyResp.c_str());

  return (code >= 200 && code < 300);
}

bool rtdb_patch_single_with_fallback(const SampleItem& it) {
  if (!wifiReady()) return false;

  // Primary 3회 시도
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (send_rtdb_patch_once(HOST_PRIMARY, PATH_BASE, it)) return true;
    LOGW("[HTTP] primary fail, retry=%d", attempt + 1);
    delay(150);
  }
  // Backup 2회 시도
  LOGW("[HTTP] try backup host…");
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (send_rtdb_patch_once(HOST_BACKUP, PATH_BASE, it)) return true;
    LOGW("[HTTP] backup fail, retry=%d", attempt + 1);
    delay(150);
  }
  return false;
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
  data.pm10  = (buffer[14] << 8) | buffer[15];
  return true;
}

/************** 키/JSON 생성(A1 유지 + 안전키 옵션) **************/
// 시간 미동기화 시 1970 키 방지: u+millis 대체키 사용을 원치 않으면 false로
#define USE_SAFE_FALLBACK_KEY true

String makeKey() {
  if (timeReady()) return nowDateTimeKey();
#if USE_SAFE_FALLBACK_KEY
  char b[24]; snprintf(b, sizeof(b), "u%010lu", millis());
  return String(b);
#else
  return nowDateTimeKey(); // 1970 키 허용
#endif
}

String makeSampleValueJSON(float mq9_v, float voc_v, const PMSData& pms, bool hasPMS) {
  String v = "{";
  if (timeReady()) {
    v += "\"timestamp\":\"" + nowIso8601KST() + "\"";
  } else {
    v += "\"timestamp\":null,\"tsMillis\":" + String(millis());
  }
  v += ",\"serverTime\":{\".sv\":\"timestamp\"}"; // 서버 기준 시각도 기록
  v += ",\"MQ9_V\":" + String(mq9_v, 3);
  v += ",\"VOC_V\":" + String(voc_v, 3);
  if (hasPMS) {
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
    while (!wifiReady()) { vTaskDelay(pdMS_TO_TICKS(200)); }
    vTaskDelay(pdMS_TO_TICKS(WARMUP_MS));
    last = xTaskGetTickCount();

    while (wifiReady()) {
      const float ADC_REF = 3.3f;

      uint32_t acc1 = 0;
      for (int i = 0; i < 4; i++) { acc1 += analogRead(PIN_MQ9_AO); delay(1); }
      float mq9_v = (acc1 / 4.0f) * (ADC_REF / 4095.0f);

      uint32_t acc2 = 0;
      for (int i = 0; i < 4; i++) { acc2 += analogRead(PIN_VOC_AO); delay(1); }
      float voc_v = (acc2 / 4.0f) * (ADC_REF / 4095.0f);

      lastMQ9_V = mq9_v;
      lastVOC_V = voc_v;
      haveSample = true;

      PMSData tmp;
      if (readPMS(tmp)) { lastPMS = tmp; havePMS = true; }

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
    if (wifiReady() && haveSample) {
      SampleItem it;
      it.key = makeKey(); // A1: 시간키 유지(+미동기화 시 안전키)
      PMSData tmpPMS = lastPMS;
      it.value = makeSampleValueJSON(lastMQ9_V, lastVOC_V, tmpPMS, havePMS);
      bool ok = rtdb_patch_single_with_fallback(it);
      if (ok) LOGI("[UPLOAD] ok"); else LOGW("[UPLOAD] fail");
    }
    vTaskDelayUntil(&last, period);
  }
}

/************** Wi-Fi 스캔 + BT 전송 **************/
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

      SerialBT.printf("WIFI:%d:%s:%d:%s\n",
                      i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), encType.c_str());
      delay(10);
    }
  }

  SerialBT.println("WIFI_LIST_END");
  LOGI("[SCAN] WiFi list sent");
}

/************** 명령 처리 **************/
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  LOGI("[CMD] Received: %s", cmd.c_str());

  if (cmd == "SCAN") {
    doWiFiScanAndSend();
  } else if (cmd.startsWith("CONNECT:")) {
    String data = cmd.substring(8);
    int sepIndex = data.indexOf('|');

    if (sepIndex > 0) {
      ssid_ = data.substring(0, sepIndex);
      pass_ = data.substring(sepIndex + 1);
      LOGI("[CMD] Connect request - SSID: %s, Pass length: %d", ssid_.c_str(), pass_.length());
      state_ = TRY_CONNECT;
    } else {
      SerialBT.println("error:invalid connect format");
      LOGE("[CMD] Invalid CONNECT format");
    }
  } else if (cmd == "CLEAR_WIFI") {
    clearWiFi(); SerialBT.println("ok:cleared");
  } else if (cmd == "REPROVISION") {
    WiFi.disconnect(true);
    clearWiFi();
    state_ = WAIT_COMMAND;
    startClassicBT();
    SerialBT.println("ok:reprovision");
    LOGI("[CMD] Reprovision mode activated");
  } else {
    SerialBT.println("error:unknown command");
    LOGW("[CMD] Unknown command: %s", cmd.c_str());
  }
}

/************** 부트 버튼 Wi-Fi 초기화 **************/
void checkBootReset() {
  if (digitalRead(BOOT_PIN) == LOW) {
    unsigned long pressedAt = millis();
    while (digitalRead(BOOT_PIN) == LOW) {
      if (millis() - pressedAt > 3000) {
        Serial.println("WiFi Reset Triggered!");
        WiFi.disconnect(true, true);
        clearWiFi();
        delay(500);
        ESP.restart();
      }
      delay(10);
    }
  }
}

/************** setup / loop **************/
void setup() {
  Serial.begin(115200);
  delay(200);
  LOGI("[BOOT] setup() start, heap=%u", ESP.getFreeHeap());

  // BLE 메모리 해제(클래식 BT만 사용)
  esp_bt_mem_release(ESP_BT_MODE_BLE);

  // 센서/타이밍 초기화
  initTimeKST();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ9_AO, ADC_11db);
  analogSetPinAttenuation(PIN_VOC_AO, ADC_11db);

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);

  pinMode(BOOT_PIN, INPUT_PULLUP);

  // 저장된 Wi-Fi 자동 연결
  {
    String s, p;
    if (loadSavedWiFi(s, p)) {
      LOGI("[NVS] Saved WiFi found: %s", s.c_str());
      if (tryWiFiConnectOnce(s, p, 10000)) {
        String ip = WiFi.localIP().toString();
        LOGI("[WiFi] Auto-connected! IP=%s", ip.c_str());
        state_ = ONLINE; // NTP 대기 없이 온라인
      } else {
        LOGW("[WiFi] Auto-connect failed → start BT provisioning");
        startClassicBT();
        state_ = WAIT_COMMAND;
      }
    } else {
      LOGI("[NVS] No saved WiFi → start BT provisioning");
      startClassicBT();
      state_ = WAIT_COMMAND;
    }
  }

  // 백그라운드 태스크
  xTaskCreatePinnedToCore(taskSample1Hz, "Sample1Hz", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskUpload1Hz, "Upload1Hz", 6144, nullptr, 1, nullptr, 1);

  LOGI("[TASK] Sample+Upload spawned");
  LOGI("[BOOT] setup() done");
}

void loop() {
  checkBootReset();

  // BT 명령 수신
  if (btEnabled && state_ != TRY_CONNECT) {
    while (SerialBT.available()) {
      char c = (char)SerialBT.read();
      if (c == '\r') c = '\n';
      if (c == '\n') { handleCommand(rxBuf_); rxBuf_.clear(); }
      else {
        rxBuf_ += c;
        if (rxBuf_.length() > 200) rxBuf_.clear();
      }
      rxLastMs_ = millis();
      yield();
    }
    if (rxBuf_.length() > 0 && (millis() - rxLastMs_ > 1200)) {
      handleCommand(rxBuf_); rxBuf_.clear();
    }
  }

  // CONNECT 명령 수신 후 1회 연결
  static bool tried = false;
  if (state_ == TRY_CONNECT && !tried) {
    tried = true;
    LOGI("[WiFi] Connecting to %s...", ssid_.c_str());
    if (btEnabled) SerialBT.printf("Connecting to %s...\n", ssid_.c_str());

    bool ok = tryWiFiConnectOnce(ssid_, pass_, 15000);

    if (ok) {
      saveWiFi(ssid_, pass_);
      String ip = WiFi.localIP().toString();
      LOGI("[WiFi] Connected! IP=%s", ip.c_str());
      if (btEnabled) { SerialBT.printf("ip:%s\n", ip.c_str()); SerialBT.println("ok:connected"); delay(300); }
      state_ = ONLINE;
      if (btEnabled) { stopClassicBT(); }
      LOGI("[NET] Online ready → sampling will start after warmup");
    } else {
      LOGW("[WiFi] Connection failed");
      if (btEnabled) SerialBT.println("error:connection failed");
      state_ = WAIT_COMMAND;
      tried = false;
    }
  }

  // 온라인 모니터링: 5초 연속 Wi-Fi 다운 시 BT 복귀
  static uint32_t lastChk = 0, offlineSince = 0;
  if (millis() - lastChk > 1000) {
    lastChk = millis();
    if (state_ == ONLINE) {
      if (!wifiReady()) {
        if (offlineSince == 0) offlineSince = millis();
        if (millis() - offlineSince > 5000) {
          LOGW("[NET] WiFi down(5s) → fallback to BT");
          state_ = WAIT_COMMAND;
          tried = false;
          if (!btEnabled) startClassicBT();
        }
      } else {
        offlineSince = 0;
      }
    } else {
      offlineSince = 0;
    }
  }

  delay(2);
}