#include "Ota.h"
#include "Secrets.h"
#include "TelnetLog.h"
#include "Theme.h"
#include "Motor.h"
#include "Platform.h"
#include "LvglBridge.h"
#include "LvglScreens.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// ── Serial 미러링 ─────────────────────────────────────────
// 이 아래의 Serial.print를 USB+Telnet 양쪽으로 보낸다.
// 반드시 모든 #include 뒤에 둘 것 — 위에 두면 라이브러리 헤더 내부의
// Serial까지 LOG로 치환되어 부팅이 깨진다(ArduinoOTA/WiFi 등).
#define Serial LOG

static OtaState      _state      = OTA_OFF;
static uint8_t       _pct        = 0;
static char          _ip[20]     = "-";
static char          _msg[48]    = "";
static unsigned long _startedMs  = 0;   // 접속 시도 시작
static unsigned long _idleSinceMs = 0;  // 업로드 없이 대기한 시작 시각
static bool          _otaBegun   = false;

OtaState    otaStatus()      { return _state; }
uint8_t     otaProgressPct() { return _pct; }
const char* otaIpString()    { return _ip; }
const char* otaMessage()     { return _msg; }

// ── 업로드 중 콜백 ────────────────────────────────────────
// ArduinoOTA.handle()은 업로드가 시작되면 끝날 때까지 그 안에서 블로킹된다.
// 그동안 loop()가 돌지 않으므로 워치독이 물고, 화면도 멈춘다.
// 그래서 진행 콜백에서 직접 워치독을 먹이고 LVGL도 한 번씩 돌려준다.
static void onProgress(unsigned int done, unsigned int total) {
  _pct = total ? (uint8_t)((uint64_t)done * 100 / total) : 0;
  wdtFeed();
  lvglUpdateOta();      // 진행률 라벨/바 갱신
  lvglBridgeTick();     // 실제 화면에 반영
}

static void (*_startCb)() = nullptr;
void otaSetStartCallback(void (*cb)()) { _startCb = cb; }

static void onStart() {
  motor.stop(); // 플래시 쓰기 전 서보 정지(속도 0), 토크는 유지.
  _state = OTA_UPDATING;
  _pct   = 0;
  snprintf(_msg, sizeof(_msg), "Uploading firmware...");
  Serial.println("[OTA] update start");
  // 이 함수가 끝나면 ArduinoOTA가 업로드가 끝날 때까지 블로킹한다(loop 정지).
  // 그 전에 화면을 업로드 화면으로 바꿔 둬야 이전 화면이 멈춘 채 남지 않는다.
  if (_startCb) _startCb();
}

static void onEnd() {
  snprintf(_msg, sizeof(_msg), "Done - rebooting");
  Serial.println("[OTA] update finished, rebooting");
  lvglUpdateOta();
  lvglBridgeTick();
}

static void onError(ota_error_t err) {
  _state = OTA_FAILED;
  const char* e = "unknown";
  switch (err) {
    case OTA_AUTH_ERROR:    e = "auth failed";    break;
    case OTA_BEGIN_ERROR:   e = "begin failed";   break;
    case OTA_CONNECT_ERROR: e = "connect failed"; break;
    case OTA_RECEIVE_ERROR: e = "receive failed"; break;
    case OTA_END_ERROR:     e = "end failed";     break;
    default: break;
  }
  snprintf(_msg, sizeof(_msg), "Failed: %s", e);
  Serial.print("[OTA] error: ");
  Serial.println(e);
}

// ══════════════════════════════════════════════════════════
void otaBegin() {
  if (_state != OTA_OFF) return;

  _pct = 0;
  strcpy(_ip, "-");
  // Secrets.h에 SSID를 채우지 않았으면(배포판 기본) 라디오를 켜지 않는다
  if (WIFI_SSID[0] == '\0') {
    snprintf(_msg, sizeof(_msg), "Wi-Fi not set up (Secrets.h)");
    _state = OTA_FAILED;
    Serial.println("[OTA] WIFI_SSID is empty - fill in Secrets.h to use Wi-Fi");
    return;
  }
  snprintf(_msg, sizeof(_msg), "Connecting to WiFi...");
  _state     = OTA_CONNECTING;
  _startedMs = millis();

  WiFi.persistent(false);       // 접속할 때마다 flash에 쓰지 않게
  WiFi.mode(WIFI_STA);
  // 송신 출력을 낮춰 피크 전류를 줄인다. 전원 경로 저항이 큰 상태라
  // 최대 출력(약 20dBm, 피크 350mA+)은 전압 강하 위험이 있음.
  // 공유기가 멀어서 접속이 안 되면 WIFI_POWER_17dBm 정도로 올릴 것.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(true);          // 대기 중 평균 소비 감소
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[OTA] connecting to ");
  Serial.println(WIFI_SSID);
}

void otaStop() {
  if (_state == OTA_OFF) return;
  if (_otaBegun) { ArduinoOTA.end(); _otaBegun = false; }
  MDNS.end();
  telnetLogStop();               // Telnet 서버 먼저 닫기
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  _state = OTA_OFF;
  _pct   = 0;
  strcpy(_ip, "-");
  _msg[0] = '\0';
  Serial.println("[OTA] radio off");
}

bool otaConfigured() { return WIFI_SSID[0] != '\0'; }

bool otaConnected() { return _state != OTA_OFF && WiFi.status() == WL_CONNECTED; }

bool otaTimedOut() {
  if (_state == OTA_CONNECTING)
    return (millis() - _startedMs) > OTA_CONNECT_TIMEOUT_MS;
  if (_state == OTA_READY)
    return (millis() - _idleSinceMs) > OTA_IDLE_TIMEOUT_MS;
  return false;   // 업로드 중이거나 실패 표시 중에는 자동 종료하지 않음
}

void otaLoop() {
  switch (_state) {
    case OTA_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        IPAddress a = WiFi.localIP();
        snprintf(_ip, sizeof(_ip), "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);

        ArduinoOTA.setHostname(OTA_HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.onStart(onStart);
        ArduinoOTA.onEnd(onEnd);
        ArduinoOTA.onProgress(onProgress);
        ArduinoOTA.onError(onError);
        ArduinoOTA.begin();
        _otaBegun = true;

        // Arduino IDE의 네트워크 포트 목록에 이름으로 뜨게 함
        MDNS.begin(OTA_HOSTNAME);

        _state       = OTA_READY;
        _idleSinceMs = millis();
        snprintf(_msg, sizeof(_msg), "Ready for upload");
        Serial.print("[OTA] connected, IP ");
        Serial.print(_ip);
        Serial.print("  RSSI ");
        Serial.print(WiFi.RSSI());
        Serial.print("dBm  host ");
        Serial.println(OTA_HOSTNAME);
        // WiFi 연결 완료 — Telnet 로그 서버 시작
        telnetLogBegin();
      } else if (otaTimedOut()) {
        _state = OTA_FAILED;
        snprintf(_msg, sizeof(_msg), "WiFi connect failed");
        Serial.println("[OTA] connect timeout");
      }
      break;

    case OTA_READY:
    case OTA_UPDATING:
      ArduinoOTA.handle();
      // 연결이 끊기면 다시 접속 단계로
      if (_state == OTA_READY && WiFi.status() != WL_CONNECTED) {
        _state     = OTA_CONNECTING;
        _startedMs = millis();
        snprintf(_msg, sizeof(_msg), "Reconnecting...");
      }
      break;

    default:
      break;
  }
}
