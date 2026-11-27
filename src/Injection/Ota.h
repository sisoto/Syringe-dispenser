#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════
// Ota.h — WiFi 무선 펌웨어 업데이트
//
// 첫 업로드는 USB로 하고, 이후에는 Arduino IDE의 네트워크 포트로 올린다.
// 설정 메뉴 "Wi-Fi"가 켜져 있으면 부팅 때 접속해 연결을 유지한다(평균 약 80mA).
// 업로드는 모터가 멈춘 화면에서만 받고(Injection.ino의 wifiIdleState),
// 시작 순간 서보를 세우고 업로드 화면으로 바꾼다.
// 송신 피크 전류를 줄이려고 출력을 11dBm으로 낮춰 둔다.
// ══════════════════════════════════════════════════════════

enum OtaState : uint8_t {
  OTA_OFF,          // 라디오 꺼짐
  OTA_CONNECTING,   // 공유기 접속 중
  OTA_READY,        // 접속됨 — 업로드 대기
  OTA_UPDATING,     // 업로드 진행 중
  OTA_FAILED,       // 접속 실패 / 업로드 실패
};

void        otaBegin();      // 라디오 켜고 접속 시작
void        otaStop();       // 라디오 끄기 (전력 절약)
void        otaLoop();       // OTA 모드일 때 매 프레임 호출
OtaState    otaStatus();
uint8_t     otaProgressPct();
const char* otaIpString();   // 접속 후 IP, 그 전에는 "-"
const char* otaMessage();    // 화면에 띄울 짧은 안내
bool        otaTimedOut();   // 대기 시간 초과 → READY로 복귀
bool        otaConnected();  // 라디오가 켜져 있고 공유기에 실제로 연결됨(상단 아이콘용)
bool        otaConfigured();   // Secrets.h에 WIFI_SSID가 채워져 있는가
// 업로드가 시작되는 순간(ArduinoOTA가 블로킹 수신에 들어가기 직전) 한 번 불린다.
// Injection.ino가 여기서 업로드 화면으로 바꿔, 업로드 내내 그 화면이 보이게 한다.
void        otaSetStartCallback(void (*cb)());
