#pragma once
#include <math.h>   // NAN (lvglShowStopped 기본값)
#include "Theme.h"

// 화면은 처음 호출할 때 한 번만 만들고, 이후 호출은 값만 갱신한다
// (매번 새로 만들면 메모리 누수·깜빡임이 생긴다). 그래서 매 프레임 불러도 안전하다.

// 메인·설정 화면
void lvglShowReady(const InjectionConfig& cfg, SystemState status, uint8_t batPct);
void lvglShowSetVolume(uint8_t volumeMl, uint8_t batPct);
void lvglShowSetTime(uint16_t timeSec, uint8_t batPct, uint16_t minSec);   // minSec: 표시할 최소 시간
void lvglShowSetNeedle(uint8_t needleG, uint8_t batPct);

// 진행 화면 — Frame은 진입 때 1회, Values는 갱신 주기마다
void lvglShowHoming(uint8_t batPct);
void lvglShowAligningFrame(uint8_t batPct, uint8_t volumeMl, uint16_t timeSec);
void lvglUpdateAligningValues(float progress, uint8_t batPct);
void lvglShowRunningFrame(uint8_t batPct, uint8_t volumeMl);
void lvglUpdateRunningValues(float progress, float elapsedSec, uint8_t batPct);
void lvglSetRunningPaused(bool paused);   // 일시정지 안내로 문구만 바꾼다

// 결과 화면
// elapsedSec: 실제 주입 시간(일시정지 제외)
void lvglShowFinished(uint8_t batPct, uint8_t volumeMl, float elapsedSec, uint8_t needleG);
// injectedMl·elapsedSec: 주입 도중 멈췄을 때만 값, 아니면 NAN (주입량 목록 숨김)
void lvglShowStopped(uint8_t batPct, StopReason reason, float injectedMl = NAN, uint8_t volumeMl = 0, float elapsedSec = NAN);

// 시스템 설정 메뉴 — 선택만 바뀔 때는 lvglUpdateSysMenu
void lvglShowSysMenu(const InjectionConfig& cfg, uint8_t sel, uint8_t batPct);
void lvglUpdateSysMenu(const InjectionConfig& cfg, uint8_t sel, uint8_t batPct);

// 니들 선택 표시 (시스템 설정 "Needle"). 끄면 설정은 용량 → 시간 2단계, 메인·완료 화면에서 니들 행이 빠진다.
void lvglSetNeedleEnabled(bool on);
bool lvglNeedleEnabled();

// 무선 업로드 화면. 업로드 중에는 loop()가 돌지 않으므로 Ota.cpp의 진행 콜백이
// lvglUpdateOta()를 직접 부른다(상태는 Ota 모듈에서 읽는다).
void lvglShowOtaFrame(uint8_t batPct);
void lvglUpdateOta();

// 수동 전진 조그. originValid=false면 호밍 전이라 물리 리밋만 보호 중임을 표시한다.
void lvglShowJogFrame(uint8_t batPct);
void lvglUpdateJogValues(float posMm, bool originValid, uint8_t batPct);

// 배터리 위험 경고창 — 어떤 화면 위에도 겹쳐 표시
void lvglSetBatteryWarning(bool show);

// 화면 맨 위 전압·전류 표시 (설정 메뉴 "Power Info")
void lvglSetBatteryDebug(bool show);
void lvglUpdateBatteryDebug(float vBus, float iBat, uint8_t pct, int calErr);
void lvglUpdateBatteryDebugNoSensor();   // INA228 미검출 시 빨간 경고
bool lvglBatteryDebugShown();

// 부팅 초기 LVGL 기본 흰 화면이 보이지 않게 하는 빈 화면 (lvglThemeInit 직후)
void lvglShowBlank();

// LVGL 힙 사용률 로그 (화면을 새로 만들 때마다 자동 호출)
void lvglLogMemUsage(const char* tag);
// 상단 WiFi 연결 아이콘 (모든 화면 공통)
void lvglSetWifiIndicator(bool connected);
