#pragma once
#include "Theme.h"
// 시리얼/Telnet 진단: 부팅 배너, 루프 지연·전압 처짐 통계, 상태 전환 로그
constexpr float DIAG_VSAG_WARN_V = 3.30f;          // 부하 중 이 전압 미만이면 경고
constexpr unsigned long DIAG_LOOP_WARN_MS = 100UL; // loop() 한 바퀴 지연 경고
constexpr float DIAG_PATH_R_WARN_OHM = 0.30f;      // 추정 배터리 경로 저항 경고
constexpr unsigned long DIAG_REPORT_BUSY_MS = 2000UL;
constexpr unsigned long DIAG_REPORT_IDLE_MS = 15000UL;
void diagBegin();
void diagLoopTick();
void diagSample(float vBus, float iMa, bool motorActive);
void diagReport(SystemState st, bool motorActive);
void diagState(SystemState from, SystemState to, StopReason reason);
const char* stateName(SystemState s);
const char* stopReasonName(StopReason r);
