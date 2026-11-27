#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════
// Platform.h — ESP32-S3 하드웨어 종속 기능 (Arduino-ESP32 3.3.x 전용)
// ══════════════════════════════════════════════════════════

// ── 빠른 GPIO 읽기 (ISR 안전) ──
// digitalRead()는 플래시에 있어 ISR에서 부르면 위험하므로 레지스터를 직접 읽는다.
bool     fastGpioRead(uint8_t pin);
uint32_t fastGpioIn0_31();   // 0~31번 핀을 한 번에 (EC11 A/B 동시 샘플링용)

// ── PWM (LEDC, 핀 기반 API) ──
constexpr uint8_t PWM_RES_BITS = 8;   // 0~255
void pwmAttach(uint8_t pin, uint32_t freqHz, uint8_t resBits = PWM_RES_BITS);
void pwmWrite(uint8_t pin, uint32_t duty);

// ── 태스크 워치독 (loop() 태스크 등록) ──
void wdtBegin(uint32_t timeoutMs);
void wdtFeed();

// ── 리셋 사유 ──
bool        resetWasWatchdog();
bool        resetWasBrownout();
bool        resetWasPanic();
const char* resetReasonStr();

// ── 내부 SRAM / PSRAM 여유 로그 ──
void logHeapInfo(const char* tag);
