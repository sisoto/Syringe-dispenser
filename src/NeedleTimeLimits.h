#pragma once
#include "Theme.h"

// 바늘별 최소 주입 시간(초). 최소값이 10초 눈금 밖이어도(예: 75초) 쓸 수 있다 —
// 그 경우 다이얼은 01:15 → 01:20 → 01:30 …처럼 10초 눈금으로 이어진다.
static constexpr uint16_t NEEDLE_MIN_TIME_SEC[] = {
   60,  // 21G  1:00
   60,  // 22G  1:00
   80,  // 23G  1:20
  100,  // 24G  1:40
  120   // 25G  2:00
};

static_assert(CFG_NEEDLE_MIN_G == 21 && CFG_NEEDLE_MAX_G == 25,
              "Update needle time table for the configured gauge range");
static_assert(CFG_TIME_STEP_SEC > 0 && CFG_TIME_MAX_SEC >= 120,
              "Invalid time configuration");

inline uint16_t needleMinTimeSec(uint8_t gauge) {
  uint32_t seconds = (gauge >= 21 && gauge <= 25)
      ? NEEDLE_MIN_TIME_SEC[gauge - 21] : CFG_TIME_MAX_SEC;
  if (seconds < CFG_TIME_MIN_SEC) seconds = CFG_TIME_MIN_SEC;
  return seconds > CFG_TIME_MAX_SEC ? CFG_TIME_MAX_SEC : seconds;
}

static_assert(NO_NEEDLE_MIN_TIME_SEC >= CFG_TIME_MIN_SEC && NO_NEEDLE_MIN_TIME_SEC <= CFG_TIME_MAX_SEC &&
              NO_NEEDLE_MIN_TIME_SEC % CFG_TIME_STEP_SEC == 0, "NO_NEEDLE_MIN_TIME_SEC out of range");

// 최소 주입 시간: 니들 선택이 켜져 있으면 바늘별, 꺼져 있으면 NO_NEEDLE_MIN_TIME_SEC
inline uint16_t minInjectTimeSec(uint8_t gauge, bool needleOn) {
  return needleOn ? needleMinTimeSec(gauge) : NO_NEEDLE_MIN_TIME_SEC;
}

// 시간 다이얼 한 칸: 10초 눈금으로 이동하되, 최소값(눈금 밖일 수 있음)에서 멈춘다.
//  올림: 75 → 80 → 90 …   내림: 90 → 80 → 75(최소)
inline uint16_t timeStepUp(uint16_t t) {
  uint32_t n = ((uint32_t)t / CFG_TIME_STEP_SEC + 1) * CFG_TIME_STEP_SEC;
  return n > CFG_TIME_MAX_SEC ? CFG_TIME_MAX_SEC : (uint16_t)n;
}
inline uint16_t timeStepDown(uint16_t t, uint16_t minimum) {
  if (t <= minimum) return minimum;
  uint16_t n = (uint16_t)(((t - 1) / CFG_TIME_STEP_SEC) * CFG_TIME_STEP_SEC);
  return n < minimum ? minimum : n;
}

// 시간을 최소값 이상·10초 눈금으로 맞춘다. 바뀌었으면 true.
inline bool enforceMinTime(InjectionConfig& cfg, uint16_t minimum) {
  uint16_t seconds = cfg.timeSec;
  if (seconds < minimum) seconds = minimum;
  // 눈금 밖 값은 최소값일 때만 허용. 바늘을 바꿔 최소값이 달라지면
  // (예: 22G 01:15 → 21G) 다음 10초 눈금(01:20)으로 올린다 — 저장값 검증과 일치.
  if (seconds % CFG_TIME_STEP_SEC != 0 && seconds != minimum)
    seconds = ((seconds + CFG_TIME_STEP_SEC - 1) / CFG_TIME_STEP_SEC) * CFG_TIME_STEP_SEC;
  if (seconds > CFG_TIME_MAX_SEC) seconds = CFG_TIME_MAX_SEC;
  const bool changed = seconds != cfg.timeSec;
  cfg.timeSec = seconds;
  return changed;
}
