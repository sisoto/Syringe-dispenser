/*
 * Injection.ino — 주사기 디스펜서
 * 보드      : ESP32-S3 DevKitC-1 N16R8 / Arduino-ESP32 3.3.x
 * 라이브러리: LovyanGFX, LVGL 8.3.11, INA228 (Rob Tillaart 0.4.1)
 *             서보 통신은 자체 구현(StsProtocol.h) — Feetech 라이브러리 불필요
 * 구동부    : Feetech STS3250 (9V) + Waveshare Bus Servo Adapter (A), UART1 TX9/RX10
 * 전류 센서 : INA228, 션트 R015 (0.015Ω), 배터리 전체 전류
 */

#include "Platform.h"
#include "Diag.h"
#include "Ota.h"
#include "TelnetLog.h"

#include <math.h>
#include <Wire.h>
#include <Preferences.h>   // NVS
#include <INA228.h>
#include "Theme.h"
#include "NeedleTimeLimits.h"
#include "Display.h"
#include "Motor.h"
#include "Ui.h"
#include "LvglBridge.h"
#include "LvglTheme.h"
#include "LvglScreens.h"

// ── Serial 미러링 ─────────────────────────────────────────
// 이 아래의 Serial.print를 USB+Telnet 양쪽으로 보낸다.
// 반드시 모든 #include 뒤에 둘 것 — 위에 두면 라이브러리 헤더 내부의
// Serial까지 LOG로 치환되어 부팅이 깨진다(ArduinoOTA/WiFi 등).
#define Serial LOG

// ══════════════════════════════════════════════════════════
// 설정값 저장 (NVS)
// ══════════════════════════════════════════════════════════
static Preferences prefs;
#define NVS_NAMESPACE "dispenser"
#define NVS_KEY_CFG   "cfg"
#define NVS_KEY_WIFI  "wifi"   // WiFi 상시 연결 On/Off
#define NVS_KEY_PWR   "pwr"    // 화면 전압·전류 표시 On/Off (설정 메뉴 "Power Info")
#define NVS_KEY_NEEDLE "needle" // 니들 선택 On/Off (설정 메뉴 "Needle")
#define NVS_KEY_FRONT "frontmm" // 학습한 주입 끝(전방 리밋이 눌리는) 위치(mm)

// ── 주입 끝 위치 자동 학습 ──
// 목표가 실제 리밋 위치와 다르면 도착 시간이 어긋난다(멀면 일찍 끝나고, 짧으면
// 리밋까지 더 미느라 늦어짐). 리밋이 눌린 위치를 기억해 다음 주입의 목표로 쓴다.
static float frontEndMm = SYRINGE_EMPTY_MM;
static void loadFrontEnd() {
  prefs.begin(NVS_NAMESPACE, true);
  const float v = prefs.getFloat(NVS_KEY_FRONT, NAN);
  prefs.end();
  if (isfinite(v) && fabsf(v - SYRINGE_EMPTY_MM) <= FRONT_LEARN_RANGE_MM) frontEndMm = v;
  Serial.printf("[CAL] dispense end %.2fmm (%s)\n", frontEndMm, isfinite(v) ? "learned" : "default");
}
static void learnFrontEnd(float pressedMm);
// 완료 범위보다 일찍 전방 리밋이 눌려 멈춘 경우: 끝 위치가 바뀐 것(부품 교체·스위치
// 위치 변경 등)이므로 학습해 다음 주입부터 맞춘다.
static void noteEarlyFrontLimit() {
  const float pos = motor.getPositionMm();
  Serial.printf("[RUN] front limit %.2fmm before target (pos=%.2f, target=%.2f) — learning new end\n",
                frontEndMm - pos, pos, frontEndMm);
  learnFrontEnd(pos);
}
static void learnFrontEnd(float pressedMm) {
  if (!isfinite(pressedMm)) return;
  if (fabsf(pressedMm - SYRINGE_EMPTY_MM) > FRONT_LEARN_RANGE_MM) {
    Serial.printf("[CAL] front pressed at %.2fmm — out of range, not learned\n", pressedMm);
    return;
  }
  if (fabsf(pressedMm - frontEndMm) < 0.05f) return;   // 사실상 같으면 플래시 쓰기 생략
  Serial.printf("[CAL] dispense end %.2f -> %.2fmm\n", frontEndMm, pressedMm);
  frontEndMm = pressedMm;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat(NVS_KEY_FRONT, frontEndMm);
  prefs.end();
}

// ── WiFi 상시 연결 설정 (시스템 설정 메뉴에서 토글) ──
static bool          wifiEnabled = WIFI_DEFAULT_ON;
static unsigned long wifiFailedSinceMs = 0;
static void loadWifiSetting() {
  prefs.begin(NVS_NAMESPACE, true);
  wifiEnabled = prefs.getBool(NVS_KEY_WIFI, WIFI_DEFAULT_ON);
  prefs.end();
}
// ── 화면 맨 위 전압·전류 표시 (시스템 설정 메뉴 "Power Info") ──
static bool showPowerInfo = POWER_INFO_DEFAULT_ON;
static void loadPowerInfoSetting() {
  prefs.begin(NVS_NAMESPACE, true);
  showPowerInfo = prefs.getBool(NVS_KEY_PWR, POWER_INFO_DEFAULT_ON);
  prefs.end();
}
static void savePowerInfoSetting() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool(NVS_KEY_PWR, showPowerInfo);
  prefs.end();
}
// ── 니들 선택 (시스템 설정 메뉴 "Needle") ──
static bool needleSelectOn = NEEDLE_SELECT_DEFAULT_ON;
static void loadNeedleSetting() {
  prefs.begin(NVS_NAMESPACE, true);
  needleSelectOn = prefs.getBool(NVS_KEY_NEEDLE, NEEDLE_SELECT_DEFAULT_ON);
  prefs.end();
}
static void saveNeedleSetting() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool(NVS_KEY_NEEDLE, needleSelectOn);
  prefs.end();
}
// 지금 설정에서의 최소 주입 시간과, 시간을 그 이상으로 맞추기
static uint16_t minTimeSec() { return minInjectTimeSec(ui.config.needleG, needleSelectOn); }
static bool enforceTime() { return enforceMinTime(ui.config, minTimeSec()); }

static void saveWifiSetting() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool(NVS_KEY_WIFI, wifiEnabled);
  prefs.end();
}
// InjectionConfig 레이아웃을 바꾸면 반드시 올린다(예전 레코드를 새 구조로 읽지 않게).
#define EEPROM_MAGIC  0xA6

struct SavedConfig {
  uint8_t          magic;
  InjectionConfig  cfg;
};

static void saveConfig() {
  enforceTime();
  SavedConfig sc;
  sc.magic = EEPROM_MAGIC;
  sc.cfg   = ui.config;
  prefs.begin(NVS_NAMESPACE, false);            // 쓰기 모드
  prefs.putBytes(NVS_KEY_CFG, &sc, sizeof(sc));
  prefs.end();
}

// 복원된 설정값 검사. magic이 맞아도 값이 어긋날 수 있다(플래시 손상 등).
// 하나라도 이상하면 통째로 기본값으로 되돌린다.
static bool isConfigValid(const InjectionConfig& c) {
  if (c.volumeMl < CFG_VOL_MIN_ML || c.volumeMl > CFG_VOL_MAX_ML) return false;
  if (c.volumeMl % CFG_VOL_STEP_ML != 0)                          return false;
  if (c.timeSec  < CFG_TIME_MIN_SEC || c.timeSec > CFG_TIME_MAX_SEC) return false;
  if (c.needleG < CFG_NEEDLE_MIN_G || c.needleG > CFG_NEEDLE_MAX_G) return false;
  // 10초 눈금이거나 그 바늘의 최소값이어야 유효
  if (c.timeSec % CFG_TIME_STEP_SEC != 0 && c.timeSec != needleMinTimeSec(c.needleG)) return false;
  if (c.brightPct < SYS_BRIGHT_MIN_PCT || c.brightPct > SYS_BRIGHT_MAX_PCT) return false;
  {
    bool dimOk = false;
    for (uint8_t i = 0; i < SYS_DIM_CHOICE_COUNT; i++)
      if (c.dimSec == SYS_DIM_CHOICES[i]) { dimOk = true; break; }
    if (!dimOk) return false;
  }
  // 정렬 위치 표에 있는 용량이어야 한다
  bool found = false;
  for (uint8_t i = 0; i < SYRINGE_SPEC_COUNT; i++)
    if (syringeSpecs[i].volumeMl == c.volumeMl) { found = true; break; }
  return found;
}

// magic 불일치(처음 부팅)면 기본값 유지
static void loadConfig() {
  SavedConfig sc;
  prefs.begin(NVS_NAMESPACE, true);             // 읽기 전용
  size_t got = prefs.getBytes(NVS_KEY_CFG, &sc, sizeof(sc));
  prefs.end();
  if (got != sizeof(sc) || sc.magic != EEPROM_MAGIC) {
    Serial.println("[CFG] no saved config, using defaults");
    return;
  }
  // 최대 용량보다 큰 값(예전 펌웨어 설정)은 최대 용량으로 옮기고 나머지는 살린다.
  bool migrated = false;
  if (sc.cfg.volumeMl > CFG_VOL_MAX_ML) {
    sc.cfg.volumeMl = CFG_VOL_MAX_ML;
    migrated = true;
    Serial.println("[CFG] saved volume above max -> set to max volume");
  }
  if (!isConfigValid(sc.cfg)) {
    Serial.print("[CFG] saved config out of range (vol=");
    Serial.print(sc.cfg.volumeMl);
    Serial.print(" time=");
    Serial.print(sc.cfg.timeSec);
    Serial.print(" needle=");
    Serial.print(sc.cfg.needleG);
    Serial.println(") -> reverting to defaults");
    return;   // ui.config는 InjectionConfig 기본값 그대로 유지
  }
  ui.config = sc.cfg;
  if (migrated) saveConfig();
  if (enforceTime()) {
    saveConfig();
    Serial.println("[CFG] raised time to minimum");
  }
  Serial.println("[CFG] loaded saved config");
}

// ══════════════════════════════════════════════════════════
// 상태
// ══════════════════════════════════════════════════════════
static StopReason stopReason = STOP_NONE;
// 설정 화면 진입 시점의 값 — CANCEL로 취소하면 되돌린다
static InjectionConfig cfgSnapshot;
// 일시정지 감속이 끝나기 전에 FWD로 재개를 누른 경우, 정지 즉시 재개
static bool resumeRequested = false;
// 주입 완료 후 아직 후진(원점 복귀)하지 않음 → RETURN LED 점등으로 안내.
// 완료 시 켜고, 원점 찾기·정렬을 시작하면 끈다(transitionTo).
static bool returnPending = false;
// 완료·정지 화면 표시용: 주입이 끝난/멈춘 순간의 경과 시간과 주입량(transitionTo에서 기록)
static float lastRunElapsedSec = 0;
static float stopInjectedMl    = NAN;   // 주입 도중 멈췄을 때만 값, 아니면 NAN
static float stopElapsedSec    = NAN;   // 주입 도중 멈췄을 때 그때까지의 주입 시간, 아니면 NAN
// 호밍을 FWD로 멈춤 → STOPPED에서 FWD를 다시 누르면 수동 전진 조그
static bool homingInterruptArmed    = false;

// ── CANCEL(KO) 인터럽트 — 플래그만 세우고 처리는 loop() 상단에서 ──
// 서보 태스크도 KO 핀을 10ms마다 직접 보고 세우므로 이 플래그는 화면·상태 처리용이다.
static volatile bool emergencyStopFlag = false;
static volatile unsigned long koLastIsrMs = 0;
static constexpr unsigned long KO_ISR_DEBOUNCE_MS = 30;

static void IRAM_ATTR koButtonISR() {
  unsigned long now = millis();
  if (now - koLastIsrMs < KO_ISR_DEBOUNCE_MS) return;  // 바운스 무시
  koLastIsrMs = now;
  if (!fastGpioRead(PIN_EC11_KO))  // LOW = 눌림 (ISR이라 레지스터를 직접 읽음)
    emergencyStopFlag = true;
}

static SystemState   state             = STATE_HOMING;
static unsigned long lastDisplayMs     = 0;
static bool          needRedraw        = true;
static float         alignTargetMm     = 0.0f;
static unsigned long finishedEnteredMs = 0;

// ══════════════════════════════════════════════════════════
// 배터리 잔량(SOC) — 모터 정지 중 전압만으로 추정
//   OCV LUT(선형 보간) → 만충 보정 → 느린 IIR 필터
// 부하 중 전압은 처지므로 표시용 %는 모터가 멈췄을 때만 갱신하고,
// 구동 중 저전압은 updateLoadGuard()가 따로 감시한다.
// ══════════════════════════════════════════════════════════
static INA228 inaBat(INA228_ADDR_BAT);
static uint8_t batPct       = 100;
static uint8_t sysMenuSel   = 0;      // 시스템 설정 메뉴에서 선택된 항목(SysMenuItem)
static bool    inaAvailable = false;
static bool    inaAlertOk   = false;  // ALERT(하드웨어 과전류) 설정·배선 확인됨
static int     inaCalError  = -99;    // 캘리브레이션 결과 (0=성공, -99=시도 전)

static constexpr float SOC_BETA_FILTER = 0.97f;   // 순간 전압 처짐이 %에 튀지 않게 강하게

// ── SOC 상태 변수 ──────────────────────────────────────────
static float socFiltered = 100.0f;  // 출력 SOC (IIR 필터 후)
static bool  socInit     = false;   // 첫 측정 완료 여부
static uint8_t socInitSkip = 0;     // 부팅 직후 불안정한 초기 샘플 건너뛰기용

// ── OCV LUT (리튬이온 1셀, 무부하 전압 기준) ──
struct BatLutEntry { float voltage; uint8_t pct; };
static const BatLutEntry BAT_LUT[] = {
  { 4.20f, 100 },
  { 4.10f,  95 },
  { 4.00f,  90 },
  { 3.93f,  80 },
  { 3.83f,  70 },
  { 3.74f,  60 },
  { 3.66f,  50 },
  { 3.59f,  40 },
  { 3.50f,  30 },
  { 3.40f,  20 },
  { 3.25f,  10 },
  { 3.00f,   0 },
};
static const uint8_t BAT_LUT_SIZE =
  sizeof(BAT_LUT) / sizeof(BAT_LUT[0]);

// OCV → SOC % (LUT 선형 보간) — 보정/정규화 전의 원시 곡선값
static float ocvToSocRaw(float v) {
  if (v >= BAT_LUT[0].voltage)              return 100.0f;
  if (v <= BAT_LUT[BAT_LUT_SIZE-1].voltage) return 0.0f;
  for (uint8_t i = 0; i < BAT_LUT_SIZE - 1; i++) {
    if (v <= BAT_LUT[i].voltage && v > BAT_LUT[i+1].voltage) {
      float ratio = (v - BAT_LUT[i+1].voltage)
                  / (BAT_LUT[i].voltage - BAT_LUT[i+1].voltage);
      return BAT_LUT[i+1].pct + ratio * (BAT_LUT[i].pct - BAT_LUT[i+1].pct);
    }
  }
  return 0.0f;
}

// OCV → SOC %. 충전 직후 전압은 4.20V에 못 미치므로 BAT_FULL_V 이상을 100%로 보고,
// BAT_FULL_BLEND_PCT 위 구간만 늘려 매핑한다(아래 곡선과 경고 임계값은 그대로).
static float ocvToSoc(float v) {
  v += BAT_SENSE_OFFSET_V;   // 측정 경로 손실 보정

  if (v >= BAT_FULL_V) return 100.0f;

  float raw = ocvToSocRaw(v);
  if (raw <= BAT_FULL_BLEND_PCT) return raw;   // 하단 구간은 그대로

  // 상단 구간 재정규화: raw가 pFull일 때 정확히 100%가 되도록 늘림
  float pFull = ocvToSocRaw(BAT_FULL_V);
  if (pFull <= BAT_FULL_BLEND_PCT) return raw; // 설정이 이상하면 원본 유지
  float t = (raw - BAT_FULL_BLEND_PCT) / (pFull - BAT_FULL_BLEND_PCT);
  if (t > 1.0f) t = 1.0f;
  return BAT_FULL_BLEND_PCT + t * (100.0f - BAT_FULL_BLEND_PCT);
}

// ── 부하 중 저전압 차단 래치 (updateSOC가 표시값을 묶을 때 참조) ──
static bool          batLoadBlocked    = false;
static unsigned long batLoadLowSinceMs = 0;

// 50ms마다 호출 (모터 정지 중에만)
static void updateSOC(float vBus) {
  if (!socInit) {
    // 부팅 직후 0.5초(10샘플)는 전원이 안정되기 전이라 건너뛴다
    if (socInitSkip < 10) {
      socInitSkip++;
      return;
    }
    socFiltered = ocvToSoc(vBus);
    socInit     = true;
    Serial.print("[SOC] init vBus=");
    Serial.print(vBus, 3);
    Serial.print(" soc=");
    Serial.println(socFiltered, 1);
    return;
  }

  float socRaw = ocvToSoc(vBus);

  socFiltered = SOC_BETA_FILTER * socFiltered
              + (1.0f - SOC_BETA_FILTER) * socRaw;
  socFiltered = constrain(socFiltered, 0.0f, 100.0f);

  batPct = (uint8_t)(socFiltered + 0.5f);

  // 부하 저전압으로 차단된 뒤에는 재시도 전까지 표시도 차단 임계값으로 묶는다
  // (전압이 회복되며 %가 다시 올라가 "Low Battery" 화면과 모순되지 않게).
  if (batLoadBlocked && batPct > BAT_BLOCK_PCT) batPct = BAT_BLOCK_PCT;
}

// ── 구동 중 저전압 감시 ──
// 원시 전압(BAT_SENSE_OFFSET_V 미적용)이 BAT_LOAD_CUTOFF_V 아래로 HOLD_MS 넘게
// 유지되면 차단을 래치한다. 래치 순간에는 표시 %도 차단 임계값으로 내린다.
static void updateLoadGuard(float vBus, bool motorActive) {
  if (!motorActive) {
    // 부하가 없으면 판정하지 않는다(래치 해제는 STOPPED에서 재시도할 때만)
    batLoadLowSinceMs = 0;
    return;
  }
  if (vBus <= BAT_LOAD_CUTOFF_V) {
    if (batLoadLowSinceMs == 0) {
      batLoadLowSinceMs = millis();
    } else if (millis() - batLoadLowSinceMs >= BAT_LOAD_CUTOFF_HOLD_MS) {
      if (!batLoadBlocked) {
        Serial.print("[BAT] LOAD CUTOFF latched, vBus=");
        Serial.print(vBus, 3);
        Serial.print(" -> batPct forced to ");
        Serial.println(BAT_BLOCK_PCT);
      }
      batLoadBlocked = true;
      // socFiltered까지 내려야 정지 후 IIR이 다시 끌어올리지 않는다
      socFiltered = (float)BAT_BLOCK_PCT;
      batPct      = BAT_BLOCK_PCT;
    }
  } else {
    batLoadLowSinceMs = 0;
  }
}

// 모터를 움직이면 안 되는가 — 센서 이상, 부하 중 저전압 래치, 정지 중 SOC 기준
static bool isBatteryBlockedNow() {
  return !inaAvailable || inaCalError!=0 || batLoadBlocked || Motor::isBatteryBlock(batPct);
}

// ══════════════════════════════════════════════════════════
// 백라이트 자동 디밍 — 마지막 입력 뒤 설정 시간(dimSec)이 지나면 TFT_BACKLIGHT_DIM으로.
// 모터가 도는 중과 업로드 중에는 진행 상황을 봐야 하므로 디밍하지 않는다.
// ══════════════════════════════════════════════════════════
// 밝기(%) → 백라이트 레벨(1~255). 0이 되면 화면이 꺼져 조작할 수 없으므로 하한 1.
static inline uint8_t brightPctToLevel(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint16_t lv = (uint16_t)pct * 255u / 100u;
  return (uint8_t)(lv < 1 ? 1 : lv);
}

static void updateBacklight(UiEvent ev, SystemState st, unsigned long now) {
  static bool          dimmed         = false;
  static unsigned long lastActivityMs = 0;

  bool motorBusy = (st == STATE_HOMING || st == STATE_ALIGNING ||
                    st == STATE_RUNNING || st == STATE_JOG_FWD ||
                    st == STATE_OTA);

  if (ev != UI_NONE || motorBusy) lastActivityMs = now;

  const unsigned long dimAfterMs = (unsigned long)ui.config.dimSec * 1000UL;
  bool shouldDim = !motorBusy && dimAfterMs > 0 &&
                   (now - lastActivityMs > dimAfterMs);

  const uint8_t fullLevel = brightPctToLevel(ui.config.brightPct);

  if (shouldDim != dimmed) {
    dimmed = shouldDim;
    display.setBacklight(dimmed ? TFT_BACKLIGHT_DIM : fullLevel);
    lvglInvalidateScreen();
    Serial.println(dimmed ? "[BL] dim" : "[BL] full");
  }
}

// ─── 모터 쪽 정지 원인 → 화면 정지 사유 ──────────────────────
static StopReason motorStopReason() {
  if (motor.isOverCurrent())  return STOP_OVERCURRENT;
  if (motor.isDriverFault())  return STOP_SERVO_FAULT;
  if (motor.isStalled())      return STOP_SERVO_STALL;
  if (motor.getFaultCode() == 3) return STOP_USER;   // 서보 태스크가 KO로 세움
  return STOP_LIMIT;
}

// ─── 상태 전환 ─────────────────────────────────────────────
static void transitionTo(SystemState next) {
  if (next == STATE_STOPPED || next == STATE_OTA) motor.stop();
  diagState(state, next, stopReason);
  if (next == STATE_FINISHED || next == STATE_STOPPED) {
    const bool fromRun = (state == STATE_RUNNING);
    if (fromRun) lastRunElapsedSec = motor.getElapsedSec();
    stopInjectedMl = (next == STATE_STOPPED && fromRun)
                     ? ui.config.volumeMl * constrain(motor.getProgress(), 0.0f, 1.0f) : NAN;
    stopElapsedSec = (next == STATE_STOPPED && fromRun) ? lastRunElapsedSec : NAN;
  }
  if (next == STATE_FINISHED) returnPending = true;
  else if (next == STATE_HOMING || next == STATE_ALIGNING) returnPending = false;
  state      = next;
  needRedraw = true;
  lastDisplayMs = 0;
}

// ─── 무선 업로드 시작 → 즉시 업로드 화면 ───────────────────
// ArduinoOTA는 업로드 내내 loop()를 막으므로 시작 콜백에서 바로 STATE_OTA로 넘기고
// 화면을 패널에 밀어낸다. 이후 진행률은 Ota.cpp의 onProgress가 갱신한다.
static void onOtaUploadStart() {
  transitionTo(STATE_OTA);
  display.setBacklight(brightPctToLevel(ui.config.brightPct));  // 디밍 중이었어도 밝게
  lvglShowOtaFrame(batPct);
  lvglInvalidateScreen();
  lvglBridgeTick();
  needRedraw = false;
}

// ─── 정렬 Offset 조회 ──────────────────────────────────────
static float getAlignOffset(uint8_t volumeMl) {
  for (uint8_t i = 0; i < SYRINGE_SPEC_COUNT; i++)
    if (syringeSpecs[i].volumeMl == volumeMl)
      return syringeSpecs[i].offsetMm;
  return 0.0f;
}

// ══════════════════════════════════════════════════════════
// setup
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  // 서보 정지를 가장 먼저: ESP32만 리셋되고 서보 전원은 살아 있으면 정속 모드 서보는
  // 마지막 속도로 계속 돈다. motor.begin()이 속도 0을 보내고 서보를 설정한 뒤 위치 루프
  // 태스크를 띄운다. 이 시점 로그는 USB 연결 전이라 안 보일 수 있어 diagBegin()이 다시 찍는다.
  motor.begin();
  delay(300);
  Serial.println("[BOOT] start");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);   // 긴 점퍼 배선에서 400kHz는 불안정

  display.begin();
  ui.begin();

  lvglBridgeInit();
  lvglThemeInit();
  lvglShowBlank();
  loadPowerInfoSetting();
  loadNeedleSetting();   // loadConfig()의 최소 시간 보정보다 먼저
  loadFrontEnd();
  lvglSetBatteryDebug(showPowerInfo);
  lvglSetNeedleEnabled(needleSelectOn);
  loadConfig();

  // ── INA228: 배터리 전압과 전체 소비 전류 ──
  // I2C 응답만이 아니라 칩 ID까지 확인한다(INA226은 호환되지 않음).
  if (inaBat.begin() && inaBat.getManufacturer()==0x5449 && inaBat.getDieID()==0x228) {
    inaCalError=inaBat.setMaxCurrentShunt(INA228_MAX_CURRENT_A, INA228_SHUNT_OHM);
    bool cfgOk=(inaCalError==0 && inaBat.getLastError()==0);
    // ADC range 0(±163.84mV) → R015로 약 ±10.9A. range 1은 2.73A에서 포화돼 7A 보호를 못 한다.
    cfgOk &= inaBat.setADCRange(false);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= !inaBat.getADCRange();
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= inaBat.setMode(INA228_MODE_CONT_BUS_SHUNT);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= inaBat.setBusVoltageConversionTime(INA228_1052_us);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= inaBat.setShuntVoltageConversionTime(INA228_1052_us);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= inaBat.setAverage(INA228_AVERAGE_IDX);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= (inaBat.getMode()==INA228_MODE_CONT_BUS_SHUNT);
    cfgOk &= (inaBat.getLastError()==0);
    cfgOk &= (inaBat.getAverage()==INA228_AVERAGE_IDX);
    cfgOk &= (inaBat.getLastError()==0);
    if (!cfgOk && inaCalError==0) inaCalError=-100;
    inaAvailable=cfgOk;
    Serial.print("[INA228] R015 / 10A / ADC range0 / calibration=");
    Serial.println(inaCalError);

    // ── ALERT: 하드웨어 과전류 차단 (PIN_INA_ALERT) ──
    // SOVL(션트 과전압 한계)을 넘으면 INA228이 ALERT를 LOW로 내리고 래치한다.
    // DIAG_ALRT: bit15 ALATCH=1(래치), bit13 SLOWALERT(평균값 비교),
    //            bit12 APOL=0(액티브 로우), bit14 CNVR=0(변환완료 알림 끔).
    if (cfgOk) {
      const uint16_t sovl = (uint16_t)INA228_SOVL_LSB;
      const uint16_t diag = 0x8000 | (INA228_ALERT_SLOW ? 0x2000 : 0x0000);
      inaBat.setShuntOvervoltageTH(sovl);
      inaAlertOk = inaBat.getLastError()==0 && inaBat.getShuntOvervoltageTH()==sovl;
      inaBat.setDiagnoseAlert(diag);
      inaAlertOk &= inaBat.getLastError()==0 &&
                    (inaBat.getDiagnoseAlert() & 0xF000)==diag;  // 읽기 = 이전 래치 해제
      delay(5);
      if (inaAlertOk && digitalRead(PIN_INA_ALERT)==LOW) {
        (void)inaBat.getDiagnoseAlert();   // 한 번 더 해제 후 재확인
        delay(40);
        if (digitalRead(PIN_INA_ALERT)==LOW) {
          inaAlertOk=false;
          Serial.print("WARN: INA228 ALERT 핀(GPIO");
          Serial.print(PIN_INA_ALERT);
          Serial.println(")이 LOW로 붙어 있음 — 배선/쇼트 확인. 하드웨어 차단 비활성(소프트웨어 차단만 동작)");
        }
      }
      if (inaAlertOk) {
        attachInterrupt(digitalPinToInterrupt(PIN_INA_ALERT), Motor::currentAlertISR, FALLING);
        Serial.print("[INA228] ALERT armed: GPIO");
        Serial.print(PIN_INA_ALERT);
        Serial.print(" trip ");
        Serial.print(INA228_ALERT_MA, 0);
        Serial.print("mA (SOVL ");
        Serial.print(sovl);
        Serial.print(" = ");
        Serial.print(sovl * 0.005f, 1);
        Serial.print("mV) ");
        Serial.println(INA228_ALERT_SLOW ? "averaged" : "per-conversion");
      } else {
        Serial.println("WARN: INA228 ALERT 설정 실패 — 소프트웨어 과전류 차단만 동작");
      }
    }
  } else {
    inaAvailable=false; inaCalError=-101;
    Serial.println("WARN: INA228 missing or wrong device ID; check address in Theme.h");
  }
  if (!inaAvailable) {
    batPct = 0;
    Serial.println("WARN: INA228 unavailable: motion blocked");
  }

  // ── 로고 1.5초 + 버튼 LED 점등 시험 ──
  // 로고가 떠 있는 동안 두 버튼 LED를 모두 켠다. 하나만 안 켜지면 그 LED의 배선 문제다.
  digitalWrite(PIN_FWD_LED, ledLevel(true));
  pwmWrite(PIN_RETURN_LED, LED_ACTIVE_LOW ? 0 : 255);
  Serial.print("[BOOT] LED test   : FWD GPIO");
  Serial.print(PIN_FWD_LED);
  Serial.print(" + RETURN GPIO");
  Serial.print(PIN_RETURN_LED);
  Serial.println(LED_ACTIVE_LOW ? " ON (active LOW: 핀=LOW)" : " ON (active HIGH: 핀=HIGH)");
  display.showSplash();
  display.setBacklight(brightPctToLevel(ui.config.brightPct));   // 로고를 그린 뒤 점등
  delay(1500);
  digitalWrite(PIN_FWD_LED, ledLevel(false));
  pwmWrite(PIN_RETURN_LED, LED_ACTIVE_LOW ? 255 : 0);

  attachInterrupt(digitalPinToInterrupt(PIN_EC11_KO), koButtonISR, FALLING);

  // 비정상 리셋(전원 붕괴·크래시·워치독)이면 원인을 정지 화면에 띄우고, 아니면 호밍
  if (resetWasBrownout() || resetWasWatchdog() || resetWasPanic()) {
    if      (resetWasBrownout()) stopReason = STOP_BROWNOUT;
    else if (resetWasPanic())    stopReason = STOP_PANIC;
    else                         stopReason = STOP_WATCHDOG;
    state = STATE_STOPPED;
    needRedraw = true;
    Serial.println("WARN: abnormal reset detected!");
  } else {
    lvglShowHoming(batPct);
    const float bootV = inaAvailable ? inaBat.getBusVoltage() : 0.0f;
    if (!inaAvailable || inaCalError != 0 || !isfinite(bootV) || bootV <= BAT_LOAD_CUTOFF_V) {
      stopReason = STOP_BATTERY;
      state = STATE_STOPPED;
    } else {
      motor.startHoming();
      state = STATE_HOMING;
    }
    lvglBridgeTick();   // 호밍 화면을 패널에 밀어냄
  }
  lvglLogMemUsage("after boot screens");

  diagBegin();

  // WiFi 상시 연결 — 켜져 있으면 지금 접속을 시작하고, 연결·업로드 대기는
  // loop()에서 모터가 멈춘 화면일 때 처리한다(wifiIdleState).
  loadWifiSetting();
  otaSetStartCallback(onOtaUploadStart);
  if (wifiEnabled) {
    otaBegin();
    Serial.println("[BOOT] Wi-Fi ON (설정 메뉴에서 끌 수 있음) — 접속 시작");
  } else {
    Serial.println("[BOOT] Wi-Fi OFF");
  }

  wdtBegin(WDT_TIMEOUT_MS);   // 마지막에 켠다(setup의 긴 지연이 물지 않게)
  Serial.println("[BOOT] task watchdog armed, setup() complete");
  logHeapInfo("setup done");
}

// ══════════════════════════════════════════════════════════
// loop
// ══════════════════════════════════════════════════════════
// 모터가 도는 중에는 화면 갱신을 늦춰 메인 루프가 오래 멈추지 않게 한다
static inline unsigned long displayIntervalMs(SystemState st) {
  const bool moving = (st == STATE_HOMING   || st == STATE_ALIGNING ||
                       st == STATE_RUNNING  || st == STATE_JOG_FWD);
  return moving ? DISPLAY_UPDATE_MS_MOVING : DISPLAY_UPDATE_MS;
}

void loop() {
  diagLoopTick();

  // WiFi: 모터가 멈춘 화면(대기·설정·완료·정지)에서만 접속 관리와 업로드 수신을 돌린다.
  // 저배터리면 업로드를 받지 않는다(플래시 쓰는 중 전원 붕괴 방지).
  telnetLogTick();
  {
    const bool wifiIdleState =
        state == STATE_READY || state == STATE_ALIGNED_READY || state == STATE_SYSMENU ||
        state == STATE_SET_VOL || state == STATE_SET_TIME || state == STATE_SET_NEEDLE ||
        state == STATE_FINISHED || state == STATE_STOPPED;
    if (wifiEnabled && wifiIdleState && !isBatteryBlockedNow()) {
      OtaState prevOtaSt = otaStatus();
      otaLoop();
      if (prevOtaSt != OTA_UPDATING && otaStatus() == OTA_UPDATING && state != STATE_OTA) {
        transitionTo(STATE_OTA);   // 보통은 시작 콜백(onOtaUploadStart)이 이미 넘겨 둠
      }
      // 접속 실패(공유기 꺼짐 등) → 잠시 뒤 다시 시도
      if (otaStatus() == OTA_FAILED && otaConfigured()) {
        if (!wifiFailedSinceMs) wifiFailedSinceMs = millis() | 1UL;
        else if (millis() - wifiFailedSinceMs > WIFI_RETRY_MS) {
          Serial.println("[WIFI] retry");
          otaStop(); otaBegin(); wifiFailedSinceMs = 0;
        }
      } else wifiFailedSinceMs = 0;
    }
    // 상단 WiFi 아이콘 (0.5초마다 상태 확인)
    static unsigned long lastWifiIconMs = 0;
    if (millis() - lastWifiIconMs > 500) {
      lastWifiIconMs = millis();
      lvglSetWifiIndicator(wifiEnabled && otaConnected());
    }
  }

  wdtFeed();
  lvglBridgeTick();

  UiEvent ev  = ui.poll();
  unsigned long now = millis();

  updateBacklight(ev, state, now);

  // INA228이 없으면 아래 측정 블록이 돌지 않으므로 여기서 "센서 미검출"을 띄운다
  if (showPowerInfo && !inaAvailable) {
    static unsigned long lastNoSensorMs = 0;
    if (now - lastNoSensorMs > 1000) {
      lastNoSensorMs = now;
      lvglUpdateBatteryDebugNoSensor();
    }
  }

  // 정기 진단 리포트 (구동 중 2초, 대기 중 15초 — 주기는 내부에서 판단)
  {
    bool motorBusy = (state == STATE_HOMING || state == STATE_ALIGNING ||
                      state == STATE_RUNNING || state == STATE_JOG_FWD);
    diagReport(state, motorBusy);
  }

  // ── 버튼 LED ─────────────────────────────────────────────
  // 규칙: 점등 = 이 버튼을 누를 차례, 점멸 = 이 버튼으로 시작한 동작이 진행 중.
  //   FWD    점등: 정렬 완료(누르면 주입 시작), 주입 일시정지(누르면 재개)
  //          점멸: 주입 중, 수동 전진 조그 중
  //   RETURN 점등: 주입 완료 후 아직 후진 전(누르면 원점 복귀)
  //          점멸: 후진(원점 찾기) 중 — 부팅·재시도 때의 원점 찾기 포함
  {
    const bool blink  = ((now / BUTTON_LED_BLINK_MS) & 1) == 0;
    const bool paused = (state == STATE_RUNNING) && motor.isDispensePaused() && !resumeRequested;

    bool fwdOn;
    if ((state == STATE_RUNNING && !paused) || state == STATE_JOG_FWD) fwdOn = blink;
    else if (paused || state == STATE_ALIGNED_READY)                    fwdOn = true;
    else                                                               fwdOn = false;
    digitalWrite(PIN_FWD_LED, ledLevel(fwdOn));

    bool retOn;
    if (state == STATE_HOMING) retOn = blink;
    else if (returnPending && (state == STATE_FINISHED || state == STATE_READY)) retOn = true;
    else retOn = false;
    // RETURN LED는 LEDC 핀이라 0/255로 켜고 끈다(액티브 로우면 반전)
    const uint8_t duty = retOn ? 255 : 0;
    pwmWrite(PIN_RETURN_LED, LED_ACTIVE_LOW ? (255 - duty) : duty);
  }

  // 서보 태스크의 로그를 찍고 "loop() 살아 있음" 신호를 준다.
  // 토크는 대기·KO·업로드 중에도 유지하고, 전기적 이상(과전류)일 때만 해제된다.
  motor.service();

  // 배터리 위험 경고창 — "Low Battery" 정지 화면 위에는 띄우지 않는다(안내를 가림)
  {
    bool battStopScreen = (state == STATE_STOPPED && stopReason == STOP_BATTERY);
    lvglSetBatteryWarning(batPct <= BAT_BLOCK_PCT && !battStopScreen);
  }

  // ══ 안전 1: CANCEL(KO) ══════════════════════════════
  if (emergencyStopFlag) {
    emergencyStopFlag = false;
    motor.emergencyStop(); // 속도 0, 토크 유지(피스톤을 놓지 않음)
    if (state == STATE_RUNNING  ||
        state == STATE_ALIGNING ||
        state == STATE_HOMING   ||
        state == STATE_JOG_FWD) {
      stopReason = STOP_USER;
      transitionTo(STATE_STOPPED);
      return;
    }
    motor.clearFaults();

    // 업로드 화면·설정 메뉴·설정 화면에서 CANCEL = 나가기
    if (state == STATE_OTA) {
      otaStop();
      if (wifiEnabled) otaBegin();   // 상시 연결이면 다시 접속해 계속 대기
      transitionTo(STATE_READY);
      return;
    }
    if (state == STATE_SYSMENU) {   // 메뉴 값은 고를 때마다 저장되므로 되돌리지 않는다
      transitionTo(STATE_READY);
      return;
    }
    if (state == STATE_SET_VOL || state == STATE_SET_TIME ||
        state == STATE_SET_NEEDLE) {
      ui.config = cfgSnapshot;   // 취소 = 진입 전 값으로 원복
      Serial.println("[CFG] setup cancelled, reverted");
      transitionTo(STATE_READY);
    }
  }

  // ══ 안전 2: 리밋·소프트 리밋·모터 이상 (서보 태스크도 독립적으로 감시) ══
  // 예외: 주입 끝(완료 허용 범위)에서 눌린 전방 리밋은 주사기를 끝까지 민 것이므로
  // 가로채지 않고 STATE_RUNNING이 완료로 처리하게 둔다.
  const bool frontIsEnd = state == STATE_RUNNING && motor.frontLimitIsDispenseEnd();
  // 전방 리밋에서 주입을 마친 뒤의 정렬은 리밋에서 멀어지는 쪽이라 막지 않는다
  const bool alignAway = state == STATE_ALIGNING && alignTargetMm < motor.getPositionMm();
  if ((state == STATE_RUNNING || state == STATE_ALIGNING) &&
      ((motor.isFrontLimit() && !frontIsEnd && !alignAway) || motor.isSoftLimit() || motor.hasMotionFault())) {
    if (state == STATE_RUNNING && motor.isFrontLimit() && !frontIsEnd) noteEarlyFrontLimit();
    stopReason = motorStopReason();
    transitionTo(STATE_STOPPED);
    return;
  }

  // ══ 안전 3: 과전류 (ALERT 인터럽트 또는 아래 50ms 측정) ══
  // ALERT는 대기 중에도 걸릴 수 있으므로(토크 해제됨) 정지 화면으로 보낸다.
  if (motor.isOverCurrent() &&
      state != STATE_STOPPED && state != STATE_OTA) {
    stopReason = STOP_OVERCURRENT;
    transitionTo(STATE_STOPPED);
    return;
  }

  // ── 전압·전류 측정 (50ms) ──
  {
    static unsigned long lastSocMs = 0;
    if (inaAvailable && now - lastSocMs >= 50UL) {
      lastSocMs = now;
      if (!inaBat.isConnected()) {
        inaAvailable=false;
        stopReason=STOP_BATTERY;
        transitionTo(STATE_STOPPED);
        return;
      }
      // ALERT 래치 해제 + 원인 기록. 서보는 이미 태스크가 세웠다.
      // DIAG_ALRT를 읽으면 래치가 풀리고, 전류가 여전히 높으면 곧 다시 걸린다.
      // Motor 쪽 차단 상태는 clearFaults()가 핀 HIGH를 확인한 뒤에만 풀린다.
      if (inaAlertOk && motor.isCurrentAlert() && digitalRead(PIN_INA_ALERT)==LOW) {
        const uint16_t d = inaBat.getDiagnoseAlert();
        static unsigned long lastAlertLogMs = 0;
        if (now - lastAlertLogMs > 1000UL) {
          lastAlertLogMs = now;
          Serial.print("[INA228] DIAG_ALRT=0x");
          Serial.print(d, HEX);
          Serial.println((d & (1u << 6)) ? " (SHNTOL: 션트 과전압 = 과전류)" : "");
        }
      }
      float shuntMV = inaBat.getShuntMilliVolt();
      const int shuntReadError=inaBat.getLastError();
      float vBus = inaBat.getBusVoltage() + shuntMV / 1000.0f;
      const int busReadError=inaBat.getLastError();
      float iMa = shuntMV / INA228_SHUNT_OHM;
      if (shuntReadError!=0 || busReadError!=0 || !isfinite(shuntMV) || !isfinite(vBus) || vBus<=0.0f) {
        inaAvailable=false;
        stopReason=STOP_BATTERY;
        transitionTo(STATE_STOPPED); return;
      }
      bool active = (state == STATE_RUNNING || state == STATE_ALIGNING ||
                     state == STATE_HOMING  || state == STATE_JOG_FWD);
      if (!active) updateSOC(vBus);   // 부하 중 전압은 처지므로 표시 %는 정지 중에만 갱신
      motor.updateCurrentMa(isfinite(iMa) ? fmaxf(iMa,0.0f) : iMa);
      if (showPowerInfo) {   // V=는 보정 전 원시값
        lvglUpdateBatteryDebug(vBus, iMa, batPct, inaCalError);
      }

      // 션트 전압으로 환산한 전류와 라이브러리 계산값을 나란히 찍는다
      if (LOG_CURRENT_RAW) {
        static unsigned long lastRawMs = 0;
        if (now - lastRawMs >= 1000) {
          lastRawMs = now;
          Serial.print("[IRAW] shunt=");
          Serial.print(shuntMV, 3);
          Serial.print("mV  lib=");
          Serial.print(inaBat.getMilliAmpere(), 1);
          Serial.print("mA  BAT=");
          Serial.print(iMa, 0);
          Serial.print("mA  bus=");
          Serial.print(inaBat.getBusVoltage(), 3);
          Serial.print("V  active=");
          Serial.println(active ? "1" : "0");
        }
      }
      updateLoadGuard(vBus, active);
      diagSample(vBus, iMa, active);
    }
  }

  // ══════════════════════════════════════════════════════
  switch (state) {

    // ── HOMING (non-blocking) ─────────────────────────
    case STATE_HOMING: {
      if (isBatteryBlockedNow()) {
        stopReason=STOP_BATTERY;
        transitionTo(STATE_STOPPED); break;
      }
      // 더블클릭 → 호밍을 건너뛰고 메인 화면으로(원점이 없어 정렬은 재호밍 전까지 불가)
      if (ev == UI_PUSH_DOUBLE) {
        motor.emergencyStop();
        Serial.println("[HOMING] double-click -> skip homing, go READY");
        transitionTo(STATE_READY);
        break;
      }
      // FWD로 호밍 중단 → 정지 화면에서 FWD를 다시 누르면 수동 전진 조그
      if (ev == UI_FWD) {
        stopReason = STOP_USER;
        homingInterruptArmed = true;
        transitionTo(STATE_STOPPED);
        break;
      }
      // RETURN으로 호밍 중단 → 단순 정지 (정지 화면에서 RETURN = 재호밍)
      if (ev == UI_RETURN) {
        stopReason = STOP_USER;
        transitionTo(STATE_STOPPED);
        break;
      }

      HomingState hs = motor.updateHoming();

      if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowHoming(batPct);
      }

      if (hs == HOMING_DONE) {
        Serial.println("[HOMING] DONE -> READY");
        transitionTo(STATE_READY);
      } else if (hs == HOMING_TIMEOUT || hs == HOMING_ERROR) {
        Serial.print("[HOMING] FAILED, hs=");
        Serial.print((int)hs);
        Serial.print(" overCurrent=");
        Serial.println(motor.isOverCurrent());
        // 호밍 중 막힘은 대개 HOME 스위치를 못 찾고 기구 끝에 닿은 경우라 "Homing Failed"로 안내
        if (motor.isOverCurrent() || motor.isDriverFault())
          stopReason = motorStopReason();
        else
          stopReason = STOP_HOMING_FAIL;
        transitionTo(STATE_STOPPED);
      }
      break;
    }

    // ── 메인 대기 ────────────────────────────────────
    case STATE_READY:
    case STATE_ALIGNED_READY:
      if (needRedraw) {
        lvglShowReady(ui.config, state, batPct);
        needRedraw = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;   // 배터리 % 등 배경 값 갱신
        lvglShowReady(ui.config, state, batPct);
      }

      if (ev == UI_PUSH_DOUBLE) {
        cfgSnapshot = ui.config;   // 취소 시 되돌릴 값 보관
        transitionTo(STATE_SET_VOL);

      } else if (ev == UI_PUSH_SINGLE) {
        alignTargetMm = getAlignOffset(ui.config.volumeMl);
        transitionTo(STATE_ALIGNING);

      } else if (ev == UI_PUSH_LONG) {   // 시스템 설정 메뉴
        motor.stop();
        sysMenuSel = 0;
        transitionTo(STATE_SYSMENU);

      } else if (ev == UI_FWD && state == STATE_ALIGNED_READY) {
        if (isBatteryBlockedNow()) {
          stopReason = STOP_BATTERY;
          transitionTo(STATE_STOPPED);
          break;
        }
        // 주입 거리 = 주사기가 비는 위치(학습값) − 현재(정렬) 위치.
        // 소프트 리밋 안쪽 경계를 상한으로 둔다.
        const float endMm = fminf(frontEndMm,
                                  MAX_TRAVEL_MM - SOFT_LIMIT_MARGIN - DISPENSE_TARGET_MARGIN_MM);
        float totalMm = constrain(endMm - motor.getPositionMm(), 0.0f, MAX_TRAVEL_MM);
        Serial.print("[FWD] positionMm=");
        Serial.print(motor.getPositionMm());
        Serial.print(" alignTargetMm=");
        Serial.print(alignTargetMm);
        Serial.print(" totalMm=");
        Serial.println(totalMm);
        enforceTime();
        motor.startDispense(totalMm, (float)ui.config.timeSec, true);   // 끝에서 전방 리밋까지 확인
        transitionTo(STATE_RUNNING);

      } else if (ev == UI_RETURN) {   // 재호밍
        if (isBatteryBlockedNow()) {
          stopReason=STOP_BATTERY; transitionTo(STATE_STOPPED); break;
        }
        motor.clearFaults();
        lvglShowHoming(batPct);
        motor.startHoming();
        transitionTo(STATE_HOMING);
      }
      break;

    // ── 설정: 용량 → (니들) → 시간. 니들 선택이 꺼져 있으면 니들 단계를 건너뛴다 ──
    // ── 설정 1 용량 ──────────────────────────────────
    case STATE_SET_VOL:
      if (needRedraw) {
        lvglShowSetVolume(ui.config.volumeMl, batPct);
        needRedraw = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowSetVolume(ui.config.volumeMl, batPct);
      }
      if      (ev == UI_ROT_CW  && ui.config.volumeMl < CFG_VOL_MAX_ML)
        { ui.config.volumeMl += CFG_VOL_STEP_ML; needRedraw = true; }
      else if (ev == UI_ROT_CCW && ui.config.volumeMl > CFG_VOL_MIN_ML)
        { ui.config.volumeMl -= CFG_VOL_STEP_ML; needRedraw = true; }
      else if (ev == UI_PUSH_SINGLE) transitionTo(needleSelectOn ? STATE_SET_NEEDLE : STATE_SET_TIME);
      break;

    // ── 설정 마지막 시간 ─────────────────────────────
    case STATE_SET_TIME:
      if (enforceTime()) needRedraw = true;
      if (needRedraw) {
        lvglShowSetTime(ui.config.timeSec, batPct, minTimeSec());
        needRedraw = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowSetTime(ui.config.timeSec, batPct, minTimeSec());
      }
      // 10초 눈금으로 이동, 최소값에서 멈춘다
      if      (ev == UI_ROT_CW  && ui.config.timeSec < CFG_TIME_MAX_SEC)
        { ui.config.timeSec = timeStepUp(ui.config.timeSec); enforceTime(); needRedraw = true; }
      else if (ev == UI_ROT_CCW && ui.config.timeSec > minTimeSec())
        { ui.config.timeSec = timeStepDown(ui.config.timeSec, minTimeSec()); enforceTime(); needRedraw = true; }
      else if (ev == UI_PUSH_SINGLE) {
        saveConfig();
        transitionTo(STATE_READY);
      }
      break;

    // ── 설정 2 바늘 (니들 선택이 켜져 있을 때만) ─────
    case STATE_SET_NEEDLE:
      if (needRedraw) {
        lvglShowSetNeedle(ui.config.needleG, batPct);
        needRedraw = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowSetNeedle(ui.config.needleG, batPct);
      }
      if      (ev == UI_ROT_CW  && ui.config.needleG < CFG_NEEDLE_MAX_G)
        { ui.config.needleG++; enforceTime(); needRedraw = true; }
      else if (ev == UI_ROT_CCW && ui.config.needleG > CFG_NEEDLE_MIN_G)
        { ui.config.needleG--; enforceTime(); needRedraw = true; }
      else if (ev == UI_PUSH_SINGLE) transitionTo(STATE_SET_TIME);
      break;

    // ── ALIGNING (non-blocking startMoveTo) ──────────
    case STATE_ALIGNING: {
      if (isBatteryBlockedNow()) {
        stopReason = STOP_BATTERY;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 진입 때 한 번: 이동 시작 + 화면 틀 그리기 (목표가 뒤쪽일 수도 있어 절대값 비교)
      if (needRedraw) {
        needRedraw = false;
        float dist = alignTargetMm - motor.getPositionMm();
        if (fabsf(dist) < 0.5f) {
          transitionTo(STATE_ALIGNED_READY);
          break;
        }
        motor.startMoveTo(alignTargetMm);
        lvglShowAligningFrame(batPct, ui.config.volumeMl, ui.config.timeSec);
      }

      MoveState ms = motor.updateMoveTo();

      if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglUpdateAligningValues(motor.getMoveProgress(), batPct);   // 정·역방향 모두 0~1
      }

      if (ms == MOVE_DONE) {
        transitionTo(STATE_ALIGNED_READY);
      } else if (ms == MOVE_ERROR) {   // 대부분 loop() 상단 안전 검사가 먼저 잡는다
        stopReason = motorStopReason();
        transitionTo(STATE_STOPPED);
      } else if (ms == MOVE_TIMEOUT) {
        stopReason = STOP_MOVE_TIMEOUT;
        transitionTo(STATE_STOPPED);
      }
      break;
    }

    // ── RUNNING ───────────────────────────────────────
    case STATE_RUNNING: {
      // FWD: 주입 중이면 일시정지, 일시정지 중이면 남은 시간으로 재개.
      // 감속이 끝나기 전에 재개를 누르면 기억해 두었다가 정지 즉시 재개한다(다시 누르면 취소).
      if (ev == UI_FWD) {
        if (!motor.isDispensePaused()) {
          resumeRequested=false;
          motor.pauseDispense();
          Serial.println("[RUN] FWD -> pause");
        } else {
          resumeRequested=!resumeRequested;
          Serial.println(resumeRequested ? "[RUN] FWD -> resume" : "[RUN] FWD -> resume cancelled");
        }
        lvglSetRunningPaused(motor.isDispensePaused() && !resumeRequested);
      }
      if (resumeRequested && motor.isDispensePaused()) {
        motor.resumeDispense();
        if (!motor.isDispensePaused()) lvglSetRunningPaused(false);
      }
      if (!motor.isDispensePaused()) resumeRequested=false;

      // 일시정지 방치 (주사기가 압력을 받은 채 남지 않게)
      if (motor.getPauseElapsedMs() > DISPENSE_PAUSE_TIMEOUT_MS) {
        Serial.println("[RUN] pause timeout -> STOPPED");
        lvglSetRunningPaused(false);
        stopReason = STOP_PAUSE_TIMEOUT;
        transitionTo(STATE_STOPPED);
        break;
      }

      // RETURN: 정지 (정지 화면에서 한 번 더 누르면 재호밍)
      if (ev == UI_RETURN) {
        stopReason = STOP_USER;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 주입 중에는 batPct가 멈춰 있으므로 실제 트리거는 부하 중 저전압 래치다
      if (isBatteryBlockedNow()) {
        stopReason = STOP_BATTERY;
        transitionTo(STATE_STOPPED);
        break;
      }

      if (needRedraw) {
        needRedraw = false;
        lvglShowRunningFrame(batPct, ui.config.volumeMl);
        lvglSetRunningPaused(motor.isDispensePaused());
      }

      bool running = motor.updateDispense();

      if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglUpdateRunningValues(motor.getProgress(), motor.getElapsedSec(), batPct);
      }

      if (!running) {
        if (motor.isDispenseComplete()) {
          // 정상 완료 (끝 무렵 전방 리밋에 닿은 경우 포함)
          Serial.printf("[RUN] complete%s  pos=%.2fmm  target=%.2fmm  seek=+%.2fmm  elapsed=%.1fs / set %us\n",
                        motor.isDispenseEndedAtFront() ? " (front limit at end of stroke)" : "",
                        motor.getPositionMm(), frontEndMm, motor.dispenseSeekMm(),
                        motor.getElapsedSec(), (unsigned)ui.config.timeSec);
          learnFrontEnd(motor.dispenseFrontPressMm());   // 리밋에서 끝났으면 그 위치를 다음 목표로
          if (motor.dispenseSeekMissed())
            Serial.printf("[RUN] WARN front limit not found within %.1fmm past target — check FRONT switch\n",
                          FRONT_SEEK_MAX_MM);
          transitionTo(STATE_FINISHED);
        } else if (motor.isFrontLimit()) {
          noteEarlyFrontLimit();
          stopReason = STOP_LIMIT;
          transitionTo(STATE_STOPPED);
        } else if (motor.isSoftLimit()) {   // 목표는 그 안쪽이므로 이상 상황
          stopReason = STOP_LIMIT;
          transitionTo(STATE_STOPPED);
        } else if (motor.isOverCurrent() || motor.isDriverFault() || motor.isStalled()) {
          stopReason = motorStopReason();
          transitionTo(STATE_STOPPED);
        } else if (motor.isDispenseIncomplete()) {   // 설정 시간 안에 목표 거리를 못 채움
          stopReason = STOP_INCOMPLETE;
          transitionTo(STATE_STOPPED);
        } else {
          transitionTo(STATE_FINISHED);
        }
      }
      break;
    }

    // ── FINISHED ──────────────────────────────────────
    case STATE_FINISHED:
      if (needRedraw) {
        lvglShowFinished(batPct, ui.config.volumeMl, lastRunElapsedSec, ui.config.needleG);
        needRedraw        = false;
        finishedEnteredMs = now;
      }
      // 완료 화면은 버튼을 누를 때까지 유지한다.
      //   RETURN → 원점 복귀 시작,  PRESS → 메인 화면,  FWD·회전 → 무시
      // 화면이 뜬 직후 0.8초는 입력을 받지 않는다(완료 직전 누름이 넘어오지 않게).
      if (now - finishedEnteredMs > 800UL && ev == UI_RETURN) {
        if (isBatteryBlockedNow()) {
          stopReason=STOP_BATTERY; transitionTo(STATE_STOPPED); break;
        }
        motor.clearFaults();
        lvglShowHoming(batPct);
        motor.startHoming();
        transitionTo(STATE_HOMING);
      } else if (now - finishedEnteredMs > 800UL &&
                 (ev == UI_PUSH_SINGLE || ev == UI_PUSH_DOUBLE)) {
        transitionTo(STATE_READY);
      }
      break;

    // ── STOPPED ───────────────────────────────────────
    case STATE_STOPPED:
      if (needRedraw) {
        lvglShowStopped(batPct, stopReason, stopInjectedMl, ui.config.volumeMl, stopElapsedSec);
        needRedraw = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowStopped(batPct, stopReason, stopInjectedMl, ui.config.volumeMl, stopElapsedSec);
      }

      // 호밍을 FWD로 멈춘 경우: FWD를 다시 누르면 수동 전진 조그 (FWD·CANCEL로 정지)
      if (homingInterruptArmed && ev == UI_FWD) {
        homingInterruptArmed = false;
        motor.jogForward();
        transitionTo(STATE_JOG_FWD);
        break;
      }

      if (ev == UI_PUSH_SINGLE || ev == UI_RETURN) {   // 재시도 = 재호밍
        homingInterruptArmed = false;
        // 부하 저전압 래치는 전압이 회복됐으면 풀어 준다
        if (batLoadBlocked && inaAvailable && inaCalError==0) {
          float retryV=inaBat.getBusVoltage();
          if (isfinite(retryV) && retryV>BAT_LOAD_CUTOFF_V+0.3f) {
            batLoadBlocked=false; batLoadLowSinceMs=0;
            socFiltered=ocvToSoc(retryV); batPct=(uint8_t)(socFiltered+0.5f);
          }
        }
        if (isBatteryBlockedNow()) {
          stopReason=STOP_BATTERY; needRedraw=true; break;
        }
        motor.clearFaults();        // 서보 재설정(필요 시) + 플래그 초기화
        batLoadBlocked    = false;
        batLoadLowSinceMs = 0;
        motor.startHoming();
        transitionTo(STATE_HOMING);
      }
      break;

    // ── 시스템 설정 메뉴 ──────────────────────────────
    case STATE_SYSMENU: {
      if (ev == UI_ROT_CW) {
        sysMenuSel = (sysMenuSel + 1) % SYS_ITEM_COUNT;
        needRedraw = true;
        Serial.print("[SYS] CW -> sel=");  Serial.println(sysMenuSel);
      } else if (ev == UI_ROT_CCW) {
        sysMenuSel = (sysMenuSel + SYS_ITEM_COUNT - 1) % SYS_ITEM_COUNT;
        needRedraw = true;
        Serial.print("[SYS] CCW -> sel="); Serial.println(sysMenuSel);
      } else if (ev == UI_PUSH_SINGLE) {
        switch (sysMenuSel) {
          case SYS_WIFI:
            // 누를 때마다 WiFi 상시 연결 On/Off (즉시 적용, NVS 저장)
            wifiEnabled = !wifiEnabled;
            saveWifiSetting();
            if (wifiEnabled) { otaBegin(); Serial.println("[WIFI] ON"); }
            else             { otaStop();  Serial.println("[WIFI] OFF"); }
            wifiFailedSinceMs = 0;
            needRedraw = true;
            break;

          case SYS_BRIGHT: {
            // 누를 때마다 한 단계씩, 최대에서 최소로 돌아온다(즉시 반영)
            int v = ui.config.brightPct + SYS_BRIGHT_STEP_PCT;
            if (v > SYS_BRIGHT_MAX_PCT) v = SYS_BRIGHT_MIN_PCT;
            ui.config.brightPct = (uint8_t)v;
            display.setBacklight(brightPctToLevel(ui.config.brightPct));
            saveConfig();
            lvglInvalidateScreen();   // 밝기 전환 순간 어긋난 픽셀이 있으면 전체 다시 그림
            needRedraw = true;
            break;
          }

          case SYS_DIM_TIME: {
            uint8_t idx = 0;
            for (uint8_t i = 0; i < SYS_DIM_CHOICE_COUNT; i++)
              if (SYS_DIM_CHOICES[i] == ui.config.dimSec) { idx = i; break; }
            idx = (idx + 1) % SYS_DIM_CHOICE_COUNT;
            ui.config.dimSec = SYS_DIM_CHOICES[idx];
            saveConfig();
            needRedraw = true;
            break;
          }

          case SYS_POWER:
            // 누를 때마다 화면 맨 위 전압·전류 표시 On/Off (NVS 저장)
            showPowerInfo = !showPowerInfo;
            savePowerInfoSetting();
            lvglSetBatteryDebug(showPowerInfo);
            Serial.println(showPowerInfo ? "[SYS] power info ON" : "[SYS] power info OFF");
            needRedraw = true;
            break;

          case SYS_NEEDLE:
            // 누를 때마다 니들 선택 On/Off (NVS 저장). 최소 시간이 바뀌므로 시간도 다시 맞춘다
            needleSelectOn = !needleSelectOn;
            saveNeedleSetting();
            lvglSetNeedleEnabled(needleSelectOn);
            if (enforceTime()) saveConfig();
            Serial.print(needleSelectOn ? "[SYS] needle select ON" : "[SYS] needle select OFF");
            Serial.print("  min time="); Serial.println(minTimeSec());
            needRedraw = true;
            break;

          case SYS_INFO:
            // IP는 화면에 나와 있고, 누르면 진단 리포트를 시리얼/Telnet에 찍는다
            diagReport(state, false);
            break;

          case SYS_EXIT:
          default:
            transitionTo(STATE_READY);
            break;
        }
      } else if (ev == UI_RETURN) {
        transitionTo(STATE_READY);
      }

      if (needRedraw) {
        lvglShowSysMenu(ui.config, sysMenuSel, batPct);
        needRedraw    = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglUpdateSysMenu(ui.config, sysMenuSel, batPct);
      }
      break;
    }

    // ── 무선 업로드 진행 화면 ─────────────────────────
    case STATE_OTA: {
      otaLoop();

      if (needRedraw) {
        lvglShowOtaFrame(batPct);
        lvglUpdateOta();
        needRedraw    = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglShowOtaFrame(batPct);
        lvglUpdateOta();
      }

      // 접속 실패·대기 시간 초과 → 복귀 (업로드가 성공하면 스스로 재부팅한다)
      if (otaTimedOut()) {
        Serial.println("[OTA] 시간 초과 — READY 복귀");
        otaStop();
        if (wifiEnabled) otaBegin();   // 상시 연결이면 다시 접속
        transitionTo(STATE_READY);
        break;
      }
      // 업로드 실패 표시는 CANCEL로 빠져나간다
      break;
    }

    // ── 수동 전진 조그 (호밍 중단 후) ────────────────────
    // loop() 상단 안전 2는 정렬·주입만 보므로 리밋·과전류·저배터리를 여기서 처리한다.
    case STATE_JOG_FWD: {
      if (motor.hasMotionFault()) {
        stopReason=motorStopReason();
        transitionTo(STATE_STOPPED); break;
      }
      if (needRedraw) {
        lvglShowJogFrame(batPct);
        lvglUpdateJogValues(motor.getPositionMm(), motor.isOriginValid(), batPct);
        needRedraw    = false;
        lastDisplayMs = now;
      } else if (now - lastDisplayMs > displayIntervalMs(state)) {
        lastDisplayMs = now;
        lvglUpdateJogValues(motor.getPositionMm(), motor.isOriginValid(), batPct);
      }

      // 1) 전방 리밋 — 원점과 무관하게 항상 유효
      if (motor.isFrontLimit()) {
        Serial.println("[JOG] STOP: front limit");
        stopReason = STOP_LIMIT;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 2) 소프트 리밋 — 원점이 있을 때만 (호밍 전이면 화면에 "리밋 스위치만" 표시)
      if (motor.isOriginValid() && motor.isSoftLimit()) {
        Serial.print("[JOG] STOP: soft limit, posMm=");
        Serial.println(motor.getPositionMm());
        stopReason = STOP_LIMIT;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 3) 과전류
      if (motor.isOverCurrent()) {
        Serial.println("[JOG] STOP: overcurrent");
        stopReason = STOP_OVERCURRENT;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 4) 부하 중 저전압 차단
      if (isBatteryBlockedNow()) {
        Serial.println("[JOG] STOP: battery");
        stopReason = STOP_BATTERY;
        transitionTo(STATE_STOPPED);
        break;
      }

      // 5) FWD를 다시 누르면 정지 (CANCEL은 loop() 상단에서 처리)
      if (ev == UI_FWD) {
        stopReason = STOP_USER;
        transitionTo(STATE_STOPPED);
      }
      break;
    }

    default: break;
  }
}
