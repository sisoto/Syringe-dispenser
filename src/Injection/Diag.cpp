#include "Diag.h"
#include "Platform.h"
#include "TelnetLog.h"
#include "Motor.h"
#include <Wire.h>
#include "driver/gpio.h"

// ── Serial 미러링 ─────────────────────────────────────────
// 이 아래의 Serial.print를 USB+Telnet 양쪽으로 보낸다.
// 반드시 모든 #include 뒤에 둘 것 — 위에 두면 라이브러리 헤더 내부의
// Serial까지 LOG로 치환되어 부팅이 깨진다(ArduinoOTA/WiFi 등).
#define Serial LOG

// ── 루프 주기 ─────────────────────────────────────────────
static unsigned long _lastLoopUs   = 0;
static unsigned long _loopMaxUs    = 0;
static unsigned long _loopSumUs    = 0;
static unsigned long _loopCount    = 0;
static unsigned long _loopWorstEver = 0;

// ── 전압/전류 ─────────────────────────────────────────────
static float _vIdle    = 0;      // 최근 무부하 전압
static float _iIdle    = 0;      // 최근 무부하 전류
static float _vMinLoad = 99.0f;  // 부하 중 최저 전압
static float _iMaxLoad = 0;      // 부하 중 최대 전류
static float _pathROhm = -1.0f;  // 추정 경로 저항
static bool  _sagWarned = false;

// ── 리포트 타이밍 ─────────────────────────────────────────
static unsigned long _lastReportMs = 0;
static SystemState   _lastState    = STATE_HOMING;

const char* stateName(SystemState s) {
  switch (s) {
    case STATE_HOMING:        return "HOMING";
    case STATE_READY:         return "READY";
    case STATE_SET_VOL:       return "SET_VOL";
    case STATE_SET_TIME:      return "SET_TIME";
    case STATE_SET_NEEDLE:    return "SET_NEEDLE";
    case STATE_ALIGNING:      return "ALIGNING";
    case STATE_ALIGNED_READY: return "ALIGNED";
    case STATE_RUNNING:       return "RUNNING";
    case STATE_FINISHED:      return "FINISHED";
    case STATE_STOPPED:       return "STOPPED";
    case STATE_JOG_FWD:       return "JOG_FWD";
    case STATE_OTA:           return "OTA";
    case STATE_SYSMENU:       return "SYSMENU";
    default:                  return "?";
  }
}

const char* stopReasonName(StopReason r) {
  switch (r) {
    case STOP_NONE:          return "-";
    case STOP_LIMIT:         return "LIMIT";
    case STOP_OVERCURRENT:   return "OVERCURRENT";
    case STOP_USER:          return "USER";
    case STOP_BATTERY:       return "BATTERY";
    case STOP_HOMING_FAIL:   return "HOMING_FAIL";
    case STOP_MOVE_TIMEOUT:  return "MOVE_TIMEOUT";
    case STOP_WATCHDOG:      return "WATCHDOG";
    case STOP_INCOMPLETE:    return "INCOMPLETE";
    case STOP_PAUSE_TIMEOUT: return "PAUSE_TIMEOUT";
    case STOP_BROWNOUT:      return "BROWNOUT";
    case STOP_PANIC:         return "PANIC";
    case STOP_SERVO_STALL:   return "SERVO_STALL";
    case STOP_SERVO_FAULT:   return "SERVO_FAULT";
    default:                 return "?";
  }
}

// ══════════════════════════════════════════════════════════
void diagBegin() {
  Serial.println();
  Serial.println("========================================");
  Serial.println(" Syringe Dispenser / ESP32-S3");
  Serial.println("========================================");

  // ① 리셋 사유
  Serial.print("[BOOT] reset reason : ");
  Serial.println(resetReasonStr());
  if (resetWasBrownout())
    Serial.println("[BOOT] !! 전원 붕괴로 리셋됨 — 배터리 경로/배선 점검 필요");
  if (resetWasWatchdog())
    Serial.println("[BOOT] !! 루프가 멈춰 워치독 리셋 — 아래 loop 통계 확인");
  if (resetWasPanic())
    Serial.println("[BOOT] !! 펌웨어 크래시 후 재부팅 — 직전 로그 확인");

  // ── 칩 정보: N16R8(16MB Flash / 8MB Octal PSRAM)로 인식되는지 ──
  // IDE 설정과 실제 칩이 어긋나면 파티션 검증 실패로 부팅이 반복된다.
  Serial.print("[CHIP] flash ");
  Serial.print(ESP.getFlashChipSize() / 1048576);
  Serial.print("MB @");
  Serial.print(ESP.getFlashChipSpeed() / 1000000);
  Serial.print("MHz  mode=");
  switch (ESP.getFlashChipMode()) {
    case FM_QIO:  Serial.print("QIO");  break;
    case FM_QOUT: Serial.print("QOUT"); break;
    case FM_DIO:  Serial.print("DIO");  break;
    case FM_DOUT: Serial.print("DOUT"); break;
    default:      Serial.print("?");    break;
  }
  Serial.print("  PSRAM ");
  if (psramFound()) {
    Serial.print(ESP.getPsramSize() / 1048576);
    Serial.println("MB");
  } else {
    Serial.println("none");
  }
  if (ESP.getFlashChipSize() < 16 * 1048576)
    Serial.println("[CHIP] !! 플래시가 16MB 미만으로 인식됨 — IDE의 Flash Size/Mode 확인");
  if (!psramFound())
    Serial.println("[CHIP] !! PSRAM 미검출 — IDE의 PSRAM 설정(OPI PSRAM) 확인");

  logHeapInfo("boot");

  // ── 서보 ── (setup() 맨 앞 motor.begin() 로그는 USB 연결 전이라 놓치기 쉬워 다시 찍는다)
  {
    const ServoTelemetry t = motor.getTelemetry();
    Serial.print("[BOOT] servo      : ID "); Serial.print(SERVO_ID);
    Serial.print(" @"); Serial.print(SERVO_BAUD);
    Serial.print("bps TX"); Serial.print(PIN_SERVO_TX);
    Serial.print("/RX"); Serial.print(PIN_SERVO_RX);
    Serial.print(" -> "); Serial.println(motor.isDriverFault() ? "NOT READY" : "OK");
    Serial.print("[BOOT] servo      : model "); Serial.print(t.model);
    if (t.model && t.model != SERVO_MODEL_EXPECTED) Serial.print(" (STS3250=2825 아님!)");
    Serial.print(" mode "); Serial.print(t.mode);
    Serial.print(" echo "); Serial.print(t.echo ? "yes" : "no");
    Serial.print(" V "); Serial.print(t.voltageV, 1);
    Serial.print(" T "); Serial.print(t.tempC);
    Serial.print("C torqueLimit "); Serial.print(SERVO_TORQUE_LIMIT / 10.0f, 1);
    Serial.println("%");
    Serial.print("[BOOT] servo      : "); Serial.print(SERVO_STEPS_PER_MM, 1);
    Serial.print(" step/mm (lead "); Serial.print(LEAD_PITCH_MM, 1);
    Serial.print("mm), max "); Serial.print(SERVO_MAX_SPEED / SERVO_STEPS_PER_MM, 2);
    Serial.println("mm/s");
    if (motor.isDriverFault())
      Serial.println("[BOOT] !! 서보 응답 없음 — 9V 전원, 어댑터 점퍼, TX-TX/RX-RX, GND 공통, ID/보율 확인");
  }
  // ② 전류 측정 — 션트를 바꾸면 임계값도 함께 재검토
  Serial.print("[BOOT] INA228     : BAT 0x");
  Serial.print(INA228_ADDR_BAT, HEX);
  Serial.println("(배터리 전체 전류, 전원 분기 앞)");
  // I2C 버스 점검: 선 상태 → 주소 스캔 → INA228 ID 레지스터
  {
    const int sda = gpio_get_level((gpio_num_t)PIN_I2C_SDA);
    const int scl = gpio_get_level((gpio_num_t)PIN_I2C_SCL);
    Serial.printf("[BOOT] I2C        : SDA=GPIO%u(%s) SCL=GPIO%u(%s) idle\r\n",
                  PIN_I2C_SDA, sda ? "HIGH" : "LOW!", PIN_I2C_SCL, scl ? "HIGH" : "LOW!");
    if (!sda || !scl)
      Serial.println("[BOOT] !! I2C 선이 LOW — 풀업 없음, 모듈 전원(VS 3V3/GND) 없음, 또는 단락");
    uint8_t found = 0;
    Serial.print("[BOOT] I2C scan   :");
    for (uint8_t a = 1; a < 127; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); ++found; }
    }
    Serial.println(found ? "" : " 응답 없음 — SDA/SCL 바뀜? VS·GND? 풀업?");
    // INA228_ADDR_BAT가 응답하면 ID 레지스터를 직접 읽어 칩 종류를 확인한다.
    auto rd16 = [](uint8_t reg, uint16_t &v) -> bool {
      Wire.beginTransmission(INA228_ADDR_BAT); Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return false;
      if (Wire.requestFrom((uint8_t)INA228_ADDR_BAT, (uint8_t)2) != 2) return false;
      v = (uint16_t)(Wire.read() << 8); v |= (uint16_t)Wire.read(); return true;
    };
    uint16_t man = 0, dev = 0;
    if (rd16(0x3E, man) && rd16(0x3F, dev)) {
      Serial.printf("[BOOT] INA ID     : manufacturer 0x%04X  device 0x%04X  -> %s\r\n", man, dev,
                    (man == 0x5449 && (dev >> 4) == 0x228) ? "INA228 OK" :
                    (dev == 0x2260) ? "INA226 (INA228 아님)" : "INA228 아님/읽기 오류");
    } else {
      Serial.printf("[BOOT] INA ID     : 0x%02X 읽기 실패\r\n", INA228_ADDR_BAT);
    }
  }
  Serial.print("[BOOT] current    : shunt ");
  Serial.print(INA228_SHUNT_OHM, 4);
  Serial.print("ohm -> range ");
  Serial.print(INA228_SATURATION_MA, 0);
  Serial.print("mA / OC trip ");
  Serial.print(OVERCURRENT_MA, 0);
  Serial.print("mA after ");
  Serial.print(OVERCURRENT_HOLD_MS);
  Serial.println("ms");

  // ③ 배터리 보정 — 배선을 바꾸면 반드시 재측정해야 하는 값
  Serial.print("[BOOT] battery     : offset +");
  Serial.print(BAT_SENSE_OFFSET_V, 3);
  Serial.print("V / full ");
  Serial.print(BAT_FULL_V, 2);
  Serial.print("V / load cutoff ");
  Serial.print(BAT_LOAD_CUTOFF_V, 2);
  Serial.println("V");

  Serial.println("----------------------------------------");
  _lastLoopUs   = micros();
  _lastReportMs = millis();
}

// ══════════════════════════════════════════════════════════
void diagLoopTick() {
  unsigned long nowUs = micros();
  unsigned long dt    = nowUs - _lastLoopUs;
  _lastLoopUs = nowUs;
  if (dt > 2000000UL) return;   // 첫 호출/오버플로 보호

  _loopSumUs += dt;
  _loopCount++;
  if (dt > _loopMaxUs)     _loopMaxUs = dt;
  if (dt > _loopWorstEver) _loopWorstEver = dt;
}

// ══════════════════════════════════════════════════════════
void diagSample(float vBus, float iMa, bool motorActive) {
  if (motorActive) {
    if (vBus < _vMinLoad) _vMinLoad = vBus;
    if (iMa   > _iMaxLoad) _iMaxLoad = iMa;

    // ① 브라운아웃 임박 경고. 1.0V 미만은 센서 없음(의미 없는 값)으로 보고 건너뛴다.
    constexpr float SENSOR_PLAUSIBLE_MIN_V = 1.0f;
    if (vBus >= SENSOR_PLAUSIBLE_MIN_V &&
        vBus < DIAG_VSAG_WARN_V && !_sagWarned) {
      _sagWarned = true;
      Serial.print("[DIAG] !! 전압 급강하 ");
      Serial.print(vBus, 3);
      Serial.print("V (부하 ");
      Serial.print(iMa, 0);
      Serial.println("mA) — 브라운아웃 임박, 배선 저항 의심");
    }

    // ② 경로 저항 추정: R = ΔV / ΔI
    //    무부하 기준점이 잡혀 있고 전류 차이가 충분할 때만 계산.
    if (_vIdle > 0 && (_iMaxLoad - _iIdle) > 100.0f) {
      float r = (_vIdle - _vMinLoad) / ((_iMaxLoad - _iIdle) / 1000.0f);
      if (r > 0 && r < 20.0f) _pathROhm = r;
    }
  } else {
    // 무부하 기준점 갱신 (모터 정지 중 값)
    _vIdle = vBus;
    _iIdle = iMa;
  }
}

// ══════════════════════════════════════════════════════════
void diagReport(SystemState st, bool motorActive) {
  unsigned long now = millis();
  unsigned long period = motorActive ? DIAG_REPORT_BUSY_MS : DIAG_REPORT_IDLE_MS;
  bool force = telnetLogNewClient();   // 새 Telnet 연결 시 즉시 찍기
  if (!force && now - _lastReportMs < period) return;
  _lastReportMs = now;

  unsigned long avgUs = _loopCount ? (_loopSumUs / _loopCount) : 0;

  Serial.print("[DIAG] ");
  Serial.print(stateName(st));
  Serial.print(" loop avg/max ");
  Serial.print(avgUs / 1000.0f, 1);
  Serial.print("/");
  Serial.print(_loopMaxUs / 1000.0f, 1);
  Serial.print("ms");

  Serial.print(" | pos ");
  Serial.print(motor.getPositionMm(), 2);
  Serial.print("mm faultCode="); Serial.print(motor.getFaultCode());

  // 서보 엔코더 실측 위치(step)와 서보 상태
  Serial.print(" | steps ");
  Serial.print(motor.getStepCount());
  {
    const ServoTelemetry t = motor.getTelemetry();
    Serial.print(" | servo ");
    Serial.print(t.online ? "on" : "OFFLINE");
    Serial.print(" "); Serial.print(t.voltageV, 1);
    Serial.print("V "); Serial.print(t.tempC);
    Serial.print("C load "); Serial.print(t.loadPermil / 10.0f, 1);
    Serial.print("% st 0x"); Serial.print(t.status, HEX);
    Serial.print(" commErr "); Serial.print(t.commErrors);
    Serial.print("/retry "); Serial.print(t.commRetries);
  }

  Serial.print(" | V ");
  Serial.print(_vIdle, 2);
  if (_vMinLoad < 99.0f) {
    Serial.print("/min ");
    Serial.print(_vMinLoad, 2);
  }
  Serial.print(" I ");
  Serial.print(_iMaxLoad, 0);
  Serial.print("mA");
  if (motor.isCurrentSaturated()) Serial.print("(SAT)");

  if (_pathROhm > 0) {
    Serial.print(" | pathR ");
    Serial.print(_pathROhm, 2);
    Serial.print("ohm");
  }
  Serial.println();

  // ── 경고들 ──
  // ④ 루프 지연 → 도달 판정 스킵 / 인코더 폴링 유실 위험
  if (_loopMaxUs / 1000UL > DIAG_LOOP_WARN_MS) {
    Serial.print("[DIAG] !! 루프 지연 ");
    Serial.print(_loopMaxUs / 1000.0f, 1);
    Serial.println("ms — 도달 판정 스킵/EC11 유실 위험 (화면 재드로우 확인)");
  }
  // ② 경로 저항 과다 → ①의 근본 원인
  if (_pathROhm > DIAG_PATH_R_WARN_OHM) {
    Serial.print("[DIAG] !! 경로 저항 ");
    Serial.print(_pathROhm, 2);
    Serial.println("ohm — 정상 기대치 0.15ohm. 홀더 접점/커넥터/배선 점검");
  }

  // 창(window) 통계 초기화 — 다음 구간을 독립적으로 보기 위함
  _loopMaxUs = 0; _loopSumUs = 0; _loopCount = 0;
  _sagWarned = false;
  if (!motorActive) { _vMinLoad = 99.0f; _iMaxLoad = 0; }
}

// ══════════════════════════════════════════════════════════
void diagState(SystemState from, SystemState to, StopReason reason) {
  if (from == to) return;
  Serial.print("[STATE] ");
  Serial.print(stateName(from));
  Serial.print(" -> ");
  Serial.print(stateName(to));
  if (to == STATE_STOPPED) {
    Serial.print("  reason=");
    Serial.print(stopReasonName(reason));
    // 정지 시점의 맥락을 함께 남겨 로그만으로 원인을 되짚을 수 있게 한다
    Serial.print("  pos=");
    Serial.print(motor.getPositionMm(), 2);
    Serial.print("mm  vMin=");
    Serial.print(_vMinLoad < 99.0f ? _vMinLoad : _vIdle, 2);
    Serial.print("V  iMax=");
    Serial.print(_iMaxLoad, 0);
    Serial.print("mA  loopWorst=");
    Serial.print(_loopWorstEver / 1000.0f, 1);
    Serial.print("ms  steps=");
    Serial.print(motor.getStepCount());
    Serial.print("  faultCode=");
    Serial.print(motor.getFaultCode());
  }
  Serial.println();

  // 이동을 새로 시작하면 부하 통계를 리셋해 구간별로 볼 수 있게 함
  if (to == STATE_ALIGNING || to == STATE_RUNNING ||
      to == STATE_HOMING   || to == STATE_JOG_FWD) {
    _vMinLoad = 99.0f;
    _iMaxLoad = 0;
    _sagWarned = false;
  }
}
