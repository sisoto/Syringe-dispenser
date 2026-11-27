#include "Motor.h"
#include "PinChecks.h"
#include "ServoProfile.h"
#include "StsProtocol.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us — ROM 함수라 ISR에서 안전
#include "soc/gpio_reg.h"  // GPIO_IN_REG — ISR에서 핀 레벨을 직접 읽기
#include "soc/soc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "TelnetLog.h"

// ── Serial 미러링 ─────────────────────────────────────────
// 반드시 모든 #include 뒤에 둘 것 (TelnetLog.h 주석 참고).
// 로그는 메인 루프(service())에서만 찍는다 — 서보 태스크는 다른 코어에서
// 돌기 때문에 거기서 출력하면 메인 루프 출력과 섞이거나 충돌한다.
#define Serial LOG

Motor motor;

namespace {

// ══════════════════════════════════════════════════════════
// 공용 정의
// ══════════════════════════════════════════════════════════
enum Guard : uint8_t { NORMAL, SEEK_HOME, RELEASE_HOME, JOG };
// 정지 사유. getFaultCode()는 이 값을 돌려주되 8=서보 고장, 9=과전류를 우선한다
// (Injection.ino는 3=KO를 사용자 정지로 본다).
enum Halt : uint8_t { HALT_OK, HOME_HIT, HOME_RELEASED, KO, LIMIT, SOFT, TIMING, FAILED,
                      HALT_STALL = 10 };
enum Mode : uint8_t { MODE_IDLE, MODE_PROFILE, MODE_VELOCITY, MODE_BRAKE };

// 서보 태스크 → 메인 루프 로그 이벤트
enum : uint32_t {
  EV_COMM_LOST  = 1u << 0,
  EV_COMM_BACK  = 1u << 1,
  EV_SERVO_ERR  = 1u << 2,
  EV_STALL      = 1u << 3,
  EV_DIR_WRONG  = 1u << 4,
  EV_GAP        = 1u << 5,
  EV_HEARTBEAT  = 1u << 6,
  EV_HALT       = 1u << 7,
  EV_WRONG_WAY  = 1u << 8,
};

constexpr double SPM        = SERVO_STEPS_PER_MM;
constexpr double SOFT_MAX   = (MAX_TRAVEL_MM - SOFT_LIMIT_MARGIN) * SERVO_STEPS_PER_MM;
constexpr double SOFT_MIN   = -SOFT_LIMIT_LOW_TOL_MM * SERVO_STEPS_PER_MM;
constexpr double FOLLOW_ERR = SERVO_FOLLOW_ERR_MM * SERVO_STEPS_PER_MM;
constexpr double FOLLOW_ERR_HARD = SERVO_FOLLOW_ERR_HARD_MM * SERVO_STEPS_PER_MM;
constexpr double STALL_LOAD = SERVO_STALL_LOAD_FRAC * SERVO_TORQUE_LIMIT;   // 0~1000 단위
constexpr double FRONT_END_TOL = FRONT_END_TOL_MM * SERVO_STEPS_PER_MM;
constexpr long   DONE_TOL   = (long)(SERVO_DONE_TOL_MM * SERVO_STEPS_PER_MM + 0.5f);
constexpr int    CMD_SIGN   = SERVO_DIR_INVERT ? -1 : 1;                 // 피스톤 좌표 → 서보 속도
constexpr int    FB_SIGN    = CMD_SIGN * (SERVO_FEEDBACK_INVERT ? -1 : 1); // 서보 위치 → 피스톤 좌표
constexpr int    HALF_REV   = sts::STEPS_PER_REV / 2;

HardwareSerial servoSerial(SERVO_UART_NUM);
SemaphoreHandle_t busMutex = nullptr;
portMUX_TYPE ctlMux = portMUX_INITIALIZER_UNLOCKED;

// ── 메인 루프와 서보 태스크가 공유하는 상태 (ctlMux로 보호) ──
struct Ctl {
  // 명령
  uint8_t mode = MODE_IDLE, guard = NORMAL;
  bool    active = false;      // 이동 명령 수행 중
  bool    started = false;     // 태스크가 현재 명령을 받아들였는가
  bool    brakeInit = false;
  ServoProfile plan;
  double  planStart = 0;       // 원점 기준 step
  double  velTarget = 0, velStart = 0, velMaxDist = 0;
  bool    wantTorque = true;
  bool    resync = true;       // 다음 샘플을 새 기준으로(펼침 이어붙이지 않음)
  // 상태
  bool    origin = false;
  bool    endOnFront = false;  // 이번 궤적(주입)은 끝 무렵 FRONT 리밋을 "완료"로 본다
  double  endTol = 0;          // 그 "끝 무렵" 범위(step) — 주입 시작 때 정함
  bool    frontEnd = false;    // 실제로 FRONT 리밋에서 완료 처리됨
  double  frontPressPos = 0;   // FRONT가 눌린 순간의 위치(step, 원점 기준) — 끝 위치 학습용
  uint8_t halt = HALT_OK;
  bool    driverFault = true;  // configureServo() 성공 전까지 true
  bool    dirFault = false;    // 피드백 방향 불일치 — 재부팅 전까지 유지
  bool    online = false, torqueOn = false;
  int32_t abs = 0;             // 부팅 후 누적 위치(피스톤 좌표, step)
  int32_t zero = 0;            // 원점의 abs
  double  ref = 0, refVel = 0, cmd = 0, meas = 0;
  uint64_t segUs = 0;          // 현재 구간 유효 경과(일시정지 대기 제외)
  int64_t heartbeatUs = 0;
  // 텔레메트리
  uint8_t voltRaw = 0, temp = 0, status = 0;
  int16_t load = 0, curRaw = 0;
  uint32_t commErrors = 0, commRetries = 0;
  // 로그 이벤트
  uint32_t ev = 0;
  uint8_t evHalt = 0, evErrBits = 0;
  float   evA = 0, evB = 0;
  int16_t evLoad = 0; bool evHard = false;   // 막힘 시점 서보 출력(‰), 안전망(②)으로 걸렸는지
};
Ctl ctl;

// ── 서보 태스크 전용 (락 불필요) ──
struct TaskLocal {
  bool     haveRaw = false;
  int      lastRaw = 0;
  int64_t  lastTickUs = 0, lastSampleUs = 0, lastWriteUs = 0;
  int64_t  followSinceUs = 0, hardFollowSinceUs = 0, stallSinceUs = 0, settleSinceUs = 0;
  int32_t  lastWritten = 0x7FFFFFFF;
  uint8_t  commFails = 0, errCount = 0;
  double   dirAccum = 0;     // 정속 이동 중 명령 방향 기준 누적 이동량
  double   integ = 0;        // 궤적 추종 적분 보정(step/s)
  double   corrF = 0;        // 저역 필터를 거친 보정량(step/s) — 소음 저감
  bool     homeAtStart = false, frontAtStart = false;
};
TaskLocal tl;

// INA228 ALERT 래치 — ISR이 세우고, 서보 태스크·메인 루프가 읽는다.
// clearFaults()가 ALERT 핀이 HIGH로 돌아온 것을 확인한 뒤에만 내린다.
volatile bool g_ocAlert = false;
volatile uint32_t g_ocAlertGlitch = 0;   // 확인 단계에서 걸러진 ALERT 잡음 횟수(로그용)

// ── 버스 상태 (busMutex로 보호) ──
bool     g_echo = false;
bool     g_writeReplies = true;
uint16_t g_model = 0;
uint8_t  g_mode = 0xFF;

inline bool limitActive(uint8_t pin) {
  return digitalRead(pin) == (LIMIT_ACTIVE_HIGH ? HIGH : LOW);
}
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int sgn(double v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

// ══════════════════════════════════════════════════════════
// 버스 입출력 (busMutex를 잡은 상태에서만 호출)
// ══════════════════════════════════════════════════════════
void drainRx() { while (servoSerial.available() > 0) servoSerial.read(); }

// 요청을 보내고 응답 파라미터를 out에 받는다.
// 어댑터 에코가 있으면 보낸 바이트를 먼저 그대로 돌려받아 확인한다.
bool transact(const uint8_t *pkt, size_t n, bool wantReply,
              uint8_t *out, uint8_t outLen, uint8_t *err) {
  drainRx();
  servoSerial.write(pkt, n);
  if (!wantReply && !g_echo) return true;   // 응답 없는 쓰기: 기다리지 않는다
  sts::Parser p;
  size_t echoed = 0;
  const uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < SERVO_REPLY_TIMEOUT_US) {
    const int c = servoSerial.read();
    if (c < 0) continue;
    if (g_echo && echoed < n) {
      if ((uint8_t)c != pkt[echoed]) return false;
      if (++echoed == n && !wantReply) return true;
      continue;
    }
    if (!wantReply) continue;
    if (p.push((uint8_t)c) != sts::Parser::PACKET) continue;
    if (p.id() != pkt[2] || p.paramCount() != outLen) continue; // 다른 패킷/요청 에코
    if (err) *err = p.error();
    for (uint8_t i = 0; i < outLen; ++i) out[i] = p.params()[i];
    return true;
  }
  return !wantReply && (!g_echo || echoed == n);
}

bool busPing(uint8_t *err = nullptr) {
  uint8_t pkt[8];
  const size_t n = sts::buildPing(pkt, sizeof(pkt), SERVO_ID);
  return transact(pkt, n, true, nullptr, 0, err);
}
bool busRead(uint8_t addr, uint8_t len, uint8_t *out, uint8_t *err = nullptr) {
  uint8_t pkt[8];
  const size_t n = sts::buildRead(pkt, sizeof(pkt), SERVO_ID, addr, len);
  return transact(pkt, n, true, out, len, err);
}
bool busWrite(uint8_t addr, const uint8_t *data, uint8_t len, bool forceReply = false) {
  uint8_t pkt[sts::MAX_PACKET];
  const size_t n = sts::buildWrite(pkt, sizeof(pkt), SERVO_ID, addr, data, len);
  uint8_t e = 0;
  return n && transact(pkt, n, forceReply || g_writeReplies, nullptr, 0, &e);
}
bool busWrite8(uint8_t addr, uint8_t v) { return busWrite(addr, &v, 1); }
bool busWrite16(uint8_t addr, uint16_t v) {
  uint8_t b[2]; sts::putLe16(b, v); return busWrite(addr, b, 2);
}
// 라이브러리 WriteSpe()와 같은 형식: ACC, 위치(0), 시간(0), 속도를 한 번에 쓴다.
bool busWriteSpeed(int32_t servoStepsPerSec) {
  uint8_t b[7] = {SERVO_ACC_REG, 0, 0, 0, 0, 0, 0};
  sts::putLe16(b + 5, sts::encodeSigned15(servoStepsPerSec));
  return busWrite(sts::REG_ACC, b, 7);
}

// 목표 속도 레지스터만 쓴다(목표 위치·시간은 건드리지 않음). 서보가 아직
// 위치 모드일 수 있는 시점(전원 직후·재연결)의 정지 명령은 반드시 이것을 쓴다.
// busWriteSpeed()는 ACC~속도 7바이트를 한 번에 써서 목표 위치를 0으로 덮으므로,
// 위치 모드에서는 서보가 각도 0으로 튀어 버린다(전원 켤 때 잠깐 도는 원인).
bool busWriteGoalSpeed(int32_t servoStepsPerSec) {
  uint8_t b[2];
  sts::putLe16(b, sts::encodeSigned15(servoStepsPerSec));
  return busWrite(sts::REG_GOAL_SPEED, b, 2);
}

// 어댑터가 TX를 RX로 되돌려주는지 PING으로 판별
bool detectEcho() {
  if (SERVO_ECHO >= 0) { g_echo = SERVO_ECHO == 1; return true; }
  uint8_t pkt[8];
  const size_t n = sts::buildPing(pkt, sizeof(pkt), SERVO_ID);
  drainRx();
  servoSerial.write(pkt, n);
  uint8_t rx[24]; size_t got = 0;
  const uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < SERVO_REPLY_TIMEOUT_US && got < sizeof(rx)) {
    const int c = servoSerial.read();
    if (c >= 0) rx[got++] = (uint8_t)c;
  }
  if (got < n) { g_echo = false; return got > 0; }
  bool same = true;
  for (size_t i = 0; i < n; ++i) same &= rx[i] == pkt[i];
  // 응답 오류바이트가 0x01(=PING 명령 코드)인 드문 경우를 빼면 에코와 응답은
  // 체크섬이 달라 구분된다.
  g_echo = same;
  return true;
}

// ══════════════════════════════════════════════════════════
// 서보 태스크 (SERVO_TASK_CORE, 10ms)
// ══════════════════════════════════════════════════════════
// ctlMux 안에서 호출. 정지 사유를 기록하고 즉시 속도 0을 명령한다.
void haltLocked(uint8_t reason) {
  ctl.active = false; ctl.mode = MODE_IDLE; ctl.cmd = 0; ctl.refVel = 0;
  ctl.halt = reason;
  // 엔코더가 실제 위치를 계속 알고 있으므로 KO·소프트리밋·막힘에서는
  // 원점을 유지한다. 리밋 스위치를 예상 밖에 밟았거나(기하 불일치)
  // 피드백이 끊긴 경우만 원점을 버린다.
  if (reason == LIMIT || reason == TIMING || (reason == FAILED && !ctl.online)) ctl.origin = false;
  if (reason >= KO) { ctl.ev |= EV_HALT; ctl.evHalt = reason; }
}
void finishLocked() { ctl.active = false; ctl.mode = MODE_IDLE; ctl.cmd = 0; ctl.refVel = 0; }

void servoTick() {
  if (!busMutex || xSemaphoreTake(busMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  const bool ocAlert = g_ocAlert;

  uint8_t fb[sts::FEEDBACK_LEN];
  uint8_t err = 0;
  // 응답 하나가 유실돼도(잡음·지연) 같은 틱 안에서 한 번 더 읽는다.
  // 연속 실패 판정(SERVO_COMM_FAIL_LIMIT)은 재시도까지 실패한 틱만 센다.
  bool ok = busRead(sts::FEEDBACK_FIRST, sts::FEEDBACK_LEN, fb, &err);
  bool retried = false;
  // 오프라인(서보 전원 꺼짐 등)일 때는 재시도하지 않는다 — 코어 1에서 도는 이 태스크가
  // 응답 대기(최대 3ms)로 loop()/화면 갱신 시간을 더 빼앗지 않게.
  if (!ok && ctl.online) { retried = true; ok = busRead(sts::FEEDBACK_FIRST, sts::FEEDBACK_LEN, fb, &err); }
  const int64_t now = esp_timer_get_time();
  const bool home = limitActive(PIN_LIMIT_HOME);
  const bool front = limitActive(PIN_LIMIT_FRNT);
  const bool koPressed = digitalRead(PIN_EC11_KO) == LOW;

  bool writeTorque = false, torqueVal = false;
  int32_t servoCmd = 0;
  bool doWrite = false;

  portENTER_CRITICAL(&ctlMux);
  if (retried) ++ctl.commRetries;
  const double dt = tl.lastTickUs ? (now - tl.lastTickUs) / 1e6 : 0.0;
  tl.lastTickUs = now;

  // ── 1) 피드백: 0~4095 각도를 다회전 위치로 펼친다 ──
  if (ok) {
    const int raw = ((sts::decodeSigned15(sts::le16(fb + 0)) % 4096) + 4096) % 4096;
    // 피드백이 오래 끊겼다가 돌아오면(서보 전원 차단 등) 그동안의 회전을
    // 알 수 없으므로 이어붙이지 않고 원점을 버린다(재호밍 필요).
    const bool longGap = tl.haveRaw && tl.lastSampleUs &&
                         now - tl.lastSampleUs > (int64_t)SERVO_FEEDBACK_GAP_MS * 1000;
    if (longGap) ctl.origin = false;
    if (!tl.haveRaw || ctl.resync || longGap) {
      tl.lastRaw = raw; tl.haveRaw = true; ctl.resync = false;
    } else {
      int d = raw - tl.lastRaw;
      if (d > HALF_REV) d -= sts::STEPS_PER_REV; else if (d < -HALF_REV) d += sts::STEPS_PER_REV;
      tl.lastRaw = raw;
      ctl.abs += FB_SIGN * d;
      const double sdt = tl.lastSampleUs ? (now - tl.lastSampleUs) / 1e6 : 0.0;
      if (sdt > 0) ctl.meas = 0.6 * ctl.meas + 0.4 * (FB_SIGN * d) / sdt;
      if (ctl.active && ctl.mode == MODE_VELOCITY) tl.dirAccum += sgn(ctl.cmd) * (FB_SIGN * d);
    }
    tl.lastSampleUs = now;
    if (!ctl.online && ctl.commErrors) ctl.ev |= EV_COMM_BACK;
    ctl.online = true; tl.commFails = 0;
    ctl.voltRaw = fb[6]; ctl.temp = fb[7]; ctl.status = fb[9];
    ctl.load = (int16_t)sts::decodeLoad(sts::le16(fb + 4));
    ctl.curRaw = (int16_t)sts::decodeSigned15(sts::le16(fb + 13));
    const uint8_t bits = (uint8_t)((err | fb[9]) & SERVO_FAULT_MASK);
    if (bits) {
      if (++tl.errCount >= SERVO_ERR_LIMIT && !ctl.driverFault) {
        ctl.driverFault = true; ctl.ev |= EV_SERVO_ERR; ctl.evErrBits = bits;
      }
    } else tl.errCount = 0;
  } else {
    ++ctl.commErrors;
    if (tl.commFails < 255) ++tl.commFails;
    if (tl.commFails >= SERVO_COMM_FAIL_LIMIT && ctl.online) {
      ctl.online = false; ctl.driverFault = true; ctl.ev |= EV_COMM_LOST;
    }
  }
  const double pos = (double)(ctl.abs - ctl.zero);

  // ── 1.5) INA228 하드웨어 과전류: 즉시 정지 + 토크 해제 ──
  if (ocAlert) {
    ctl.wantTorque = false;
    if (ctl.active) haltLocked(FAILED);
  }

  // ── 2) 명령 처리 ──
  if (ctl.active && !ctl.started) {
    ctl.started = true; ctl.segUs = 0; ctl.cmd = 0;
    tl.followSinceUs = tl.hardFollowSinceUs = tl.stallSinceUs = tl.settleSinceUs = 0; tl.dirAccum = 0; tl.integ = 0; tl.corrF = 0;
    tl.homeAtStart = home; tl.frontAtStart = front;
    if (ctl.mode == MODE_PROFILE) { ctl.ref = ctl.planStart; ctl.refVel = 0; }
  } else if (ctl.active && dt > 0 && dt < 1.0) {
    ctl.segUs += (uint64_t)(dt * 1e6);
  }

  if (ctl.active) {
    // 이동 의도 방향. 궤적 이동은 계획 거리 부호를 쓴다 — 명령 부호를 쓰면 끝에서 살짝
    // 지나쳐 뒤로 보정하는 순간(cmd<0) 눌린 전방 리밋을 "반대로 돌았다"로 오판한다.
    int dir = (ctl.mode == MODE_PROFILE && ctl.plan.dist != 0) ? sgn(ctl.plan.dist) : sgn(ctl.cmd);
    if (!dir) dir = ctl.mode == MODE_VELOCITY ? sgn(ctl.velTarget) : sgn(ctl.plan.dist);

    uint8_t h = HALT_OK;
    bool endFront = false;
    if (ctl.driverFault || ctl.dirFault)                                  h = FAILED;
    else if (now - tl.lastSampleUs > (int64_t)SERVO_FEEDBACK_GAP_MS * 1000) { h = TIMING; ctl.ev |= EV_GAP; }
    else if (now - ctl.heartbeatUs > (int64_t)SERVO_HEARTBEAT_MS * 1000)   { h = TIMING; ctl.ev |= EV_HEARTBEAT; }
    else if (koPressed)                                                    h = KO;
    else if (home && front)                                                h = LIMIT;
    // 뒤로 간다고 했는데 FRONT가 새로 눌림(또는 그 반대) = 서보 회전 방향이
    // 기구와 반대로 설치됨. 호밍 탐색 중에도 잡아낸다.
    else if ((dir < 0 && front && !tl.frontAtStart) ||
             (dir > 0 && home && !tl.homeAtStart && ctl.guard != RELEASE_HOME)) {
      h = LIMIT; ctl.ev |= EV_WRONG_WAY;
    }
    else if (ctl.guard == SEEK_HOME && home)                               h = HOME_HIT;
    else if (ctl.guard == RELEASE_HOME && !home)                           h = HOME_RELEASED;
    // 주입 끝 무렵 FRONT가 눌림 = 주사기를 끝까지 밂 → 리밋 정지가 아니라 정상 완료
    // (목표 도착 후 리밋을 찾아 정속으로 더 미는 중이면 거리와 무관하게 완료)
    else if (dir > 0 && front && ctl.endOnFront &&
             ((ctl.mode == MODE_PROFILE && (ctl.planStart + ctl.plan.dist) - pos <= ctl.endTol) ||
              ctl.mode == MODE_VELOCITY))                                  endFront = true;
    // NORMAL 이동은 "이동 중 새로 눌린" 리밋에서 멈춘다. 시작할 때 이미 눌려 있던
    // 스위치(예: 전방 리밋에서 주입을 마친 뒤 뒤로 정렬)는 반대 방향 이동을 막지 않는다.
    else if ((dir > 0 && front) || (dir < 0 && home) ||
             (ctl.guard == NORMAL && ((front && !tl.frontAtStart) || (home && !tl.homeAtStart)))) h = LIMIT;
    else if (ctl.origin && ctl.guard != SEEK_HOME && ctl.guard != RELEASE_HOME &&
             ((dir > 0 && pos >= SOFT_MAX) || (dir < 0 && pos <= SOFT_MIN))) h = SOFT;

    if (endFront) {
      finishLocked(); ctl.frontEnd = true; ctl.frontPressPos = pos;
    } else if (h != HALT_OK) {
      haltLocked(h);
    } else if (ctl.mode == MODE_PROFILE) {
      // ── 시간 기반 궤적 추종 ──
      const double t = ctl.segUs / 1e6;
      double off, v;
      ctl.plan.sample(t, off, v);
      ctl.ref = ctl.planStart + off; ctl.refVel = v;
      const double target = ctl.planStart + ctl.plan.dist;
      if (t < ctl.plan.total) {
        const double e = ctl.ref - pos;
        tl.integ = clampd(tl.integ + SERVO_KI * e * dt, -SERVO_MAX_CORR, SERVO_MAX_CORR);
        // 보정량을 저역 필터로 걸러 엔코더 잡음이 속도 명령을 흔들지 않게 한다(소음 저감)
        const double corr = clampd(SERVO_KP * e + tl.integ, -SERVO_MAX_CORR, SERVO_MAX_CORR);
        tl.corrF += (corr - tl.corrF) * fmin(1.0, dt / SERVO_CORR_TAU_S);
        ctl.cmd = v + tl.corrF;
        // ① 힘을 다 쓰는데(출력 포화) 거의 못 움직이며 뒤처짐 = 진짜 막힘
        const bool saturated = fabs((double)ctl.load) >= STALL_LOAD;
        const bool slow = fabs(ctl.meas) < 0.5 * fabs(v) + 0.05 * SPM;
        if (fabs(e) > FOLLOW_ERR && saturated && slow) {
          if (!tl.followSinceUs) tl.followSinceUs = now;
        } else tl.followSinceUs = 0;
        // ② 안전망: 출력과 무관하게 크게 뒤처짐
        if (fabs(e) > FOLLOW_ERR_HARD) {
          if (!tl.hardFollowSinceUs) tl.hardFollowSinceUs = now;
        } else tl.hardFollowSinceUs = 0;
        const bool stallA = tl.followSinceUs && now - tl.followSinceUs > (int64_t)SERVO_FOLLOW_ERR_MS * 1000;
        const bool stallB = tl.hardFollowSinceUs && now - tl.hardFollowSinceUs > (int64_t)SERVO_FOLLOW_ERR_HARD_MS * 1000;
        if (stallA || stallB) {
          ctl.ev |= EV_STALL; ctl.evA = (float)(e / SPM); ctl.evB = (float)ctl.meas;
          ctl.evLoad = ctl.load; ctl.evHard = stallB && !stallA;
          if (sgn(ctl.meas) == -sgn(v) && fabs(ctl.meas) > 100) ctl.ev |= EV_DIR_WRONG;
          haltLocked(HALT_STALL);
        }
      } else {
        // 계획 시간 종료 → 남은 오차를 천천히 메운다
        const double e = target - pos;
        if (fabs(e) <= SERVO_POS_TOL) finishLocked();
        else {
          tl.integ = clampd(tl.integ + SERVO_KI * e * dt, -SERVO_MAX_CORR, SERVO_MAX_CORR);
          double c = clampd(SERVO_KP * e + tl.integ, -SERVO_MAX_CORR, SERVO_MAX_CORR);
          if (fabs(c) < SERVO_MIN_SPEED) c = sgn(e) * SERVO_MIN_SPEED;
          ctl.cmd = c;
          if (!tl.settleSinceUs) tl.settleSinceUs = now;
          else if (now - tl.settleSinceUs > (int64_t)SERVO_SETTLE_MS * 1000) {
            if (fabs(e) <= DONE_TOL) finishLocked();
            else { ctl.ev |= EV_STALL; ctl.evA = (float)(e / SPM); ctl.evB = (float)ctl.meas;
                   ctl.evLoad = ctl.load; ctl.evHard = fabs((double)ctl.load) < STALL_LOAD;
                   haltLocked(HALT_STALL); }
          }
        }
      }
    } else if (ctl.mode == MODE_BRAKE) {
      // ── 일시정지: 현재 기준 속도에서 가속도 한계로 감속 후 정지 유지 ──
      if (!ctl.brakeInit) ctl.brakeInit = true;  // ref/refVel은 직전 궤적 값 그대로
      const double dv = SERVO_ACCEL * dt;
      if (ctl.refVel > 0) ctl.refVel = ctl.refVel > dv ? ctl.refVel - dv : 0;
      else if (ctl.refVel < 0) ctl.refVel = -ctl.refVel > dv ? ctl.refVel + dv : 0;
      ctl.ref += ctl.refVel * dt;
      ctl.cmd = ctl.refVel + clampd(SERVO_KP * (ctl.ref - pos), -SERVO_MAX_CORR, SERVO_MAX_CORR);
      if (ctl.refVel == 0) finishLocked();
    } else if (ctl.mode == MODE_VELOCITY) {
      // ── 정속 이동(호밍·백오프·조그): 가속도 한계로 목표 속도까지 ──
      const double dv = SERVO_ACCEL * dt;
      ctl.cmd = clampd(ctl.velTarget, ctl.cmd - dv, ctl.cmd + dv);
      ctl.refVel = ctl.cmd; ctl.ref = pos;
      if (fabs(pos - ctl.velStart) >= ctl.velMaxDist) {
        finishLocked();   // 최대 거리 도달 — 호밍이면 updateHoming()이 실패로 판정
      } else if (tl.dirAccum < -1.0 * SPM) {
        // 명령과 반대로 1mm 이상 움직임 = 피드백 부호가 뒤집힌 서보
        // (리밋 정지 직후 관성 0.2mm 정도는 여기에 걸리지 않는다)
        ctl.dirFault = true; ctl.ev |= EV_DIR_WRONG; haltLocked(FAILED);
      } else if (fabs(ctl.cmd) >= 0.99 * fabs(ctl.velTarget)) {
        if (fabs(ctl.meas) < SERVO_STALL_RATIO * fabs(ctl.cmd)) {
          if (!tl.stallSinceUs) tl.stallSinceUs = now;
          else if (now - tl.stallSinceUs > (int64_t)SERVO_STALL_MS * 1000) {
            ctl.ev |= EV_STALL; ctl.evA = 0; ctl.evB = (float)ctl.meas;
            haltLocked(HALT_STALL);
          }
        } else tl.stallSinceUs = 0;
      }
    }
    if (ctl.active) ctl.cmd = clampd(ctl.cmd, -SERVO_MAX_SPEED, SERVO_MAX_SPEED);
  } else {
    ctl.cmd = 0;
  }

  // ── 3) 출력 결정 ──
  servoCmd = (int32_t)lround(ctl.cmd) * CMD_SIGN;
  // 작은 변화(±SERVO_CMD_DEADBAND 미만)는 다시 쓰지 않는다 — 정지(0)·첫 명령·200ms 갱신은 항상 쓴다
  doWrite = tl.lastWritten == 0x7FFFFFFF ||
            (servoCmd == 0 && tl.lastWritten != 0) ||
            labs((long)servoCmd - (long)tl.lastWritten) >= SERVO_CMD_DEADBAND ||
            now - tl.lastWriteUs > 200000;
  if (ctl.wantTorque != ctl.torqueOn && ctl.online) { writeTorque = true; torqueVal = ctl.wantTorque; }
  // 설정(휠 모드) 확인 전이거나 끊겼다 돌아온 서보는 위치 모드일 수 있다 → 속도만 쓴다
  const bool wheelConfirmed = !ctl.driverFault;
  portEXIT_CRITICAL(&ctlMux);

  // ── 4) 버스 쓰기 (락 밖) ──
  if (writeTorque) {
    if (!torqueVal) busWriteGoalSpeed(0);
    if (busWrite8(sts::REG_TORQUE_ENABLE, torqueVal ? 1 : 0)) {
      portENTER_CRITICAL(&ctlMux); ctl.torqueOn = torqueVal; portEXIT_CRITICAL(&ctlMux);
    }
  }
  if (doWrite && (wheelConfirmed ? busWriteSpeed(servoCmd) : busWriteGoalSpeed(0))) {
    tl.lastWritten = wheelConfirmed ? servoCmd : 0; tl.lastWriteUs = now;
  }
  xSemaphoreGive(busMutex);
}

void servoTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    servoTick();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SERVO_PERIOD_MS));
  }
}

struct Snap {
  long pos; bool running, valid, driverFault, online; uint8_t halt; uint64_t segUs;
};
Snap snapshot() {
  portENTER_CRITICAL(&ctlMux);
  Snap s{(long)(ctl.abs - ctl.zero), ctl.active, ctl.origin,
         ctl.driverFault || ctl.dirFault, ctl.online, ctl.halt, ctl.segUs};
  portEXIT_CRITICAL(&ctlMux);
  return s;
}

const char *haltName(uint8_t h) {
  switch (h) {
    case KO: return "KO"; case LIMIT: return "LIMIT"; case SOFT: return "SOFT_LIMIT";
    case TIMING: return "TIMING"; case FAILED: return "FAILED"; case HALT_STALL: return "STALL";
    default: return "-";
  }
}

} // namespace

// ══════════════════════════════════════════════════════════
// 초기화 / 설정
// ══════════════════════════════════════════════════════════
bool Motor::configureServo() {
  if (!busMutex || xSemaphoreTake(busMutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
  bool ok = false;
  for (uint8_t attempt = 0; attempt < 3 && !ok; ++attempt) {
    if (attempt) delay(20);
    detectEcho();
    ok = busPing();
  }
  uint8_t mode = 0xFF;
  if (ok) {
    uint8_t m[2];
    g_model = busRead(sts::REG_MODEL, 2, m) ? sts::le16(m) : 0;
    // 쓰기 응답 여부(서보의 응답 레벨 설정)를 한 번 확인해 둔다.
    uint8_t acc = SERVO_ACC_REG;
    g_writeReplies = busWrite(sts::REG_ACC, &acc, 1, true);
    // 정속(휠) 모드로. EEPROM 잠금 상태라 전원을 끄면 원래 모드로 돌아간다.
    if (busRead(sts::REG_MODE, 1, &mode) && mode != sts::MODE_WHEEL) {
      // 위치 모드에서 모드를 바꾸는 동안 움직이지 않게 토크를 잠깐 끈다
      // (여기서 busWriteSpeed를 쓰면 목표 위치 0이 써져 서보가 튄다).
      // 휠 모드 확인 뒤 아래에서 속도 0 → 토크 ON 순서로 다시 켠다.
      busWrite8(sts::REG_TORQUE_ENABLE, 0);
      busWrite8(sts::REG_MODE, sts::MODE_WHEEL);
      if (!busRead(sts::REG_MODE, 1, &mode)) mode = 0xFF;
      // 잠긴 상태의 쓰기를 무시하는 펌웨어라면 잠금을 잠깐 풀어 한 번만
      // 저장한다(다음 부팅부터는 이미 휠 모드라 다시 쓰지 않는다).
      if (mode != sts::MODE_WHEEL) {
        busWrite8(sts::REG_LOCK, 0);
        busWrite8(sts::REG_MODE, sts::MODE_WHEEL);
        busWrite8(sts::REG_LOCK, 1);
        if (!busRead(sts::REG_MODE, 1, &mode)) mode = 0xFF;
      }
    }
    g_mode = mode;
    ok = mode == sts::MODE_WHEEL &&
         busWrite16(sts::REG_TORQUE_LIMIT, SERVO_TORQUE_LIMIT) &&
         busWriteSpeed(0) &&
         busWrite8(sts::REG_TORQUE_ENABLE, 1);
    // 쓰기 응답을 끈 서보면 위 쓰기들은 확인 없이 true다. 마지막으로 읽어서 확인.
    uint8_t tq[2];
    if (ok && busRead(sts::REG_TORQUE_LIMIT, 2, tq)) ok = sts::le16(tq) == SERVO_TORQUE_LIMIT;
    else ok = false;
  }
  portENTER_CRITICAL(&ctlMux);
  if (ok) {
    if (!ctl.online) ctl.resync = true;   // 끊겼던 동안의 회전은 이어붙이지 않는다
    ctl.online = true; ctl.driverFault = false;
    ctl.wantTorque = true; ctl.torqueOn = true;
  } else {
    ctl.driverFault = true;
  }
  portEXIT_CRITICAL(&ctlMux);
  xSemaphoreGive(busMutex);
  return ok;
}

// ALERT 하강 에지 → 잠깐 뒤 핀이 아직 LOW인지 다시 확인한다.
// INA228은 ALATCH=1(래치)이라 진짜 과전류면 DIAG_ALRT를 읽기 전까지 LOW로 남는다.
// 배선에 튄 짧은 잡음(승압 모듈·서보 전류 변화가 ALERT 선에 유기)은 그 사이 HIGH로
// 돌아오므로 과전류로 오판하지 않는다.
void IRAM_ATTR Motor::currentAlertISR() {
  esp_rom_delay_us(INA228_ALERT_CONFIRM_US);
  const bool low = ((REG_READ(GPIO_IN_REG) >> PIN_INA_ALERT) & 1u) == 0;
  if (low) g_ocAlert = true; else g_ocAlertGlitch = g_ocAlertGlitch + 1;
}
bool Motor::isCurrentAlert() const { return g_ocAlert; }

void Motor::begin() {
  pinMode(PIN_INA_ALERT, INPUT_PULLUP);   // 인터럽트는 INA228 설정 뒤 Injection.ino가 붙인다
  pinMode(PIN_LIMIT_HOME, INPUT_PULLUP);
  pinMode(PIN_LIMIT_FRNT, INPUT_PULLUP);
  pinMode(PIN_EC11_KO, INPUT_PULLUP);
  if (!busMutex) busMutex = xSemaphoreCreateMutex();
  servoSerial.setRxBufferSize(256);   // begin() 전에 호출해야 적용됨
  servoSerial.begin(SERVO_BAUD, SERIAL_8N1, PIN_SERVO_RX, PIN_SERVO_TX);
  // ESP32만 리셋되고 서보 전원은 살아 있던 경우, 서보는 마지막 속도로 계속
  // 돌고 있을 수 있다. 무엇보다 먼저 정지 명령을 보낸다(응답 확인 없이 2회).
  if (busMutex && xSemaphoreTake(busMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    const bool savedReplies = g_writeReplies;
    g_writeReplies = false; g_echo = false;
    busWriteGoalSpeed(0); delayMicroseconds(300); busWriteGoalSpeed(0);   // 위치 모드여도 안 튐
    g_writeReplies = savedReplies;
    delay(2); drainRx();
    xSemaphoreGive(busMutex);
  }
  const bool ok = configureServo();
  Serial.print("[SERVO] init "); Serial.print(ok ? "OK" : "FAILED");
  Serial.print(" model="); Serial.print(g_model);
  Serial.print(" mode="); Serial.print(g_mode);
  Serial.print(" echo="); Serial.println(g_echo);
  static TaskHandle_t handle = nullptr;
  if (!handle) {
    portENTER_CRITICAL(&ctlMux); ctl.heartbeatUs = esp_timer_get_time(); portEXIT_CRITICAL(&ctlMux);
    // loop()(우선순위 1)보다 높은 우선순위로 SERVO_TASK_CORE에 고정
    xTaskCreatePinnedToCore(servoTask, "servo", 4096, nullptr, 5, &handle, SERVO_TASK_CORE);
  }
}

void Motor::service() {
  // INA228 ALERT → 소프트웨어 과전류 상태로 승격(화면 "Overcurrent")
  if (g_ocAlert && !_overCurrent) {
    _overCurrent = true;
    failMotion(true);
    Serial.print("[OC] INA228 ALERT(하드웨어) — 배터리 전류가 ");
    Serial.print(INA228_ALERT_MA, 0);
    Serial.print("mA를 넘어 차단 (서보 정지·토크 해제). 직전 측정 ");
    Serial.print(_currentMa, 0);
    Serial.print("mA, 최근 최대 ");
    Serial.print(_peakMa, 0);
    Serial.println("mA");
  }
  {
    static uint32_t seenGlitch = 0;
    const uint32_t g = g_ocAlertGlitch;
    if (g != seenGlitch) {
      seenGlitch = g;
      Serial.print("[OC] ALERT 잡음 무시 (누적 ");
      Serial.print(g);
      Serial.println("회) — 핀이 곧바로 HIGH로 돌아옴. 잦으면 ALERT 배선·풀업 확인");
    }
  }
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&ctlMux);
  ctl.heartbeatUs = now;
  const uint32_t ev = ctl.ev; ctl.ev = 0;
  const uint8_t evHalt = ctl.evHalt, evBits = ctl.evErrBits;
  const float evA = ctl.evA, evB = ctl.evB;
  const int16_t evLoad = ctl.evLoad; const bool evHard = ctl.evHard;
  const bool active = ctl.active;
  const double pos = ctl.abs - ctl.zero, ref = ctl.ref, cmd = ctl.cmd, meas = ctl.meas;
  const uint8_t volt = ctl.voltRaw, temp = ctl.temp;
  const int16_t load = ctl.load, cur = ctl.curRaw;
  portEXIT_CRITICAL(&ctlMux);

  if (ev & EV_COMM_LOST) Serial.println("[SERVO] !! 응답 없음 — 9V 전원, 어댑터 점퍼(A), TX/RX·GND 배선, ID/보율 확인");
  if (ev & EV_COMM_BACK) Serial.println("[SERVO] 응답 복구 (clearFaults 후 재설정 필요)");
  if (ev & EV_SERVO_ERR) { Serial.print("[SERVO] !! 서보 오류 비트 0x"); Serial.print(evBits, HEX);
                           Serial.println(" (1=전압 2=센서 4=과열 8=과전류 0x20=과부하)"); }
  if (ev & EV_GAP)       Serial.println("[SERVO] !! 구동 중 위치 피드백 공백 — 원점 무효");
  if (ev & EV_HEARTBEAT) Serial.println("[SERVO] !! loop()가 멈춰 서보를 정지시킴");
  if (ev & EV_STALL) {
    Serial.print("[SERVO] !! 막힘(STALL): 추종오차 "); Serial.print(evA, 2);
    Serial.print("mm, 실측속도 "); Serial.print(evB / SPM, 2);
    Serial.print("mm/s, 서보 출력 "); Serial.print(evLoad / 10.0f, 1);
    Serial.print("% (한계 "); Serial.print(SERVO_TORQUE_LIMIT / 10.0f, 1); Serial.print("%)");
    Serial.println(evHard ? " — 출력 여유가 있는데 크게 뒤처짐: 서보/피드백 이상 확인"
                          : " — 힘을 다 써도 못 밂: 바늘·주사기·기구 걸림, 또는 SERVO_TORQUE_LIMIT 부족");
  }
  if (ev & EV_WRONG_WAY)
    Serial.println("[SERVO] !! 이동 방향과 반대쪽 리밋이 눌림 — 서보가 반대로 돈다. Theme.h의 SERVO_DIR_INVERT를 바꿀 것");
  if (ev & EV_DIR_WRONG)
    Serial.println("[SERVO] !! 명령과 반대로 회전함 — Theme.h의 SERVO_FEEDBACK_INVERT를 바꿀 것");
  if ((ev & EV_HALT) && !(ev & (EV_STALL | EV_GAP | EV_HEARTBEAT | EV_WRONG_WAY))) {
    Serial.print("[SERVO] halt="); Serial.println(haltName(evHalt));
  }

  static unsigned long lastLog = 0;
  if (SERVO_LOG_MS && active && millis() - lastLog >= SERVO_LOG_MS) {
    lastLog = millis();
    Serial.print("[SERVO] pos="); Serial.print(pos / SPM, 3);
    Serial.print(" ref="); Serial.print(ref / SPM, 3);
    Serial.print(" err="); Serial.print((ref - pos) / SPM, 3);
    Serial.print("mm cmd="); Serial.print(cmd / SPM, 3);
    Serial.print(" meas="); Serial.print(meas / SPM, 3);
    Serial.print("mm/s load="); Serial.print(load / 10.0f, 1);
    Serial.print("% I="); Serial.print(cur * 6.5f, 0);
    Serial.print("mA V="); Serial.print(volt / 10.0f, 1);
    Serial.print(" T="); Serial.print(temp);
    // 배터리 쪽 전류(INA228): 지금 값과 지난 로그 이후 최대 — 과전류 기준과 비교용
    Serial.print(" Ibat="); Serial.print(_currentMa, 0);
    Serial.print("mA max="); Serial.print(_peakMa, 0); Serial.println("mA");
    _peakMa = 0;
  }
}

// ══════════════════════════════════════════════════════════
// 이동 시작 (메인 루프)
// ══════════════════════════════════════════════════════════
bool Motor::launchProfile(long target, double seconds, uint8_t guard) {
  if (_overCurrent || g_ocAlert || digitalRead(PIN_EC11_KO) == LOW) return false;
  portENTER_CRITICAL(&ctlMux);
  ServoProfile plan;
  const double start = ctl.abs - ctl.zero;
  const bool ok = !ctl.active && ctl.halt < KO && !ctl.driverFault && !ctl.dirFault &&
                  ServoProfile::make(target - start, seconds, SERVO_RAMP_SEC,
                                     SERVO_MAX_SPEED, SERVO_ACCEL, plan);
  if (ok) {
    ctl.mode = MODE_PROFILE; ctl.plan = plan; ctl.planStart = start; ctl.guard = guard;
    ctl.endOnFront = _dispenseLaunch; ctl.endTol = _frontEndTol; ctl.frontEnd = false;
    ctl.halt = HALT_OK; ctl.started = false; ctl.brakeInit = false; ctl.segUs = 0;
    ctl.heartbeatUs = esp_timer_get_time(); ctl.active = true;
  }
  portEXIT_CRITICAL(&ctlMux);
  return ok;
}

bool Motor::launchVelocity(float speedMmS, float maxDistMm, uint8_t guard, bool endOnFront) {
  if (_overCurrent || g_ocAlert || digitalRead(PIN_EC11_KO) == LOW || !(maxDistMm > 0)) return false;
  const double v = clampd(speedMmS * SPM, -SERVO_MAX_SPEED, SERVO_MAX_SPEED);
  portENTER_CRITICAL(&ctlMux);
  const bool ok = !ctl.active && ctl.halt < KO && !ctl.driverFault && !ctl.dirFault && v != 0;
  if (ok) {
    ctl.mode = MODE_VELOCITY; ctl.velTarget = v; ctl.velStart = ctl.abs - ctl.zero;
    ctl.endOnFront = endOnFront; ctl.frontEnd = false;
    ctl.velMaxDist = maxDistMm * SPM; ctl.guard = guard;
    ctl.halt = HALT_OK; ctl.started = false; ctl.segUs = 0;
    ctl.heartbeatUs = esp_timer_get_time(); ctl.active = true;
  }
  portEXIT_CRITICAL(&ctlMux);
  return ok;
}

void Motor::stop() {
  portENTER_CRITICAL(&ctlMux);
  if (ctl.active) finishLocked();   // 태스크가 다음 틱(≤10ms)에 속도 0을 보낸다
  portEXIT_CRITICAL(&ctlMux);
  _dispensing = false; _paused = false; _seeking = false;
}

void Motor::emergencyStop() {
  const Snap s = snapshot();
  const bool motionContext = s.running || _dispensing || s.halt == KO;
  stop();   // 토크는 유지 — 피스톤을 놓지 않는다
  // 대기 중 KO는 메뉴 취소일 뿐이므로 다음 정렬 요청을 막지 않는다.
  if (motionContext) {
    portENTER_CRITICAL(&ctlMux); ctl.halt = KO; portEXIT_CRITICAL(&ctlMux);
  }
}

void Motor::failMotion(bool electrical) {
  stop(); _incomplete = true;
  portENTER_CRITICAL(&ctlMux);
  if (ctl.halt < KO) ctl.halt = FAILED;   // 이미 기록된 원인(막힘 등)은 보존
  if (electrical) ctl.wantTorque = false; // 전기적 이상 → 토크 해제
  portEXIT_CRITICAL(&ctlMux);
}

void Motor::clearFaults() {
  const Snap s = snapshot();
  if (s.running || digitalRead(PIN_EC11_KO) == LOW ||
      !isfinite(_currentMa) || _currentMa >= OVERCURRENT_MA) return;
  // ALERT로 차단된 경우: INA228 래치는 메인 루프가 DIAG_ALRT를 읽어 이미
  // 풀었으므로, 핀이 HIGH(전류가 기준 아래)여야만 해제한다.
  if (g_ocAlert) {
    if (digitalRead(PIN_INA_ALERT) == LOW) return;
    g_ocAlert = false;
  }
  _overCurrent = false; _overCurrentMs = 0;
  bool needCfg;
  portENTER_CRITICAL(&ctlMux);
  needCfg = ctl.driverFault || !ctl.wantTorque || !ctl.torqueOn;
  const bool dirFault = ctl.dirFault;
  portEXIT_CRITICAL(&ctlMux);
  if (dirFault) return;                       // 상수 수정 + 재부팅이 필요한 고장
  if (needCfg && !configureServo()) return;   // 재연결·토크 재인가
  portENTER_CRITICAL(&ctlMux); ctl.halt = HALT_OK; portEXIT_CRITICAL(&ctlMux);
  _incomplete = false;
}

// ══════════════════════════════════════════════════════════
// 호밍
// ══════════════════════════════════════════════════════════
void Motor::startHoming() {
  stop(); clearFaults();
  portENTER_CRITICAL(&ctlMux); ctl.origin = false; portEXIT_CRITICAL(&ctlMux);
  _homing = HOMING_ERROR; _homeMs = millis(); _releaseMs = 0;
  if (isHomeLimit() && isFrontLimit()) return;
  if (isHomeLimit()) {
    if (launchVelocity(BACKOFF_SPEED_MM_S, HOME_BACKOFF_MAX_MM, RELEASE_HOME)) _homing = HOMING_BACKOFF;
  } else if (launchVelocity(-HOME_SPEED_MM_S, MAX_TRAVEL_MM + HOME_BACKOFF_MAX_MM, SEEK_HOME)) {
    _homing = HOMING_RUNNING;
  }
}

HomingState Motor::updateHoming() {
  if (_homing != HOMING_RUNNING && _homing != HOMING_BACKOFF) return _homing;
  const Snap s = snapshot();
  if (s.halt >= KO || s.driverFault || _overCurrent) { stop(); return _homing = HOMING_ERROR; }
  if (millis() - _homeMs > HOMING_TIMEOUT_MS) { stop(); return _homing = HOMING_TIMEOUT; }
  if (_homing == HOMING_RUNNING && !s.running) {
    if (s.halt != HOME_HIT) return _homing = HOMING_ERROR;
    if (!launchVelocity(BACKOFF_SPEED_MM_S, HOME_BACKOFF_MAX_MM, RELEASE_HOME)) return _homing = HOMING_ERROR;
    _homing = HOMING_BACKOFF;
  } else if (_homing == HOMING_BACKOFF && !s.running) {
    if (s.halt != HOME_RELEASED || isHomeLimit() || isFrontLimit()) return _homing = HOMING_ERROR;
    if (!_releaseMs) _releaseMs = millis();
    if (millis() - _releaseMs >= HOME_SETTLE_MS) {
      // 정지 후 정착한 실제 엔코더 위치를 원점으로
      portENTER_CRITICAL(&ctlMux);
      ctl.zero = ctl.abs; ctl.origin = true; ctl.halt = HALT_OK;
      portEXIT_CRITICAL(&ctlMux);
      _homing = HOMING_DONE;
    }
  }
  return _homing;
}

// ══════════════════════════════════════════════════════════
// 정렬 이동
// ══════════════════════════════════════════════════════════
void Motor::startMoveTo(float target) {
  _move = MOVE_ERROR;
  if (!isfinite(target) || target < 0 || target * SPM >= SOFT_MAX || !isOriginValid()) return;
  _moveStart = getStepCount(); _moveTarget = lroundf(target * SERVO_STEPS_PER_MM); _moveMs = millis();
  const long delta = _moveTarget - _moveStart;
  if (labs(delta) <= DONE_TOL) { _move = MOVE_DONE; return; }
  // 정속 구간이 MOVE_SPEED가 되도록: T = D/v + 램프
  const double seconds = labs(delta) / (MOVE_SPEED_MM_S * SPM) + SERVO_RAMP_SEC;
  if (launchProfile(_moveTarget, seconds, NORMAL)) _move = MOVE_RUNNING;
}

MoveState Motor::updateMoveTo() {
  if (_move != MOVE_RUNNING) return _move;
  const Snap s = snapshot();
  if (s.halt >= KO || s.driverFault || _overCurrent) { stop(); return _move = MOVE_ERROR; }
  if (millis() - _moveMs > MOVE_TIMEOUT_MS) { stop(); return _move = MOVE_TIMEOUT; }
  if (!s.running) _move = labs(s.pos - _moveTarget) <= DONE_TOL ? MOVE_DONE : MOVE_ERROR;
  return _move;
}

// ══════════════════════════════════════════════════════════
// 주입
// ══════════════════════════════════════════════════════════
void Motor::startDispense(float distance, float seconds, bool seekFront) {
  _incomplete = true; _dispensing = false; _paused = false; _complete = false; _endedAtFront = false;
  _seeking = false; _seekMissed = false; _seekFromStep = 0; _seekMm = 0; _seekFront = seekFront; _frontPressMm = NAN;
  if (!isfinite(distance) || !isfinite(seconds) || distance <= 0 || seconds <= 0 ||
      seconds > 3600 || distance > MAX_TRAVEL_MM || !isOriginValid()) return;
  _doseStart = getStepCount(); _doseTarget = _doseStart + lroundf(distance * SERVO_STEPS_PER_MM);
  _frontEndTol = fmax(FRONT_END_TOL, (double)(_doseTarget - _doseStart) * DISPENSE_COMPLETE_TOL_PCT / 100.0);
  if (_doseTarget >= SOFT_MAX) return;
  _dosePriorUs = 0; _doseTotalUs = (uint64_t)llround((double)seconds * 1e6);
  _dispenseLaunch = true;
  const bool ok = launchProfile(_doseTarget, seconds, NORMAL);
  _dispenseLaunch = false;
  if (ok) { _incomplete = false; _dispensing = true; }
}

bool Motor::updateDispense() {
  if (!_dispensing) return false;
  const Snap s = snapshot();
  if (s.halt >= KO || s.driverFault || _overCurrent) { failMotion(); return false; }
  portENTER_CRITICAL(&ctlMux); const bool frontEnd = ctl.frontEnd; const double pressPos = ctl.frontPressPos; portEXIT_CRITICAL(&ctlMux);
  // 목표 도착 후 리밋 찾기 단계: 리밋이 눌리거나(완료) 최대 거리에서 멈출 때까지
  if (_seeking) {
    if (s.running) return true;
    _seeking = false; _seekMissed = !frontEnd; _seekMm = (float)((s.pos - _seekFromStep) / SPM);
    _frontPressMm = frontEnd ? (float)(pressPos / SPM) : NAN;
    _dispensing = false; _paused = false; _complete = true; _endedAtFront = frontEnd; return false;
  }
  if (!s.running && (labs(s.pos - _doseTarget) <= DONE_TOL || frontEnd)) {
    // 목표에는 왔는데 전방 리밋이 아직 안 눌림 = 주사기에 아직 남아 있음 → 리밋까지 마저 민다
    // (소프트 리밋 앞 1mm까지만)
    const double seekMax = fmin((double)FRONT_SEEK_MAX_MM, (SOFT_MAX - s.pos) / SPM - 1.0);
    if (_seekFront && !frontEnd && !isFrontLimit() && s.valid && seekMax > 0.1) {
      _seekFromStep = s.pos;
      // 새 이동이 시작되면 구간 시간(segUs)이 0으로 돌아가므로, 지금까지의 주입 시간을 먼저 누적
      const uint64_t seg = s.segUs;
      if (launchVelocity(FRONT_SEEK_SPEED_MM_S, (float)seekMax, NORMAL, true)) {
        _dosePriorUs += seg; _seeking = true; return true;
      }
    }
    _endedAtFront = frontEnd || isFrontLimit();
    _frontPressMm = frontEnd ? (float)(pressPos / SPM) : (_endedAtFront ? (float)(s.pos / SPM) : NAN);
    _dispensing = false; _paused = false; _complete = true; return false;
  }
  if (_paused) return true;
  if (!s.running) { _incomplete = true; _dispensing = false; return false; }
  return true;
}

void Motor::pauseDispense() {
  if (!_dispensing || _paused || _seeking) return;   // 리밋 찾기(마지막 몇 mm)는 일시정지하지 않음
  _paused = true; _pauseMs = millis();
  portENTER_CRITICAL(&ctlMux);
  if (ctl.active && ctl.mode == MODE_PROFILE) { ctl.mode = MODE_BRAKE; ctl.brakeInit = false; }
  portEXIT_CRITICAL(&ctlMux);
}

void Motor::resumeDispense() {
  if (!_dispensing || !_paused) return;
  const Snap s = snapshot();
  if (s.running) return; // 감속이 끝날 때까지 UI가 FWD 유지 중 재시도한다
  if (labs(s.pos - _doseTarget) <= DONE_TOL) { _paused = false; return; }
  const uint64_t used = _dosePriorUs + s.segUs;
  if (used >= _doseTotalUs || s.halt >= KO || !s.valid) { failMotion(); return; }
  const double remainSec = (double)(_doseTotalUs - used) / 1e6;
  _dispenseLaunch = true;
  const bool ok = launchProfile(_doseTarget, remainSec, NORMAL);
  _dispenseLaunch = false;
  if (!ok) { failMotion(); return; }
  _dosePriorUs = used; _paused = false;
}

void Motor::jogForward() {
  if (hasMotionFault() || isFrontLimit()) return;
  const float distMm = isOriginValid() ? (float)((SOFT_MAX - getStepCount() - 1) / SPM) : MAX_TRAVEL_MM;
  if (distMm > 0 && !launchVelocity(JOG_SPEED_MM_S, distMm, JOG)) failMotion();
}

// ══════════════════════════════════════════════════════════
// 상태 조회
// ══════════════════════════════════════════════════════════
bool Motor::isFrontLimit() const { return limitActive(PIN_LIMIT_FRNT); }
bool Motor::frontLimitIsDispenseEnd() const {
  if (_complete && _endedAtFront) return true;
  if (!_dispensing) return false;
  return (double)(_doseTarget - getStepCount()) <= _frontEndTol;
}
bool Motor::isHomeLimit() const { return limitActive(PIN_LIMIT_HOME); }
bool Motor::isOriginValid() const { return snapshot().valid; }
bool Motor::isSoftLimit() const {
  const Snap s = snapshot();
  return s.halt == SOFT || (s.valid && (s.pos < SOFT_MIN || s.pos >= SOFT_MAX));
}
bool Motor::hasMotionFault() const { const Snap s = snapshot(); return s.halt >= KO || s.driverFault || _overCurrent; }
bool Motor::isDriverFault() const { return snapshot().driverFault; }
bool Motor::isStalled() const { return snapshot().halt == HALT_STALL; }
uint8_t Motor::getFaultCode() const {
  const Snap s = snapshot();
  return s.driverFault ? 8 : (_overCurrent ? 9 : s.halt);
}
long Motor::getStepCount() const { return snapshot().pos; }
float Motor::getPositionMm() const { return getStepCount() / SERVO_STEPS_PER_MM; }
float Motor::getProgress() const {
  return _doseTarget > _doseStart ? constrain((float)(getStepCount() - _doseStart) / (_doseTarget - _doseStart), 0.0f, 1.0f) : 0;
}
float Motor::getMoveProgress() const {
  return _moveTarget != _moveStart ? constrain((float)(getStepCount() - _moveStart) / (_moveTarget - _moveStart), 0.0f, 1.0f) : 1;
}
float Motor::getElapsedSec() const { return (double)(_dosePriorUs + snapshot().segUs) / 1e6; }

ServoTelemetry Motor::getTelemetry() const {
  ServoTelemetry t;
  portENTER_CRITICAL(&ctlMux);
  t.online = ctl.online; t.echo = g_echo;
  t.model = g_model; t.mode = g_mode;
  t.voltageV = ctl.voltRaw / 10.0f; t.tempC = ctl.temp; t.loadPermil = ctl.load;
  t.status = ctl.status;
  t.posMm = (float)((ctl.abs - ctl.zero) / SPM); t.refMm = (float)(ctl.ref / SPM);
  t.cmdMmS = (float)(ctl.cmd / SPM);
  t.commErrors = ctl.commErrors; t.commRetries = ctl.commRetries;
  portEXIT_CRITICAL(&ctlMux);
  return t;
}

void Motor::updateCurrentMa(float ma) {
  _currentMa = ma;
  if (!isfinite(ma)) {
    if (!_overCurrent) Serial.println("[OC] 전류값을 읽을 수 없음(NaN) — 안전을 위해 차단");
    _overCurrent = true; failMotion(true); return;
  }
  if (ma > _peakMa) _peakMa = ma;
  if (ma < OVERCURRENT_MA) { _overCurrentMs = 0; return; }
  if (!_overCurrentMs) _overCurrentMs = millis();
  if (ma >= OVERCURRENT_HARD_MA || millis() - _overCurrentMs >= OVERCURRENT_HOLD_MS) {
    if (!_overCurrent) {
      Serial.print("[OC] 소프트웨어 차단 — 배터리 전류 ");
      Serial.print(ma, 0);
      Serial.print("mA (기준 ");
      Serial.print(ma >= OVERCURRENT_HARD_MA ? OVERCURRENT_HARD_MA : OVERCURRENT_MA, 0);
      Serial.print("mA, ");
      Serial.print(ma >= OVERCURRENT_HARD_MA ? "즉시" : "지속");
      Serial.println(")");
    }
    _overCurrent = true; failMotion(true);
  }
}
