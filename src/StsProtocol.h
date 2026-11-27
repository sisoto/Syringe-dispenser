#pragma once
#include <stdint.h>
#include <stddef.h>

// ══════════════════════════════════════════════════════════
// Feetech STS(SMS_STS) 버스 서보 프로토콜 — 하드웨어와 무관한 순수 코드
//
// STS3250(모델 번호 2825)은 STS3215와 같은 SMS_STS 메모리 테이블을 쓴다.
// 패킷: FF FF ID LEN INST PARAM... CHK
//   LEN = PARAM 개수 + 2,  CHK = ~(ID + LEN + INST + ΣPARAM) & 0xFF
// 응답: FF FF ID LEN ERR PARAM... CHK
// 16비트 값은 리틀 엔디언(하위 바이트 먼저).
// 부호 있는 값은 2의 보수가 아니라 "부호 비트 + 크기" 형식이다.
//   속도·위치·전류: bit15가 부호 / 부하: bit10이 부호
//
// 주소 근거: Feetech 공식 Arduino 라이브러리(FTServo_Arduino) SMS_STS.h
// ══════════════════════════════════════════════════════════
namespace sts {

constexpr uint8_t INST_PING  = 0x01;
constexpr uint8_t INST_READ  = 0x02;
constexpr uint8_t INST_WRITE = 0x03;

// ── EEPROM 영역 ──
// EEPROM 잠금(LOCK=1, 전원 투입 기본값) 상태에서 쓰면 값은 즉시 적용되지만
// 전원을 끄면 원래 값으로 돌아간다(문서상 동작). 그래서 이 펌웨어는 기본적으로
// 잠금을 풀지 않고 매 부팅 값을 다시 적용한다. 잠금 상태 쓰기를 무시하는
// 개체일 때만 동작 모드를 한 번 저장한다(Motor::configureServo 참고).
constexpr uint8_t REG_MODEL           = 3;   // 2바이트
constexpr uint8_t REG_ID              = 5;
constexpr uint8_t REG_MODE            = 33;  // 0=위치, 1=정속(휠), 2=PWM, 3=스텝

// ── SRAM 영역 ──
constexpr uint8_t REG_TORQUE_ENABLE   = 40;  // 0=토크 해제, 1=토크 ON
constexpr uint8_t REG_ACC             = 41;  // 가속도, 단위 100 step/s², 0=최대(램프 없음)
constexpr uint8_t REG_GOAL_POSITION   = 42;  // 2바이트
constexpr uint8_t REG_GOAL_SPEED      = 46;  // 2바이트, bit15 부호, 단위 step/s
constexpr uint8_t REG_TORQUE_LIMIT    = 48;  // 2바이트, 0~1000 (0.1%)
constexpr uint8_t REG_LOCK            = 55;

// ── 피드백(읽기 전용, 56~70을 한 번에 읽는다) ──
constexpr uint8_t REG_PRESENT_POSITION    = 56; // 2바이트, 0~4095
constexpr uint8_t REG_PRESENT_SPEED       = 58; // 2바이트, bit15 부호
constexpr uint8_t REG_PRESENT_LOAD        = 60; // 2바이트, bit10 부호, 0~1000
constexpr uint8_t REG_PRESENT_VOLTAGE     = 62; // 1바이트, 0.1V
constexpr uint8_t REG_PRESENT_TEMPERATURE = 63; // 1바이트, °C
constexpr uint8_t REG_STATUS              = 65; // 1바이트, 오류 비트
constexpr uint8_t REG_PRESENT_CURRENT     = 69; // 2바이트, bit15 부호
constexpr uint8_t FEEDBACK_FIRST = REG_PRESENT_POSITION;
constexpr uint8_t FEEDBACK_LEN   = REG_PRESENT_CURRENT + 2 - REG_PRESENT_POSITION; // 15

constexpr uint8_t MODE_WHEEL    = 1;   // REG_MODE 값: 정속(휠)

constexpr uint16_t STEPS_PER_REV = 4096;
constexpr size_t   MAX_PACKET    = 64;

inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline void putLe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }

// 부호+크기(bit15) 인코딩. 크기는 0~32767로 포화시킨다.
inline uint16_t encodeSigned15(int32_t v) {
  uint32_t mag = (uint32_t)(v < 0 ? -(int64_t)v : v);
  if (mag > 0x7FFF) mag = 0x7FFF;
  return (uint16_t)(mag | (v < 0 ? 0x8000u : 0u));
}
inline int32_t decodeSigned15(uint16_t raw) {
  const int32_t mag = raw & 0x7FFF;
  return (raw & 0x8000) ? -mag : mag;
}
// 부하는 bit10이 부호(라이브러리 ReadLoad와 동일한 해석).
inline int32_t decodeLoad(uint16_t raw) {
  const int32_t mag = raw & ~(1u << 10);
  return (raw & (1u << 10)) ? -mag : mag;
}

// ID..마지막 파라미터까지의 합으로 체크섬 계산
inline uint8_t checksum(const uint8_t *p, size_t n) {
  uint32_t s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return (uint8_t)(~s & 0xFF);
}

// 요청 패킷 조립. 성공 시 전체 길이, 실패 시 0.
inline size_t buildPacket(uint8_t *out, size_t cap, uint8_t id, uint8_t inst,
                          const uint8_t *params, uint8_t nParams) {
  const size_t total = (size_t)nParams + 6;
  if (total > cap || nParams > MAX_PACKET - 6) return 0;
  out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = (uint8_t)(nParams + 2); out[4] = inst;
  for (uint8_t i = 0; i < nParams; ++i) out[5 + i] = params[i];
  out[5 + nParams] = checksum(out + 2, (size_t)nParams + 3);
  return total;
}
inline size_t buildPing(uint8_t *out, size_t cap, uint8_t id) {
  return buildPacket(out, cap, id, INST_PING, nullptr, 0);
}
inline size_t buildRead(uint8_t *out, size_t cap, uint8_t id, uint8_t addr, uint8_t len) {
  const uint8_t p[2] = {addr, len};
  return buildPacket(out, cap, id, INST_READ, p, 2);
}
inline size_t buildWrite(uint8_t *out, size_t cap, uint8_t id, uint8_t addr,
                         const uint8_t *data, uint8_t len) {
  uint8_t p[MAX_PACKET];
  if ((size_t)len + 1 > sizeof(p)) return 0;
  p[0] = addr;
  for (uint8_t i = 0; i < len; ++i) p[1 + i] = data[i];
  return buildPacket(out, cap, id, INST_WRITE, p, (uint8_t)(len + 1));
}

// 응답 패킷을 한 바이트씩 받아 조립한다. 잡음/에코로 헤더가 어긋나도
// FF FF를 다시 찾아 동기화한다.
struct Parser {
  enum Result : uint8_t { NEED_MORE, PACKET, BAD_CHECKSUM };
  uint8_t buf[MAX_PACKET];
  uint8_t n = 0;

  void reset() { n = 0; }
  uint8_t id() const { return buf[2]; }
  uint8_t error() const { return buf[4]; }       // 응답일 때 오류 바이트(요청 에코면 INST)
  uint8_t paramCount() const { return (uint8_t)(buf[3] - 2); }
  const uint8_t *params() const { return buf + 5; }

  Result push(uint8_t b) {
    if (n == 0) { if (b == 0xFF) buf[n++] = b; return NEED_MORE; }
    if (n == 1) { if (b == 0xFF) buf[n++] = b; else n = 0; return NEED_MORE; }
    if (n == 2) {
      if (b == 0xFF) return NEED_MORE;           // FF FF FF ... : 헤더 연장
      buf[n++] = b; return NEED_MORE;
    }
    if (n == 3) {
      if (b < 2 || (size_t)b + 4 > MAX_PACKET) { n = 0; return NEED_MORE; }
      buf[n++] = b; return NEED_MORE;
    }
    buf[n++] = b;
    if (n < (size_t)buf[3] + 4) return NEED_MORE;
    const bool ok = checksum(buf + 2, (size_t)buf[3] + 1) == buf[n - 1];
    n = 0;  // 다음 패킷용. buf 내용은 다음 push 전까지 유효
    return ok ? PACKET : BAD_CHECKSUM;
  }
};

} // namespace sts
