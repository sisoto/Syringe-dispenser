#pragma once
#include <Arduino.h>
#include "Theme.h"
#include "Platform.h"

// 입력 이벤트. CANCEL(KO)은 이벤트가 아니라 인터럽트(koButtonISR)로 처리한다.
enum UiEvent : uint8_t {
  UI_NONE,
  UI_ROT_CW,       // 시계방향 한 칸
  UI_ROT_CCW,      // 반시계 한 칸
  UI_PUSH_SINGLE,  // 엔코더 클릭
  UI_PUSH_DOUBLE,  // 더블클릭 (설정 진입)
  UI_PUSH_LONG,    // 롱프레스 (시스템 설정 메뉴) — 누른 채 LONG_PRESS_MS
  UI_FWD,          // 전진 버튼 누름 (모멘터리, 누를 때마다 1회)
  UI_RETURN,       // 리턴 버튼 누름
};

class Ui {
public:
  void begin();

  // 매 호출마다 모든 입력을 검사해 이벤트를 큐에 넣고, 가장 오래된 하나를 돌려준다
  // (같은 루프에 들어온 입력이 서로를 가리지 않게).
  UiEvent poll();

  InjectionConfig config;   // 현재 설정값

private:
  static constexpr uint8_t EVENT_QUEUE_SIZE = 8;
  UiEvent _evQueue[EVENT_QUEUE_SIZE];
  uint8_t _evHead = 0;   // 꺼낼 위치
  uint8_t _evTail = 0;   // 넣을 위치
  void    pushEvent(UiEvent e);
  UiEvent popEvent();

  // EC11 Gray code 4배 해석
  uint8_t _prevEncoderState = 0;  // 이전 A/B 2비트
  int8_t  _subStepCount     = 0;  // 서브 스텝 누적 (-4 ~ +4)
  static constexpr int8_t PULSES_PER_DETENT = 4;   // 딸깍 한 번당 펄스 (EC11 일반값)

  // PUSH: 디바운스 + 싱글/더블/롱 판정
  uint8_t        _pushState      = HIGH;
  uint8_t        _pushCount      = 0;
  unsigned long  _pushFirstMs    = 0;
  bool           _pushPending    = false;
  unsigned long  _pushDebounceMs = 0;
  unsigned long  _pushDownMs     = 0;
  bool           _longFired      = false;

  uint8_t        _fwdState      = HIGH;
  unsigned long  _fwdDebounce   = 0;
  uint8_t        _retState      = HIGH;
  unsigned long  _retDebounce   = 0;

  bool debounce(uint8_t& lastState, uint8_t pin, unsigned long& lastMs,
                unsigned long threshMs = 30);
};

extern Ui ui;
