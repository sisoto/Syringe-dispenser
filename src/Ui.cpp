#include "Ui.h"

Ui ui;

// Gray code 전이 → 이동 방향(-1, 0, +1). 인덱스: (이전 A·B << 2) | 현재 A·B
static const int8_t encoderTable[16] = {
   0,  1, -1,  0,  // 00 -> 00, 01, 10, 11
  -1,  0,  0,  1,  // 01 -> 00, 01, 10, 11
   1,  0,  0, -1,  // 10 -> 00, 01, 10, 11
   0, -1,  1,  0   // 11 -> 00, 01, 10, 11
};

void Ui::begin() {
  pinMode(PIN_EC11_A,    INPUT_PULLUP);
  pinMode(PIN_EC11_B,    INPUT_PULLUP);
  pinMode(PIN_EC11_PUSH, INPUT_PULLUP);
  pinMode(PIN_EC11_KO,   INPUT_PULLUP);
  pinMode(PIN_SW_FORWARD, INPUT_PULLUP);
  pinMode(PIN_SW_RETURN,  INPUT_PULLUP);

  // 버튼 LED는 Injection.ino가 상태에 맞춰 켜고 끈다. 여기서는 소등 상태로 준비만 한다.
  pinMode(PIN_FWD_LED, OUTPUT);
  digitalWrite(PIN_FWD_LED, ledLevel(false));
  // RETURN LED는 LEDC로 구동한다(pinMode를 부르면 LEDC 연결이 끊기므로 쓰지 않는다).
  pwmAttach(PIN_RETURN_LED, LED_PWM_FREQ_HZ);
  pwmWrite(PIN_RETURN_LED, ledLevel(false) == HIGH ? 255 : 0);

  uint32_t gpio = fastGpioIn0_31();
  uint8_t a = (gpio >> PIN_EC11_A) & 0x01;
  uint8_t b = (gpio >> PIN_EC11_B) & 0x01;
  _prevEncoderState = (a << 1) | b;
  _subStepCount = 0;
}

bool Ui::debounce(uint8_t& lastState, uint8_t pin,
                  unsigned long& lastMs, unsigned long threshMs) {
  uint8_t cur = digitalRead(pin);
  if (cur != lastState && millis() - lastMs > threshMs) {
    lastState = cur;
    lastMs    = millis();
    return true;
  }
  return false;
}

void Ui::pushEvent(UiEvent e) {
  uint8_t next = (uint8_t)((_evTail + 1) % EVENT_QUEUE_SIZE);
  if (next == _evHead) return;   // 가득 참 — 새 것을 버림
  _evQueue[_evTail] = e;
  _evTail = next;
}

UiEvent Ui::popEvent() {
  if (_evHead == _evTail) return UI_NONE;
  UiEvent e = _evQueue[_evHead];
  _evHead = (uint8_t)((_evHead + 1) % EVENT_QUEUE_SIZE);
  return e;
}

UiEvent Ui::poll() {
  unsigned long now = millis();

  // ── 1. EC11 로터리 ──
  // A/B를 한 번의 레지스터 읽기로 동시에 샘플링한다(따로 읽으면 사이에 값이 바뀔 수 있음).
  uint32_t gpio = fastGpioIn0_31();
  uint8_t currA = (gpio >> PIN_EC11_A) & 0x01;
  uint8_t currB = (gpio >> PIN_EC11_B) & 0x01;
  uint8_t currState = (currA << 1) | currB;

  if (currState != _prevEncoderState) {
    uint8_t idx = (_prevEncoderState << 2) | currState;
    _subStepCount += encoderTable[idx];
    _prevEncoderState = currState;

    if (_subStepCount >= PULSES_PER_DETENT) {
      _subStepCount = 0;
      pushEvent(UI_ROT_CW);
    } else if (_subStepCount <= -PULSES_PER_DETENT) {
      _subStepCount = 0;
      pushEvent(UI_ROT_CCW);
    }
  }

  // ── 2. PUSH (30ms 디바운스 + 더블클릭) ──
  uint8_t pushCur = digitalRead(PIN_EC11_PUSH);
  if (pushCur != _pushState && (now - _pushDebounceMs) > 30) {
    _pushDebounceMs = now;
    if (pushCur == LOW) {
      _pushState  = LOW;
      _pushDownMs = now;
      _longFired  = false;
      if (now - _pushFirstMs < DOUBLE_CLICK_MS && _pushCount > 0) {
        _pushCount   = 0;
        _pushPending = false;
        pushEvent(UI_PUSH_DOUBLE);
      } else {
        if (_pushCount == 0) _pushFirstMs = now;
        _pushCount++;
        _pushPending = true;
      }
    } else {
      _pushState = HIGH;
    }
  }

  // 롱프레스: 누른 채 LONG_PRESS_MS가 지나면 떼기 전에 바로 발행하고 클릭 대기는 취소
  if (_pushState == LOW && !_longFired &&
      now - _pushDownMs >= LONG_PRESS_MS) {
    _longFired   = true;
    _pushPending = false;
    _pushCount   = 0;
    pushEvent(UI_PUSH_LONG);
  }

  // 싱글클릭 확정: 손을 뗀 상태에서 SINGLE_CLICK_WAIT가 지났고 한 번만 눌렸을 때
  // (누르고 있는 동안 확정하면 롱프레스를 채우기 전에 클릭이 먼저 나간다)
  if (_pushPending && _pushCount == 1 && _pushState == HIGH &&
      now - _pushFirstMs > (unsigned long)SINGLE_CLICK_WAIT) {
    _pushPending = false;
    _pushCount   = 0;
    pushEvent(UI_PUSH_SINGLE);
  }

  // ── 3. FWD / RETURN (누르는 순간만 이벤트) ──
  if (debounce(_fwdState, PIN_SW_FORWARD, _fwdDebounce)) {
    if (_fwdState == LOW) pushEvent(UI_FWD);
  }
  if (debounce(_retState, PIN_SW_RETURN, _retDebounce)) {
    if (_retState == LOW) pushEvent(UI_RETURN);
  }

  return popEvent();
}
