#pragma once
#include "Theme.h"
// 빌드 시 핀 충돌·예약 핀 사용과 주요 상수 범위를 검사한다.
// 핀을 추가하면 아래 PINS 목록에도 넣을 것.
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This project requires ESP32-S3 N16R8"
#endif

namespace pin_checks {
constexpr uint8_t PINS[] = {
  PIN_SERVO_TX, PIN_SERVO_RX, PIN_LIMIT_HOME, PIN_LIMIT_FRNT,
  PIN_I2C_SDA, PIN_I2C_SCL, PIN_INA_ALERT,
  PIN_TFT_SCL, PIN_TFT_SDA, PIN_TFT_RES, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_BLK,
  PIN_EC11_A, PIN_EC11_B, PIN_EC11_PUSH, PIN_EC11_KO,
  PIN_SW_FORWARD, PIN_SW_RETURN, PIN_FWD_LED, PIN_RETURN_LED,
};
constexpr size_t PIN_COUNT = sizeof(PINS) / sizeof(PINS[0]);
// 예약: 0/3/45/46(스트래핑), 19/20(USB), 26~37(플래시·옥탈 PSRAM), 38/48(RGB LED), 43/44(UART0)
constexpr bool reserved(uint8_t p) {
  return p == 0 || p == 3 || p == 45 || p == 46 || p == 19 || p == 20 ||
         (p >= 26 && p <= 37) || p == 38 || p == 48 || p == 43 || p == 44;
}
constexpr bool noReservedPins() {
  for (size_t i = 0; i < PIN_COUNT; ++i) if (reserved(PINS[i])) return false;
  return true;
}
constexpr bool noPinCollisions() {
  for (size_t i = 0; i < PIN_COUNT; ++i)
    for (size_t j = i + 1; j < PIN_COUNT; ++j) if (PINS[i] == PINS[j]) return false;
  return true;
}
} // namespace pin_checks

static_assert(pin_checks::noReservedPins(), "Reserved GPIO used (see pin_checks::reserved)");
static_assert(pin_checks::noPinCollisions(), "Two pins in Theme.h share the same GPIO");
static_assert(PIN_EC11_A<32 && PIN_EC11_B<32,"EC11 requires low GPIO bank");
// Motor.cpp의 ALERT ISR이 GPIO_IN_REG(0~31번 핀)를 직접 읽는다.
static_assert(PIN_INA_ALERT < 32, "PIN_INA_ALERT must be GPIO0..31 (ISR reads GPIO_IN_REG)");
static_assert(SERVO_STEPS_PER_MM>0,"Invalid mechanics");
static_assert(SERVO_TORQUE_LIMIT<=1000,"SERVO_TORQUE_LIMIT is 0..1000");
static_assert(SERVO_MAX_SPEED>0 && SERVO_MAX_SPEED<=32767,"STS speed field is 15 bits");
static_assert(TFT_ROTATION==1 || TFT_ROTATION==3,"TFT_ROTATION must be 1 or 3 (landscape 480x320)");
static_assert(SYRINGE_EMPTY_MM > 0 && SYRINGE_EMPTY_MM <= MAX_TRAVEL_MM - SOFT_LIMIT_MARGIN - DISPENSE_TARGET_MARGIN_MM,
              "SYRINGE_EMPTY_MM must lie inside the soft limit");
static_assert(INA228_SOVL_LSB>0 && INA228_SOVL_LSB<=32767,"INA228 ALERT threshold exceeds shunt ADC range (163.84mV)");
static_assert(INA228_ALERT_MA<INA228_SATURATED_MA,"ALERT threshold must be below INA228 saturation");
static_assert(SERVO_PERIOD_MS>=5 && SERVO_PERIOD_MS<=50,"Servo loop period out of range");
// 0~4095 각도를 펼치려면 피드백 공백 동안의 이동이 반 바퀴보다 작아야 한다.
static_assert(SERVO_MAX_SPEED*SERVO_FEEDBACK_GAP_MS/1000.0f < 2048,"Feedback gap too long for unwrapping");
