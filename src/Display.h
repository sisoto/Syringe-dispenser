#pragma once
#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "Theme.h"

// ══════════════════════════════════════════════════════════
// Display — 4.0" ST7796S 480x320 (LCDwiki MSP4031) 패널 초기화
//   begin()        패널·SPI(DMA)·백라이트 초기화 (백라이트는 꺼진 채로 둠)
//   showSplash()   부팅 로고 (LVGL 시작 전 1회)
//   setBacklight() 밝기 0~255
// 화면 그리기는 전부 LVGL(LvglBridge/LvglScreens)이 맡는다.
// 부팅 로고(Logo.h, 300KB)는 Display.cpp에서만 include한다.
// ══════════════════════════════════════════════════════════

constexpr int16_t TFT_W = 480;   // 가로 (회전 적용 후)
constexpr int16_t TFT_H = 320;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX();
};

extern LGFX tft;

class Display {
public:
  void begin();
  void showSplash();
  void setBacklight(uint8_t level);
};

extern Display display;
