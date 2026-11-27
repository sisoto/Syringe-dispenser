#include "Display.h"
#include "driver/gpio.h"
#include "Logo.h"

// TFT SPI 클럭. 480x320 한 프레임(307KB) 전송이 20MHz에서 약 123ms.
// 점퍼 배선에서 40MHz는 통신이 불안정했다. 깜빡임·노이즈가 보이면 10MHz로 내린다.
constexpr uint32_t TFT_SPI_HZ = 20000000;

LGFX    tft;
Display display;

LGFX::LGFX() {
  { // ── SPI 버스 (GPIO 매트릭스 경유, DMA) ──
    auto cfg = _bus.config();
    cfg.spi_host   = SPI2_HOST;      // ESP32-S3 FSPI
    cfg.spi_mode   = 0;
    cfg.freq_write = TFT_SPI_HZ;
    cfg.freq_read  = 16000000;
    cfg.spi_3wire  = false;
    cfg.use_lock   = true;
    cfg.dma_channel = SPI_DMA_CH_AUTO;
    cfg.pin_sclk   = PIN_TFT_SCL;
    cfg.pin_mosi   = PIN_TFT_SDA;
    cfg.pin_miso   = -1;             // 화면을 읽지 않음 (모듈 SDO 미연결)
    cfg.pin_dc     = PIN_TFT_DC;
    _bus.config(cfg);
    _panel.setBus(&_bus);
  }
  { // ── 패널: 물리 해상도는 세로형 320x480, setRotation으로 가로 480x320 ──
    auto cfg = _panel.config();
    cfg.pin_cs           = PIN_TFT_CS;
    cfg.pin_rst          = PIN_TFT_RES;
    cfg.pin_busy         = -1;
    cfg.panel_width      = 320;
    cfg.panel_height     = 480;
    cfg.offset_x         = 0;
    cfg.offset_y         = 0;
    cfg.offset_rotation  = 0;
    cfg.readable         = false;
    cfg.invert           = TFT_INVERT;
    cfg.rgb_order        = TFT_RGB_ORDER;
    cfg.dlen_16bit       = false;
    cfg.bus_shared       = false;    // SD 슬롯 미사용 → 패널 전용 버스
    _panel.config(cfg);
  }
  { // ── 백라이트 ──
    // LovyanGFX가 IDF ledc로 채널을 직접 잡는다. Arduino ledcAttach()(RETURN LED)는
    // 0번 채널부터 할당하므로, 겹치지 않게 마지막 채널 7을 쓴다.
    auto cfg = _light.config();
    cfg.pin_bl      = PIN_TFT_BLK;
    cfg.invert      = TFT_BL_INVERT;
    cfg.freq        = TFT_BL_PWM_HZ;
    cfg.pwm_channel = 7;
    _light.config(cfg);
    _panel.setLight(&_light);
  }
  setPanel(&_panel);
}

void Display::begin() {
  tft.init();
  // 백라이트 핀 구동 세기를 최소로 — 전환이 완만해져 옆 선(SCK)으로 넘어가는 잡음이 준다.
  gpio_set_drive_capability((gpio_num_t)PIN_TFT_BLK, GPIO_DRIVE_CAP_0);
  tft.setRotation(TFT_ROTATION);
  tft.setColorDepth(16);
  tft.fillScreen(TFT_BLACK);
  // 부팅 중 화면 전환이 비치지 않게 백라이트는 첫 그림(로고)을 그린 뒤 켠다.
  setBacklight(0);

  Serial.print("[TFT] ST7796S 4.0\" MSP4031 ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.print(tft.height());
  Serial.print(" @");
  Serial.print(TFT_SPI_HZ / 1000000);
  Serial.println("MHz DMA");
}

// 부팅 로고 (480x320 전체 화면). SISOTO는 호스트 바이트 순서 RGB565 = lgfx::rgb565_t.
// 빨강·파랑이 뒤바뀌어 보이면 캐스팅을 lgfx::swap565_t로 바꾼다.
void Display::showSplash() {
  tft.pushImage(0, 0, SISOTO_WIDTH, SISOTO_HEIGHT, (const lgfx::rgb565_t*)SISOTO);
}

void Display::setBacklight(uint8_t level) {
  tft.setBrightness(level);
}
