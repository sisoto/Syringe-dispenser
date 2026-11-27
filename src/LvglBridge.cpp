#include "LvglBridge.h"
#include "Display.h"   // extern LGFX tft;
#include "Platform.h"

// LVGL 8.3 API (v9와 드라이버 구조가 다르다).
// 부분 렌더 버퍼 480 x 60줄 x 2바이트 = 57.6KB, 내부 SRAM에 둔다(PSRAM은 느리고 DMA 제약).
#define LV_BUF_LINES 60
static lv_disp_draw_buf_t drawBuf;
static lv_color_t lvBuf1[TFT_W * LV_BUF_LINES];

// flush 콜백이 버퍼를 uint16_t로 넘기므로 RGB565(2바이트)여야 한다.
static_assert(sizeof(lv_color_t) == 2,
              "lv_color_t가 2바이트가 아님 — LV_COLOR_DEPTH 설정 확인 필요");

static lv_disp_drv_t dispDrv;

static void dispFlushCb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* colorP) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  // ST7796은 빅엔디안 RGB565를 받는다. lv_conf.h의 LV_COLOR_16_SWAP=1이면 LVGL이
  // 이미 바꿔 두었으므로 그대로, 0이면 LovyanGFX가 전송 중에 바꾼다.
#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
  constexpr bool pixel_swap = false;
#else
  constexpr bool pixel_swap = true;
#endif
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels((uint16_t*)colorP, w * h, pixel_swap);
  tft.endWrite();

  lv_disp_flush_ready(drv);
}

void lvglBridgeInit() {
  lv_init();
  logHeapInfo("before LVGL init");

  lv_disp_draw_buf_init(&drawBuf, lvBuf1, nullptr, TFT_W * LV_BUF_LINES);

  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res  = TFT_W;
  dispDrv.ver_res  = TFT_H;
  dispDrv.flush_cb = dispFlushCb;
  dispDrv.draw_buf = &drawBuf;
  lv_disp_drv_register(&dispDrv);
}

void lvglInvalidateScreen() {
  lv_obj_t* s = lv_scr_act();
  if (s) lv_obj_invalidate(s);
}

void lvglBridgeTick() {
  // LV_TICK_CUSTOM(lv_conf.h)이 millis()를 쓰므로 lv_tick_inc()는 필요 없다
  lv_timer_handler();
}
