#pragma once
#include <lvgl.h>

// 화면 팔레트 (RGB565 양자화는 LVGL이 한다)
#define LV_CUTE_BG          lv_color_hex(0xF0F2F6)  // 화면 배경
#define LV_CUTE_CARD        lv_color_hex(0xFFFFFF)  // 카드
#define LV_CUTE_ROW_HILITE  lv_color_hex(0xE0EDF8)  // 선택된 행 (설정 메뉴)
#define LV_CUTE_TEXT        lv_color_hex(0x161C26)
#define LV_CUTE_SUB         lv_color_hex(0x2F3743)  // 보조 글자 (TN 시야각 대비로 진하게)

#define LV_CUTE_BLUE        lv_color_hex(0x246DA5)  // 용량
#define LV_CUTE_BLUE_BG     lv_color_hex(0xF1F7FC)
#define LV_CUTE_GREEN       lv_color_hex(0x287A5D)  // 시간
#define LV_CUTE_GREEN_BG    lv_color_hex(0xF2F8F4)
#define LV_CUTE_PINK        lv_color_hex(0xA14972)  // 바늘
#define LV_CUTE_PINK_BG     lv_color_hex(0xFCF4F7)
#define LV_CUTE_MINT        lv_color_hex(0x287A5D)  // 완료·배터리
#define LV_CUTE_MINT_BG     lv_color_hex(0xDEF0E6)

#define LV_CUTE_SEP         lv_color_hex(0xE5E5EA)  // 구분선
#define LV_CUTE_HINTBAR_BG  lv_color_hex(0xF3F4F5)  // 조그 화면 하단 바
#define LV_CUTE_RED         lv_color_hex(0xB54E49)  // 정지·경고
#define LV_CUTE_RED_BG      lv_color_hex(0xFFEFEE)
#define LV_CUTE_ORANGE      lv_color_hex(0x986D37)  // 일시정지·조그·배터리 주의
#define LV_CUTE_ORANGE_BG   lv_color_hex(0xFBF0E3)

constexpr int16_t LV_CUTE_RADIUS_CARD = 21;

// 공용 스타일 (조그 화면에서 사용)
extern lv_style_t style_cute_screen;
extern lv_style_t style_cute_card;

void lvglThemeInit();  // lvglBridgeInit() 이후, 첫 화면 생성 전에 1회 호출
