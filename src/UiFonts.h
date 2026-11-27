#pragma once
#include <lvgl.h>
// Bundled Nimbus Sans: 4bpp; independent of lv_conf.h font switches.
LV_FONT_DECLARE(ui_font_14);
LV_FONT_DECLARE(ui_font_17);
LV_FONT_DECLARE(ui_font_18);
LV_FONT_DECLARE(ui_font_20);
LV_FONT_DECLARE(ui_font_22);
LV_FONT_DECLARE(ui_font_26);
LV_FONT_DECLARE(ui_font_28);
LV_FONT_DECLARE(ui_font_32);
LV_FONT_DECLARE(ui_font_52);
#define UI_FONT_S (&ui_font_14)
#define UI_FONT_M (&ui_font_18)
#define UI_FONT_L (&ui_font_22)
#define UI_FONT_XL (&ui_font_26)
#define UI_FONT_2XL (&ui_font_32)
