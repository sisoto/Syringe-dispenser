#include "LvglTheme.h"
#include "UiFonts.h"

lv_style_t style_cute_screen;
lv_style_t style_cute_card;

void lvglThemeInit() {
  lv_style_init(&style_cute_screen);
  lv_style_set_bg_color(&style_cute_screen, LV_CUTE_BG);
  lv_style_set_bg_opa(&style_cute_screen, LV_OPA_COVER);
  lv_style_set_text_color(&style_cute_screen, LV_CUTE_TEXT);
  lv_style_set_text_font(&style_cute_screen, UI_FONT_L);

  lv_style_init(&style_cute_card);
  lv_style_set_radius(&style_cute_card, LV_CUTE_RADIUS_CARD);
  lv_style_set_bg_color(&style_cute_card, LV_CUTE_CARD);
  lv_style_set_bg_opa(&style_cute_card, LV_OPA_COVER);
  lv_style_set_border_width(&style_cute_card, 0);
  lv_style_set_pad_all(&style_cute_card, 0);
}
