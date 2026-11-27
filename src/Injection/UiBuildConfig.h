#pragma once
#include <lvgl.h>
#if LVGL_VERSION_MAJOR != 8
#error "This UI requires LVGL v8 (verified with 8.3.11), not v9."
#endif
#if LV_COLOR_DEPTH != 16
#error "Set LV_COLOR_DEPTH to 16 in the active lv_conf.h."
#endif
#if !LV_USE_IMG || !LV_USE_LABEL || !LV_USE_LINE
#error "Enable LV_USE_IMG, LV_USE_LABEL and LV_USE_LINE in lv_conf.h."
#endif
#if LV_MEM_CUSTOM == 0 && LV_MEM_SIZE < (128U * 1024U)
#error "UI needs LV_MEM_SIZE >= (128U * 1024U). Update the active lv_conf.h; see README-SERVO-KO.md."
#endif
