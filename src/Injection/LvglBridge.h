#pragma once
#include <lvgl.h>

// LVGL 초기화 — 디스플레이 플러시 드라이버 등록. display.begin() 뒤에 호출한다.
// LVGL은 렌더링 전용이고 입력·상태 처리는 Ui::poll()/Injection.ino가 한다.
void lvglBridgeInit();

// loop()에서 매 프레임 호출 — LVGL 내부 타이머/렌더링 처리
void lvglBridgeTick();

// 현재 화면 전체를 다시 그리게 한다(밝기 변경 직후 등, 혹시 어긋난 픽셀 복구용)
void lvglInvalidateScreen();
