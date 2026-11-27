#include "Theme.h"

// 용량별 정렬 위치 (mm, 원점 = HOME 해제점). 10mL당 15mm 등간격.
const SyringeSpec syringeSpecs[] = {
  { 10, 60.0f },
  { 20, 45.0f },
  { 30, 30.0f },
  { 40, 15.0f },
  { 50,  0.0f },
};

const uint8_t SYRINGE_SPEC_COUNT =
  sizeof(syringeSpecs) / sizeof(syringeSpecs[0]);
