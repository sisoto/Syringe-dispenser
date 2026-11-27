#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════
// Theme.h — 핀 배치와 모든 튜닝 상수 (ESP32-S3 DevKitC-1 N16R8)
// ══════════════════════════════════════════════════════════

// ── 서보 버스: Feetech STS3250 + Waveshare Bus Servo Adapter (A) ──
// 어댑터 점퍼는 MCU UART 모드. 핀 이름이 MCU 기준이라 "TX-TX, RX-RX"로 잇는다.
//   GPIO9  (TX, J1-15) → 어댑터 TX
//   GPIO10 (RX, J1-16) ← R1 1kΩ ← 어댑터 RX
//   GND 공통 (ESP32 ↔ 어댑터 ↔ 9V 전원)
constexpr uint8_t PIN_SERVO_TX = 9;
constexpr uint8_t PIN_SERVO_RX = 10;

// ── 리밋 스위치 ──
constexpr uint8_t PIN_LIMIT_HOME = 13;
constexpr uint8_t PIN_LIMIT_FRNT = 14;
// false = NO 결선(COM→GPIO, NO→GND): 평소 HIGH, 눌리면 LOW
// true  = NC 결선(COM→GPIO, NC→GND): 평소 LOW, 눌림·단선 시 HIGH
constexpr bool LIMIT_ACTIVE_HIGH = false;

// ── INA228 (I2C 주소 0x40) ──
constexpr uint8_t PIN_I2C_SDA    = 18;
constexpr uint8_t PIN_I2C_SCL    = 17;
// ALERT: 오픈 드레인·액티브 로우, 내부 풀업 사용. 션트 전압이 INA228_ALERT_MA에
// 해당하는 값을 넘으면 INA228이 LOW로 래치 → 인터럽트 → 서보 태스크가 10ms 안에
// 정지·토크 해제한다. I2C 폴링이나 메인 루프와 무관한 하드웨어 보호선.
constexpr uint8_t PIN_INA_ALERT  = 8;

// ── 4.0" TFT ST7796S 480x320 (LCDwiki MSP4031, 14핀) ──
// 모듈 3~8번이 J1-4~9에 같은 순서로 대응해 평행 배선이 된다.
//   1 VCC → 5V   2 GND   3 LCD_CS   4 LCD_RST   5 LCD_RS(DC)   6 MOSI   7 SCK   8 LED
//   9 MISO, 10~13 터치, 14 SD_CS는 연결하지 않는다.
constexpr uint8_t PIN_TFT_CS     = 4;   // 모듈 3 LCD_CS
constexpr uint8_t PIN_TFT_RES    = 5;   // 모듈 4 LCD_RST
constexpr uint8_t PIN_TFT_DC     = 6;   // 모듈 5 LCD_RS
constexpr uint8_t PIN_TFT_SDA    = 7;   // 모듈 6 SDI(MOSI)
constexpr uint8_t PIN_TFT_SCL    = 15;  // 모듈 7 SCK
constexpr uint8_t PIN_TFT_BLK    = 16;  // 모듈 8 LED

// 화면이 뒤집혀 보이면 TFT_ROTATION 1 ↔ 3. 색 반전이면 TFT_INVERT,
// 빨강·파랑이 바뀌면 TFT_RGB_ORDER, 밝기가 거꾸로면 TFT_BL_INVERT.
constexpr uint8_t TFT_ROTATION   = 3;
constexpr bool    TFT_INVERT     = false;
constexpr bool    TFT_RGB_ORDER  = false;
constexpr bool    TFT_BL_INVERT  = false;
// 백라이트 PWM 주파수. LED 선 바로 옆이 SCK라 PWM 전환 잡음이 화면 데이터를
// 깨뜨릴 수 있어 전환 횟수가 적은 1kHz로 둔다.
constexpr uint32_t TFT_BL_PWM_HZ = 1000;

// ── EC11 로터리 + CANCEL(KO) ──
constexpr uint8_t PIN_EC11_A     = 1;   // GPIO<32 필수 (A/B 동시 샘플링)
constexpr uint8_t PIN_EC11_B     = 2;   // GPIO<32 필수
constexpr uint8_t PIN_EC11_PUSH  = 42;
constexpr uint8_t PIN_EC11_KO    = 41;  // 인터럽트

// ── 조명 버튼 (모멘터리) ──
constexpr uint8_t PIN_SW_FORWARD = 40;
constexpr uint8_t PIN_SW_RETURN  = 39;
constexpr uint8_t PIN_FWD_LED    = 47;
constexpr uint8_t PIN_RETURN_LED = 21;

// 버튼 LED가 액티브 로우(LOW=점등)로 배선돼 있다.
constexpr bool LED_ACTIVE_LOW = true;
// 코드에서는 항상 ledLevel(true)=켜기 / ledLevel(false)=끄기로 쓴다.
constexpr uint8_t ledLevel(bool on) {
  return (on != LED_ACTIVE_LOW) ? HIGH : LOW;
}
// LED 규칙(FWD·RETURN 공통): 점등 = 누를 차례, 점멸 = 그 버튼으로 시작한 동작 진행 중
constexpr unsigned long BUTTON_LED_BLINK_MS = 500UL;   // 점멸 반주기
constexpr uint32_t      LED_PWM_FREQ_HZ     = 1000;    // RETURN LED(LEDC)

// ══════════════════════════════════════════════════════════
// STS3250 서보 구동부 (README-SERVO-KO.md 참고)
// ══════════════════════════════════════════════════════════
// 서보는 정속(휠) 모드로 돌리고 위치 제어는 ESP32가 한다. 서보 엔코더(1회전
// 4096 step)를 10ms마다 읽어 다회전 위치로 펼치고, 궤적(ServoProfile)의 기준
// 위치와의 차이를 보정해 속도 명령을 보낸다. 서보 위치 모드는 1회전 남짓
// 범위라 90mm(22.5회전) 스트로크를 다룰 수 없다.

// ── 버스 ──
constexpr uint8_t  SERVO_UART_NUM   = 1;        // UART1 (UART0=43/44는 USB 로그)
constexpr uint32_t SERVO_BAUD       = 1000000;  // STS 공장 기본값
constexpr uint8_t  SERVO_ID         = 1;        // STS 공장 기본 ID
constexpr uint16_t SERVO_MODEL_EXPECTED = 2825; // STS3250 (다르면 경고만)
// 어댑터 에코(보낸 바이트가 RX로 돌아옴): -1=부팅 시 자동 판별, 0=없음, 1=있음
constexpr int8_t   SERVO_ECHO       = -1;
constexpr uint32_t SERVO_REPLY_TIMEOUT_US = 3000;

// ── 기구 환산: 서보 → 커플러 → T8 리드스크류(4mm/rev) = 1024 step/mm ──
constexpr float    LEAD_PITCH_MM      = 4.0f;
constexpr float    SERVO_GEAR_RATIO   = 1.0f;   // 나사 1회전당 서보 회전수
constexpr float    SERVO_STEPS_PER_REV = 4096.0f;
constexpr float    SERVO_STEPS_PER_MM = SERVO_STEPS_PER_REV * SERVO_GEAR_RATIO / LEAD_PITCH_MM;
// 피스톤 전진이 서보 +속도와 반대이면 true (첫 호밍이 HOME 반대로 가면 바꾼다).
constexpr bool     SERVO_DIR_INVERT   = true;
// +속도에서 위치 레지스터가 감소하는 개체면 true. 틀리면 호밍 중 감지해 로그로 안내한다.
constexpr bool     SERVO_FEEDBACK_INVERT = false;
constexpr float    MAX_TRAVEL_MM      = 92.0f;

// ── 주입 끝 위치 ──
// 주사기가 비는(전방 리밋이 눌리는) 위치(mm, 원점 기준). 주입 거리 = 이 위치 − 정렬 위치.
// 처음(학습 전) 목표이자 학습값의 기준이다.
constexpr float    SYRINGE_EMPTY_MM   = 76.2f;
// 주입 끝 무렵 전방 리밋이 눌리면, 남은 거리가 max(FRONT_END_TOL_MM, 주입 거리 ×
// DISPENSE_COMPLETE_TOL_PCT%) 이하일 때 정상 완료로 처리한다. 그보다 일찍 눌리면 리밋 정지.
constexpr float    FRONT_END_TOL_MM   = 3.0f;
constexpr float    DISPENSE_COMPLETE_TOL_PCT = 5.0f;
// 목표에 도착했는데 전방 리밋이 안 눌렸으면 이 속도로 최대 이 거리까지 더 밀어
// 리밋을 찾는다. 못 찾으면 그 자리에서 완료(로그에 경고).
constexpr float    FRONT_SEEK_MAX_MM     = 3.0f;
constexpr float    FRONT_SEEK_SPEED_MM_S = 1.0f;
// 주입이 리밋에서 끝나면 눌린 위치를 NVS에 저장해 다음 목표로 쓴다.
// SYRINGE_EMPTY_MM ± 이 범위 안의 값만 받는다.
constexpr float    FRONT_LEARN_RANGE_MM  = 4.0f;

// ── 토크 제한 (0~1000 = 0~100%, 매 부팅 SRAM에 씀) ──
// Feetech의 토크 제한은 실제로는 모터 PWM 출력 상한이다. 9V·2mm/s에서 무부하로도
// 약 50%를 쓰므로 700이면 약 20%p 여유가 남는다. 막힘이 잦으면 800, 과전류가 뜨면 600.
constexpr uint16_t SERVO_TORQUE_LIMIT = 700;

// ── 속도·가속 (STS 속도 단위 1 step/s) ──
// 9V 무부하 최고속 약 3700 step/s에서 여유를 두고 3000(≈2.9mm/s)으로 제한.
constexpr float SERVO_MAX_SPEED = 3000.0f;   // step/s
constexpr float SERVO_ACCEL     = 4096.0f;   // step/s² (= 4 mm/s²)
constexpr float SERVO_RAMP_SEC  = 0.5f;      // 기본 가감속 시간
// 서보 자체 가속도 레지스터(×100 step/s²). 10ms 명령 사이를 잇는 상한 역할만 한다.
constexpr uint8_t SERVO_ACC_REG = 100;
constexpr float MOVE_SPEED_MM_S    = 2.0f;   // 정렬 이동
constexpr float HOME_SPEED_MM_S    = 2.0f;   // 호밍 탐색
constexpr float BACKOFF_SPEED_MM_S = 1.0f;   // HOME 해제(백오프)
constexpr float JOG_SPEED_MM_S     = 1.0f;   // 수동 전진 조그
constexpr float HOME_BACKOFF_MAX_MM = 5.0f;
constexpr unsigned long HOME_SETTLE_MS = 20;

// ── 위치 루프 (10ms, 별도 FreeRTOS 태스크) ──
constexpr uint32_t SERVO_PERIOD_MS  = 10;
constexpr float    SERVO_KP         = 2.0f;    // 1/s: 위치오차 1mm → +2mm/s 보정
// 적분 보정(1/s²). 부하로 처지는 정상상태 오차를 메운다. SERVO_MAX_CORR로 제한하고
// 이동 시작마다 0으로 초기화한다.
constexpr float    SERVO_KI         = 1.0f;
constexpr float    SERVO_MAX_CORR   = 1024.0f; // 보정 속도 상한 step/s (1mm/s)
// 소음 저감: 보정량을 1차 저역 필터로 거르고(시정수, 초), 속도 명령 변화가
// 이 값(step/s) 미만이면 다시 쓰지 않는다(200ms마다는 갱신).
constexpr float    SERVO_CORR_TAU_S    = 0.15f;
constexpr int32_t  SERVO_CMD_DEADBAND  = 2;
constexpr float    SERVO_MIN_SPEED  = 30.0f;   // 마지막 접근 최소 속도 step/s
constexpr float    SERVO_POS_TOL    = 4.0f;    // 목표 도달 판정 step (≈0.004mm)
constexpr float    SERVO_DONE_TOL_MM = 0.05f;  // 정착 시간이 지나도 이 안이면 완료
constexpr unsigned long SERVO_SETTLE_MS = 1500;
// 막힘 판정
//  ① 추종오차 > SERVO_FOLLOW_ERR_MM, 출력 ≥ 토크 제한 × SERVO_STALL_LOAD_FRAC,
//     거의 안 움직이는 상태가 SERVO_FOLLOW_ERR_MS 지속 → 정지(진짜 막힘)
//  ② 출력과 무관하게 추종오차 > SERVO_FOLLOW_ERR_HARD_MM 가
//     SERVO_FOLLOW_ERR_HARD_MS 지속 → 정지(서보·피드백 이상)
constexpr float    SERVO_FOLLOW_ERR_MM = 1.0f;
constexpr unsigned long SERVO_FOLLOW_ERR_MS = 500;
constexpr float    SERVO_STALL_LOAD_FRAC = 0.90f;
constexpr float    SERVO_FOLLOW_ERR_HARD_MM = 3.0f;
constexpr unsigned long SERVO_FOLLOW_ERR_HARD_MS = 1500;
// 정속 이동(호밍·조그)에서 실측 속도가 명령의 25% 미만으로 1초 지속 → 막힘
constexpr float    SERVO_STALL_RATIO = 0.25f;
constexpr unsigned long SERVO_STALL_MS = 1000;
// ── 통신/안전 ──
constexpr unsigned long SERVO_FEEDBACK_GAP_MS = 200; // 구동 중 위치 샘플 공백 한계
constexpr uint8_t  SERVO_COMM_FAIL_LIMIT = 5;        // 연속 읽기 실패 → 서보 고장
// 위치 루프 태스크 코어. WiFi 스택(코어 0)과 떨어뜨려 통신 오류를 줄인다.
constexpr uint8_t  SERVO_TASK_CORE = 1;
constexpr uint8_t  SERVO_ERR_LIMIT       = 3;        // 오류 비트 연속 보고 → 고장
// 고장으로 볼 상태 비트: 1=전압 2=센서 4=과열 8=과전류 0x20=과부하
constexpr uint8_t  SERVO_FAULT_MASK      = 0x2F;
// loop()가 이 시간 넘게 motor.service()를 못 부르면 서보를 세운다.
constexpr unsigned long SERVO_HEARTBEAT_MS = 2000;
constexpr unsigned long SERVO_LOG_MS = 1000;         // 구동 중 [SERVO] 로그 주기 (0=끔)
// 원점 아래쪽 소프트 리밋 여유 — 엔코더 흔들림(-1 step)을 리밋으로 오판하지 않게
constexpr float    SOFT_LIMIT_LOW_TOL_MM = 0.3f;
constexpr float    SOFT_LIMIT_MARGIN     = 1.0f;     // MAX_TRAVEL_MM 앞 소프트 리밋 (mm)
// 주입 목표 상한을 소프트 리밋보다 이만큼 더 안쪽에 둔다(반올림으로 리밋이 먼저 걸리지 않게)
constexpr float    DISPENSE_TARGET_MARGIN_MM = 1.0f;

constexpr unsigned long DISPENSE_PAUSE_TIMEOUT_MS = 300000UL; // 일시정지 방치 5분
constexpr unsigned long HOMING_TIMEOUT_MS = 180000UL;
constexpr unsigned long MOVE_TIMEOUT_MS   = 90000UL;          // 정렬 이동

// ══════════════════════════════════════════════════════════
// 전류·배터리 (INA228: 배터리 → F1 → 션트 → 9V/5V 승압 분기 앞)
// ══════════════════════════════════════════════════════════
// 배터리 전체 전류 기준. OVERCURRENT_MA 이상이 OVERCURRENT_HOLD_MS 지속되면 차단,
// OVERCURRENT_HARD_MA 이상이면 즉시 차단(두 값이 같으면 항상 즉시).
constexpr float         OVERCURRENT_MA      = 7000.0f;
constexpr float         OVERCURRENT_HARD_MA = 7000.0f;
constexpr unsigned long OVERCURRENT_HOLD_MS = 200UL;

constexpr uint8_t INA228_ADDR_BAT     = 0x40;     // A0=A1=GND
constexpr float   INA228_SHUNT_OHM    = 0.015f;   // R015
constexpr float   INA228_SHUNT_MAX_MV = 163.84f;  // ADC range 0
constexpr float   INA228_SATURATION_MA = INA228_SHUNT_MAX_MV / INA228_SHUNT_OHM;  // ≈10923 mA
constexpr float   INA228_MAX_CURRENT_A = 10.0f;   // 캘리브레이션 범위 (보호 임계값보다 넓게)
constexpr float   INA228_SATURATED_MA  = INA228_SATURATION_MA * 0.98f;  // 이상이면 포화로 간주
// setAverage 인덱스: 0=1, 1=4, 2=16, 3=64 … (16회 × 1.052ms × 2 ≈ 35ms)
constexpr uint8_t INA228_AVERAGE_IDX = 2;

// ── INA228 ALERT (하드웨어 과전류 차단) ──
// SOVL은 ADC range 0에서 5µV/LSB → 7000mA × 0.015Ω = 105mV = 21000 LSB.
constexpr float INA228_ALERT_MA   = OVERCURRENT_HARD_MA;
// true: 16회 평균값으로 비교(기동 스파이크에 오탐하지 않음), false: 변환마다 비교
constexpr bool  INA228_ALERT_SLOW = true;
// ALERT 하강 에지 후 이 시간(µs) 뒤에도 LOW일 때만 과전류로 인정(배선 잡음 거름)
constexpr uint32_t INA228_ALERT_CONFIRM_US = 20;
constexpr long  INA228_SOVL_LSB   = (long)(INA228_ALERT_MA / 1000.0f * INA228_SHUNT_OHM / 5.0e-6f + 0.5f);

constexpr uint8_t BAT_WARN_PCT     = 20;   // 배터리 아이콘 주황
constexpr uint8_t BAT_CRITICAL_PCT = 10;   // 배터리 아이콘 빨강
constexpr uint8_t BAT_BLOCK_PCT    = 5;    // 주입 차단 + 경고창

// 구동 중 저전압 차단 — 보드가 읽는 원시 전압 기준(BAT_SENSE_OFFSET_V 미적용).
// 부하 중에는 전압이 처지는 게 정상이라 무부하 컷오프보다 낮게, HOLD_MS로 순간 처짐을 거른다.
constexpr float         BAT_LOAD_CUTOFF_V       = 3.00f;
constexpr unsigned long BAT_LOAD_CUTOFF_HOLD_MS = 500UL;

// 측정값에 더할 경로 손실 보정(V). SOC는 모터 정지 중에만 갱신하므로 저부하 기준값을 쓴다.
// 배선을 바꾸면 화면 Power Info의 V= 값과 멀티미터 셀 전압의 차이로 다시 잡는다.
constexpr float BAT_SENSE_OFFSET_V = 0.01f;
// 이 전압(보정 후) 이상이면 100%. 충전 모듈의 실제 만충 안정 전압에 맞춘다.
constexpr float BAT_FULL_V         = 4.10f;
// 이 % 위 구간만 BAT_FULL_V에서 100%가 되도록 늘려 매핑한다(아래 곡선은 그대로).
constexpr float BAT_FULL_BLEND_PCT = 80.0f;

// ── 니들 선택 (설정 메뉴 "Needle", NVS 저장) ──
// 끄면 설정이 용량 → 시간 2단계가 되고 메인·완료 화면에서 니들이 빠진다.
// 이때 바늘별 최소 시간 대신 NO_NEEDLE_MIN_TIME_SEC를 최소 주입 시간으로 쓴다.
constexpr bool     NEEDLE_SELECT_DEFAULT_ON = false;   // 처음 기본값
constexpr uint16_t NO_NEEDLE_MIN_TIME_SEC   = 120;     // 02:00

// ── 진단 표시 ──
// 화면 맨 위 전압·전류 표시(설정 메뉴 "Power Info", NVS 저장)의 처음 기본값
constexpr bool POWER_INFO_DEFAULT_ON = true;
// 션트 전압·전류 원시값을 1초마다 시리얼/Telnet에 [IRAW]로 출력
constexpr bool LOG_CURRENT_RAW = true;

// ── 주사기 스펙 테이블 (Theme.cpp) ──
struct SyringeSpec {
  uint8_t  volumeMl;
  float    offsetMm;   // 정렬 위치
};
extern const SyringeSpec syringeSpecs[];
extern const uint8_t     SYRINGE_SPEC_COUNT;

// ══════════════════════════════════════════════════════════
// 화면·입력·WiFi
// ══════════════════════════════════════════════════════════
// 시스템 설정에서 고르는 밝기(%) 범위와 디밍 시간 선택지(초, 0=디밍 안 함)
constexpr uint8_t SYS_BRIGHT_MIN_PCT  = 20;
constexpr uint8_t SYS_BRIGHT_MAX_PCT  = 100;
constexpr uint8_t SYS_BRIGHT_STEP_PCT = 10;
constexpr uint16_t SYS_DIM_CHOICES[] = { 0, 30, 60, 180, 300, 600 };
constexpr uint8_t  SYS_DIM_CHOICE_COUNT =
    sizeof(SYS_DIM_CHOICES) / sizeof(SYS_DIM_CHOICES[0]);
constexpr uint8_t TFT_BACKLIGHT_DIM  = 26;  // 디밍 밝기 (약 10%)

constexpr unsigned long DOUBLE_CLICK_MS   = 400UL;
constexpr unsigned long SINGLE_CLICK_WAIT = 400UL;
constexpr unsigned long LONG_PRESS_MS     = 3000UL;  // 시스템 설정 메뉴 진입

// 화면 갱신 주기. 480x320 재드로우가 100ms 넘게 걸리므로 모터가 도는 중에는
// 느리게 갱신해 메인 루프가 오래 멈추지 않게 한다.
constexpr unsigned long DISPLAY_UPDATE_MS        = 100UL;  // 대기 중
constexpr unsigned long DISPLAY_UPDATE_MS_MOVING = 400UL;  // 모터 구동 중

// WiFi 상시 연결(설정 메뉴 "Wi-Fi", NVS 저장)의 처음 기본값. 켜져 있으면 Telnet 로그와
// 무선 업로드를 쓸 수 있고, 업로드는 모터가 멈춘 화면에서만 받는다.
constexpr bool WIFI_DEFAULT_ON = true;
constexpr unsigned long WIFI_RETRY_MS = 30000UL;            // 접속 실패 후 재시도
constexpr unsigned long OTA_CONNECT_TIMEOUT_MS = 20000UL;   // 공유기 접속 대기
constexpr unsigned long OTA_IDLE_TIMEOUT_MS    = 300000UL;  // 업로드 화면 대기

constexpr uint32_t WDT_TIMEOUT_MS = 5000;   // loop() 태스크 워치독

// ─── 시스템 상태 ──────────────────────────────────────────
enum SystemState : uint8_t {
  STATE_HOMING,
  STATE_READY,
  STATE_SET_VOL,
  STATE_SET_TIME,
  STATE_SET_NEEDLE,
  STATE_ALIGNING,
  STATE_ALIGNED_READY,
  STATE_RUNNING,
  STATE_FINISHED,
  STATE_STOPPED,
  STATE_JOG_FWD,   // 호밍을 FWD로 멈춘 뒤 FWD를 다시 누르면 수동 전진
  STATE_OTA,       // 무선 업로드 진행 화면
  STATE_SYSMENU,   // 시스템 설정 메뉴 (엔코더 3초 롱프레스)
};

// ─── 시스템 설정 메뉴 항목 (순서 = LvglScreens.cpp 라벨 배열 순서) ──
enum SysMenuItem : uint8_t {
  SYS_WIFI,        // WiFi 상시 연결 On/Off
  SYS_BRIGHT,      // 화면 밝기
  SYS_DIM_TIME,    // 디밍까지의 시간
  SYS_POWER,       // 전압·전류 표시 On/Off
  SYS_NEEDLE,      // 니들 선택 화면 On/Off (끄면 설정·메인·완료 화면에서 니들이 빠짐)
  SYS_INFO,        // IP 표시, 진단 리포트 출력
  SYS_EXIT,
  SYS_ITEM_COUNT
};

// ─── STOPPED 진입 원인 ────────────────────────────────────
enum StopReason : uint8_t {
  STOP_NONE,
  STOP_LIMIT,         // 리밋 스위치 / 소프트 리밋
  STOP_OVERCURRENT,
  STOP_USER,          // CANCEL·FWD·RETURN으로 사용자 정지
  STOP_BATTERY,       // 저배터리 / 센서 이상
  STOP_HOMING_FAIL,
  STOP_MOVE_TIMEOUT,  // 정렬 이동 시간 초과
  STOP_WATCHDOG,      // 워치독 리셋 후 재부팅
  STOP_INCOMPLETE,    // 설정 시간 안에 목표 거리를 못 채움
  STOP_PAUSE_TIMEOUT, // 일시정지 방치
  STOP_BROWNOUT,      // 전원 붕괴 리셋
  STOP_PANIC,         // 펌웨어 크래시 후 재부팅
  STOP_SERVO_STALL,   // 막힘 (토크 제한에 걸려 못 밂)
  STOP_SERVO_FAULT,   // 서보 무응답·오류 비트·피드백 끊김
};

// ─── 설정 (NVS에 통째로 저장) ─────────────────────────────
// 필드를 바꾸면 Injection.ino의 EEPROM_MAGIC도 올릴 것.
struct InjectionConfig {
  uint8_t  volumeMl  = 50;    // 주입 용량 (mL)
  uint16_t timeSec   = 300;   // 주입 시간 (초)
  uint8_t  needleG   = 23;    // 바늘 규격 (G)
  uint8_t  brightPct = 100;   // 화면 밝기 (%)
  uint16_t dimSec    = 300;   // 디밍까지 (초), 0=디밍 안 함
};

// ─── 설정값 범위 (UI 증감 한계 + 저장값 검증 공용) ─────────
constexpr uint8_t  CFG_VOL_MIN_ML   = 10;
constexpr uint8_t  CFG_VOL_MAX_ML   = 50;
constexpr uint8_t  CFG_VOL_STEP_ML  = 10;
constexpr uint16_t CFG_TIME_MIN_SEC = 60;
constexpr uint16_t CFG_TIME_MAX_SEC = 600;
constexpr uint16_t CFG_TIME_STEP_SEC = 10;
constexpr uint8_t  CFG_NEEDLE_MIN_G = 21;
constexpr uint8_t  CFG_NEEDLE_MAX_G = 25;
