# Injection — 주사기 디스펜서 (STS3250 서보)

ESP32-S3 N16R8 + Feetech STS3250 버스 서보 + 4.0" ST7796S(MSP4031) 화면으로 주사기를 설정 시간 동안 일정하게 밀어 주는 펌웨어입니다. 전체 배선도는 [`wiring.png`](wiring.png)에 있습니다.

## 1. 배선

| ESP32-S3 | 연결 | 비고 |
|---|---|---|
| GPIO9 (UART1 TX) · J1-15 | 서보 어댑터 **TX** | Waveshare 문서: "TX-TX, RX-RX" (교차 아님) |
| GPIO10 (UART1 RX) · J1-16 | R1 1kΩ → 어댑터 **RX** | 어댑터 RX 전압이 3.3V인지 먼저 측정 |
| GPIO18 / GPIO17 | INA228 SDA / SCL | 주소 0x40 |
| GPIO8 · J1-12 | INA228 ALERT | 오픈 드레인, 내부 풀업 |
| GPIO4·5·6·7·15·16 | TFT CS·RST·RS(DC)·MOSI·SCK·LED | 모듈 3~8번이 J1-4~9에 같은 순서 |
| GPIO13 / GPIO14 | HOME / FRONT 리밋 | NO 결선(COM→GPIO, NO→GND) |
| GPIO1 / 2 / 42 | EC11 A / B / PUSH | |
| GPIO41 | CANCEL(KO) | 인터럽트 |
| GPIO40 / 39 | FWD / RETURN 스위치 | 모멘터리 |
| GPIO47 / 21 | FWD / RETURN LED | 액티브 로우 |

```text
26650 ─ BMS ─ F1 10A ─ INA228(R015) ─┬─ MP3429 9V ─┬─ 어댑터 DC ── STS3250
                                     │             └─ C1 1000µF/25V ─ GND
                                     └─ 5V 승압 ──── ESP32-S3 5Vin / TFT VCC
GND 전부 공통 (ESP32 ↔ 어댑터 ↔ 9V 전원)
```

| 기호 | 부품 | 위치 | 역할 |
|---|---|---|---|
| F1 | 퓨즈 10A | BMS P+ 바로 뒤, INA228 VIN+ 앞 (+ 선에 직렬) | 배선 단락 시 마지막 보호 |
| C1 | 전해 1000µF · 25V | 서보 어댑터 DC 입력에 병렬 | 서보 전류 펄스 흡수, 9V 처짐 완화. 극성 주의 |
| R1 | 1kΩ | 어댑터 RX ↔ GPIO10 직렬 | ESP32 RX 핀 보호 |
| R2·R3 | 1kΩ × 2 (조건부) | GPIO10 ↔ GND, 직렬 2kΩ | 어댑터 RX가 5V로 측정될 때만(분압) |

- 어댑터 점퍼는 **MCU UART 제어 위치**입니다. 어댑터의 USB-C는 쓰지 않습니다.
- C1은 배터리 쪽에 달지 않습니다(켜는 순간 돌입 전류로 BMS가 차단할 수 있음). F1은 + 쪽에 넣습니다.
- TFT 9번 SDO, 10~13번 터치, 14번 SD_CS는 연결하지 않습니다. 화면 방향·색·백라이트 극성은 `Theme.h`의 `TFT_ROTATION`, `TFT_INVERT`, `TFT_RGB_ORDER`, `TFT_BL_INVERT`에서 바꿉니다.
- 서보 출력축 → 커플러 → T8 리드스크류(리드 4mm) 직결 기준 1024 step/mm입니다.

## 2. 구동 방식

- 서보를 **정속(휠) 모드**로 쓰고 위치 제어는 ESP32가 합니다(서보 위치 모드는 1회전 남짓 범위라 90mm 스트로크를 다룰 수 없음).
- 서보 엔코더를 **10ms마다** 읽어 다회전 위치로 이어 붙이고, 사다리꼴 시간 계획(`ServoProfile.h`)의 기준 위치와 차이를 비례·적분 보정해 속도 명령을 냅니다. 보정량은 저역 필터를 거쳐 소음을 줄입니다.
- 이 루프는 별도 FreeRTOS 태스크(코어 1)에서 돌아 화면 갱신과 상관없이 KO·리밋·위치를 감시합니다.
- 주입 목표는 **주사기가 비는 위치**(`SYRINGE_EMPTY_MM`, 이후 학습값)입니다. 목표에 도착했는데 전방 리밋이 안 눌렸으면 최대 3mm까지 더 밀어 리밋을 찾고, 리밋이 눌린 위치를 NVS에 저장해 다음 주입의 목표로 씁니다.
- 끝 무렵(남은 거리 ≤ max(3mm, 주입 거리의 5%)) 전방 리밋이 눌리면 정상 완료로 처리합니다.
- 토크 제한·가속도는 SRAM에만 쓰고 매 부팅 다시 적용합니다. 동작 모드(휠)는 서보가 잠금 상태 쓰기를 무시할 때만 한 번 저장합니다.
- 프로토콜은 `StsProtocol.h`에 직접 구현했습니다(Feetech 공식 `SMS_STS.h` 레지스터 기준).

## 3. 안전 동작

| 상황 | 동작 | 화면 |
|---|---|---|
| CANCEL(KO) | 10ms 안에 속도 0, 토크 유지 | Cancelled by user |
| 이동 방향 리밋 | 즉시 정지, 원점 무효 | Motor Stopped |
| 반대쪽 리밋이 새로 눌림 | 회전 방향 반대 → 정지 + `SERVO_DIR_INVERT` 안내 로그 | Motor Stopped |
| 출력 포화 상태로 1mm 넘게 0.5초 뒤처짐 | 바늘 막힘·주사기 끝·기구 걸림 → 정지 | Plunger Blocked |
| 서보 무응답 5회 / 오류 비트 3회 연속, 피드백 200ms 공백 | 정지, 재설정 필요 | Servo Error |
| `loop()`가 2초 이상 멈춤 | 서보 태스크가 스스로 정지 | — |
| 배터리 전류 7A 초과 (INA228 ALERT) | GPIO8 LOW 래치 → 인터럽트 → 10ms 안에 정지 + 토크 해제 | Overcurrent |
| 배터리 전류 7A 초과 (50ms 측정) | ALERT가 동작하지 않을 때의 예비 | Overcurrent |
| 구동 중 3.0V 이하 0.5초 / 잔량 5% 이하 | 정지 | Low Battery |

- ALERT 하강 에지 후 20µs 뒤에도 LOW일 때만 과전류로 인정합니다(배선 잡음 거름). 걸러진 잡음은 `[OC] ALERT 잡음 무시` 로그로 남습니다.
- 부팅 로그에 `[INA228] ALERT armed: GPIO8 trip 7000mA (SOVL 21000 = 105.0mV) averaged`가 나오면 정상입니다.
- **서보에는 통신 끊김 시 멈추는 기능이 없습니다.** ESP32가 리셋되면 재부팅이 끝날 때까지 서보는 마지막 속도로 돕니다. 그래서 `setup()`의 첫 동작으로 정지 명령을 보냅니다.

## 4. 첫 구동 순서

1. **통신 확인**: 부팅 배너(시리얼 115200 또는 Telnet 23)의 `[BOOT] servo` 줄에서 `OK`, 모델 2825, 전압을 확인합니다. `NOT READY`면 9V 전원, 어댑터 점퍼, TX-TX/RX-RX, GND 공통, ID/보율을 보세요.
2. **방향 확인**: 피스톤을 중간쯤에 두고 켭니다. 호밍이 HOME 쪽으로 가는지 보세요. 반대로 가면 리밋에서 멈추고 `SERVO_DIR_INVERT를 바꿀 것` 로그가 나옵니다.
3. **이동량 확인**: 정렬 후 실제 이동 거리를 자로 재서 `LEAD_PITCH_MM`과 `Theme.cpp`의 용량별 정렬 위치(60/45/30/15/0mm)를 확인합니다.
4. **토크·전류 확인**: 실제 주사기로 시험하며 `[SERVO]` 로그(1초마다 pos/ref/err/cmd/load/V/Ibat/max)를 봅니다. 막힘 오탐이 나면 `SERVO_TORQUE_LIMIT`를 올리고, 9V 승압이 처지거나 과전류가 뜨면 내립니다.

## 5. 주요 설정 (`Theme.h`)

| 상수 | 값 | 의미 |
|---|---|---|
| `PIN_SERVO_TX` / `PIN_SERVO_RX` | 9 / 10 | UART1 |
| `SERVO_ID` / `SERVO_BAUD` | 1 / 1,000,000 | STS 공장값 |
| `SERVO_ECHO` | -1 | 어댑터 에코 자동 판별 (0/1로 강제 가능) |
| `LEAD_PITCH_MM` | 4.0 | 리드 |
| `SERVO_DIR_INVERT` | true | 서보 회전과 피스톤 방향이 반대 |
| `SERVO_TORQUE_LIMIT` | 700 (70%) | 0~1000, 실제로는 PWM 출력 상한 |
| `SERVO_MAX_SPEED` / `SERVO_ACCEL` | 3000 step/s / 4096 step/s² | ≈2.9mm/s, 4mm/s² |
| `SERVO_KP` / `SERVO_KI` / `SERVO_MAX_CORR` | 2.0 / 1.0 / 1024 | 위치 보정 |
| `SERVO_CORR_TAU_S` / `SERVO_CMD_DEADBAND` | 0.15 / 2 | 소음 저감 필터 |
| 이동 속도 (mm/s) | 호밍 2.0, 정렬 2.0, 백오프 1.0, 조그 1.0 | |
| `SYRINGE_EMPTY_MM` | 76.2 | 주사기가 비는 위치(학습 전 기본값, 학습은 ±4mm) |
| `FRONT_END_TOL_MM` / `DISPENSE_COMPLETE_TOL_PCT` | 3.0 / 5 | 끝 무렵 리밋 = 완료 범위 |
| `FRONT_SEEK_MAX_MM` / `FRONT_SEEK_SPEED_MM_S` | 3.0 / 1.0 | 목표 도착 후 리밋 찾기 |
| `OVERCURRENT_MA` / `INA228_ALERT_MA` | 7000 | 배터리 전체 전류 차단 |

바늘별 최소 주입 시간(`NeedleTimeLimits.h`): 21G 1:00, 22G 1:00, 23G 1:20, 24G 1:40, 25G 2:00.

니들 선택은 시스템 설정 메뉴(엔코더 3초 길게 누르기)의 **Needle** 항목에서 켜고 끕니다. 처음 기본값은 Off(`NEEDLE_SELECT_DEFAULT_ON`)입니다.
- Off: 설정이 용량 → 시간 2단계, 메인·완료 화면에 니들 없음, 최소 주입 시간 2:00(`NO_NEEDLE_MIN_TIME_SEC`)
- On: 용량 → 니들 → 시간 3단계, 위의 바늘별 최소 시간 적용
- Off로 바꿀 때 설정 시간이 2:00보다 짧으면 2:00으로 올려 저장합니다.

## 6. 라이브러리 / 빌드

- Arduino-ESP32 3.3.x / ESP32S3 Dev Module / Flash 16MB QIO / PSRAM OPI / Partition Custom(`partitions.csv`)
- LovyanGFX 1.2.x, lvgl 8.3.11, INA228(Rob Tillaart) 0.4.1
- 적용되는 `lv_conf.h`에서: `LV_COLOR_DEPTH 16`, `LV_USE_IMG / LV_USE_LABEL / LV_USE_LINE 1`, `LV_MEM_CUSTOM 0`이면 `LV_MEM_SIZE ≥ 128KB`, `LV_TICK_CUSTOM 1`(millis). `LV_COLOR_16_SWAP`은 0/1 어느 쪽이든 자동 처리됩니다. 조건이 안 맞으면 `UiBuildConfig.h`가 컴파일 오류로 알려 줍니다.
- `Secrets.example.h`를 복사해 `Secrets.h`로 이름을 바꾼 뒤 WiFi SSID·비밀번호·OTA 비밀번호를 채웁니다(`Secrets.h`는 `.gitignore`로 저장소에서 제외됩니다). `WIFI_SSID`가 비어 있으면 Wi-Fi를 켜지 않고(설정 메뉴에 "On (not set up)"), 업로드는 USB로만 할 수 있습니다. 값을 채운 `Secrets.h`는 공유하지 마세요.
