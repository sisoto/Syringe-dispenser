# Syringe-dispenser

ESP32-S3 motorized syringe dispenser with 4.0-inch TFT UI. Non-commercial license.

ESP32-S3와 Feetech STS3250 버스 서보로 시린지를 밀어내는 DIY 전동 시린지 디스펜서입니다.
4.0" TFT 화면에서 용량과 시간을 설정하면, 설정한 시간 동안 설정한 용량을 일정한 속도로 토출합니다.

> ⚠️ **주의**
> 개인이 취미로 만든 제작품입니다. 따라 만들거나 사용하면서 생기는 결과에 대한 책임은 사용자 본인에게 있습니다.
> **사람이나 동물을 대상으로 절대 사용하지 마세요.**

---

## 주요 기능

- 용량과 시간을 설정하면 그에 맞춰 일정한 속도로 토출 (사다리꼴 속도 계획 + 위치 보정)
- 리밋 스위치를 이용한 자동 원점 복귀(호밍), 전방 리밋 위치 학습
- LVGL 기반 480x320 UI, 로터리 엔코더 조작
- 배터리 잔량 표시, 과전류(INA228 ALERT 하드웨어 인터럽트)·저전압 차단
- WiFi OTA 펌웨어 업데이트 (필요할 때만 WiFi 켜짐)

## 하드웨어 구성

| 구분 | 부품 |
|---|---|
| 제어 | ESP32-S3-DevKitC-1 N16R8 (Flash 16MB / PSRAM 8MB) |
| 화면·입력 | 4.0" TFT SPI 480x320 (ST7796S, MSP4031), EC11 로터리 엔코더, 버튼 |
| 구동 | Feetech STS3250 버스 서보 + Waveshare Bus Servo Adapter (A), T8 리드스크류(리드 4mm) |
| 전원 | 26650 리튬이온 셀 → MP3429 9V 승압(서보) / 5V 승압(로직) |
| 측정·보호 | INA228 전류/전압 센서(R015), 10A 퓨즈 |

👉 **전체 부품 리스트:** [DIY 시린지 디스펜서 - 부품 리스트 (Notion)](https://app.notion.com/p/421a262d807681ed8dade10bde627520)

## 폴더 구조

```
Syringe-dispenser/
├─ README.md            프로젝트 소개 및 사용 설명
├─ LICENSE              펌웨어 코드 라이선스 (PolyForm Noncommercial 1.0.0)
├─ .gitignore
├─ src/
│  └─ Injection/        Arduino 스케치 폴더 (Injection.ino 외)
├─ hardware/            회로도 및 하드웨어 자료 (CC BY-NC 4.0)
└─ docs/                조립 및 제작 안내서, 배선도 (CC BY-NC 4.0)
   ├─ BUILD-GUIDE-KO.md 배선·구동 방식·안전 동작·첫 구동·설정·빌드 안내
   └─ wiring.png        전체 배선도
```

Arduino IDE는 `.ino` 파일이 같은 이름의 폴더 안에 있어야 열리므로, 스케치는 `src/Injection/` 안에 있습니다.

## 빌드 방법

자세한 배선과 설정은 [docs/BUILD-GUIDE-KO.md](docs/BUILD-GUIDE-KO.md)를 참고하세요.

### 1. 개발 환경

- Arduino IDE 2.x
- 보드 패키지: **Arduino-ESP32 3.3.x**

### 2. 보드 설정 (Tools 메뉴)

| 항목 | 값 |
|---|---|
| Board | ESP32S3 Dev Module |
| Flash Size | 16MB |
| Flash Mode | QIO 80MHz |
| PSRAM | OPI PSRAM |
| Partition Scheme | Custom (스케치 폴더의 `partitions.csv` 사용) |

### 3. 필요한 라이브러리

| 라이브러리 | 테스트한 버전 |
|---|---|
| [LovyanGFX](https://github.com/lovyan03/LovyanGFX) | 1.2.x |
| [lvgl](https://github.com/lvgl/lvgl) | 8.3.11 |
| [INA228](https://github.com/RobTillaart/INA228) (Rob Tillaart) | 0.4.1 |

서보 통신 프로토콜은 `StsProtocol.h`에 직접 구현되어 있어 별도 서보 라이브러리가 필요 없습니다.
`lv_conf.h` 필수 설정은 빌드 안내서의 "라이브러리 / 빌드" 항목을 참고하세요. 설정이 맞지 않으면 `UiBuildConfig.h`가 컴파일 오류로 알려 줍니다.

### 4. WiFi 설정 (Secrets.h)

1. `src/Injection/Secrets.example.h`를 복사해 같은 폴더에 `Secrets.h`로 저장합니다.
2. WiFi SSID, 비밀번호, OTA 비밀번호를 채웁니다.

`Secrets.h`는 `.gitignore`로 저장소에서 제외되어 있습니다. 값을 채운 파일은 공유하지 마세요.
`WIFI_SSID`를 비워 두면 WiFi를 켜지 않고, 업로드는 USB로만 할 수 있습니다.

### 5. 업로드

- 첫 업로드는 USB로 합니다.
- 이후에는 READY 화면에서 엔코더를 5초 길게 눌러 시스템 메뉴 → **WiFi Update**로 들어가면 OTA로 업로드할 수 있습니다.

---

## 라이선스

이 프로젝트는 **소스 공개(Source-available)** 프로젝트입니다.
비상업적 이용은 자유롭지만, **상업적 이용은 사전에 서면 허락을 받아야 합니다.**

| 대상 | 라이선스 |
|---|---|
| 펌웨어 코드 (`src/`) | [PolyForm Noncommercial 1.0.0](LICENSE) |
| 하드웨어 자료 (`hardware/`) | [CC BY-NC 4.0](hardware/LICENSE) |
| 문서·배선도 (`docs/`, 이 README) | [CC BY-NC 4.0](docs/LICENSE) |

- 개인 학습, 연구, 비영리 목적이라면 출처를 밝히고 자유롭게 보고, 따라 만들고, 수정해서 공유할 수 있습니다.
- 판매, 유료 서비스, 제품 탑재 등 상업적 이용은 아래 연락처로 먼저 문의해 주세요.
- 부팅 로고 이미지(`Logo.h`, SISOTO 로고)는 위 라이선스 적용 대상에서 **제외**되며, 무단 사용을 허용하지 않습니다.
- `UiFonts.cpp`에 포함된 폰트는 원 저작자의 라이선스를 따릅니다 ([`src/Injection/assets/FONT-LICENSE.txt`](src/Injection/assets/FONT-LICENSE.txt)).
- LovyanGFX, lvgl, INA228, Arduino-ESP32 등 외부 라이브러리는 각 라이브러리의 라이선스를 따릅니다.

**상업적 이용 문의:** https://www.instagram.com/sisoto.ca

Copyright (c) 2026 sisoto
