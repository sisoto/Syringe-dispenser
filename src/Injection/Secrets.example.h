#pragma once

// ══════════════════════════════════════════════════════════
// Secrets.h — WiFi / 무선 업로드 정보 (직접 채워 넣으세요)
//
// WIFI_SSID를 비워 두면 Wi-Fi를 켜지 않습니다(설정 메뉴에 "On (not set up)").
// 이 경우 펌웨어 업로드는 USB로만 할 수 있습니다.
//
// 주의: ESP32-S3는 2.4GHz만 지원합니다. 공유기가 대역을 나눠 두었다면
//       2.4GHz 쪽 SSID를 적으세요.
// 이 파일에 값을 채운 뒤에는 다른 사람과 공유하지 마세요.
// ══════════════════════════════════════════════════════════

#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""

// Arduino IDE의 네트워크 포트 목록에 이 이름으로 뜹니다.
#define OTA_HOSTNAME   "syringe-dispenser"

// 무선 업로드 비밀번호. 같은 네트워크의 누구나 펌웨어를 덮어쓸 수 있으므로
// Wi-Fi를 쓴다면 반드시 본인만 아는 값으로 바꾸세요.
#define OTA_PASSWORD   "change-me"
