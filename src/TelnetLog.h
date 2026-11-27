#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════
// TelnetLog — Serial 출력을 USB와 Telnet(포트 23) 양쪽으로 보낸다.
//
// 로그를 찍는 .cpp 파일 맨 위(모든 #include 다음)에
//     #define Serial LOG
// 를 두면 그 파일의 Serial.print가 양쪽으로 나간다. #include보다 위에 두면
// 라이브러리 헤더 안의 Serial까지 바뀌어 부팅이 깨진다.
// WiFi 연결 뒤 telnetLogBegin(), loop()에서 telnetLogTick()을 부른다.
// ══════════════════════════════════════════════════════════

class TelnetMirror : public Print {
public:
  void   begin(unsigned long baud);   // 실제 Serial.begin으로 위임
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;
};

extern TelnetMirror LOG;

void telnetLogBegin();
void telnetLogStop();
void telnetLogTick();
bool telnetLogNewClient();   // 새 클라이언트가 붙었으면 한 번만 true
