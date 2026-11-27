#include "TelnetLog.h"
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFi.h>

// 이 파일은 실제 Serial에 써야 하므로 #define Serial LOG를 하지 않는다(무한 재귀 방지).

static WiFiServer  _srv(23);
static WiFiClient  _client;
static bool        _started   = false;
static bool        _newClient = false;

TelnetMirror LOG;

void TelnetMirror::begin(unsigned long baud) {
  Serial.begin(baud);
}

size_t TelnetMirror::write(uint8_t c) {
  Serial.write(c);
  if (_client && _client.connected()) _client.write(c);
  return 1;
}

size_t TelnetMirror::write(const uint8_t* buf, size_t size) {
  Serial.write(buf, size);
  if (_client && _client.connected()) _client.write(buf, size);
  return size;
}

void telnetLogBegin() {
  if (_started) return;
  _srv.begin();
  _srv.setNoDelay(true);
  _started = true;
  Serial.println("[TELNET] port 23 open  -- nc <IP> 23");
}

void telnetLogStop() {
  if (_client) _client.stop();
  _srv.stop();
  _started   = false;
  _newClient = false;
}

void telnetLogTick() {
  if (!_started) return;

  if (_srv.hasClient()) {
    WiFiClient nc = _srv.accept();
    if (!_client || !_client.connected()) {
      _client    = nc;
      _newClient = true;
      Serial.println("[TELNET] client connected");

      _client.print("\r\n=== Syringe Dispenser Log ===\r\n");
      _client.print("IP: ");
      _client.print(WiFi.localIP().toString().c_str());
      _client.print("\r\nUptime: ");
      _client.print(millis() / 1000);
      _client.print("s  FreeHeap: ");
      _client.print(ESP.getFreeHeap());
      _client.print(" bytes\r\n--- live log below ---\r\n\r\n");
    } else {
      nc.println("Already connected.");
      nc.stop();
    }
  }

  // 로그 전용이라 클라이언트가 보낸 입력은 버린다.
  while (_client && _client.connected() && _client.available()) _client.read();
}

bool telnetLogNewClient() {
  if (!_newClient) return false;
  _newClient = false;
  return true;
}
