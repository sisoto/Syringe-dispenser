#include "Platform.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

// ── GPIO ── (GPIO.in 구조체 대신 안정적인 레지스터 주소를 읽는다)
bool IRAM_ATTR fastGpioRead(uint8_t pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG)  >> pin) & 0x1;
  return         (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 0x1;
}

uint32_t IRAM_ATTR fastGpioIn0_31() {
  return REG_READ(GPIO_IN_REG);
}

// ── PWM ──
void pwmAttach(uint8_t pin, uint32_t freqHz, uint8_t resBits) {
  if (!ledcAttach(pin, freqHz, resBits)) {
    Serial.print("WARN: ledcAttach 실패, pin=");
    Serial.println(pin);
  }
}
void pwmWrite(uint8_t pin, uint32_t duty) {
  ledcWrite(pin, duty);
}

// ── 태스크 워치독 ──
static bool g_wdtArmed = false;
void wdtBegin(uint32_t timeoutMs) {
  // Arduino 3.x는 부팅 때 TWDT를 이미 초기화해 두므로 타임아웃만 바꾼다
  // (TWDT가 꺼진 sdkconfig라면 init으로 대체).
  esp_task_wdt_config_t cfg = {
    .timeout_ms     = timeoutMs,
    .idle_core_mask = 0,      // idle 태스크는 감시하지 않음
    .trigger_panic  = true,   // 물리면 패닉 → 리셋 사유 TASK_WDT
  };
  esp_err_t err = esp_task_wdt_reconfigure(&cfg);
  if (err != ESP_OK) err = esp_task_wdt_init(&cfg);
  if (err != ESP_OK) {
    Serial.print("WARN: TWDT 설정 실패 err=");
    Serial.println((int)err);
  }
  esp_task_wdt_add(NULL);   // 지금 태스크(loopTask) 등록. 이미 등록돼 있으면 무시됨
  g_wdtArmed = true;
}

// 등록 전에 먹이면 IDF가 "task not found"를 찍으므로 등록 뒤에만 먹인다.
void wdtFeed() {
  if (g_wdtArmed) esp_task_wdt_reset();
}

// ── 리셋 사유 ──
bool resetWasWatchdog() {
  esp_reset_reason_t r = esp_reset_reason();
  return (r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT);
}

bool resetWasBrownout() {
  return esp_reset_reason() == ESP_RST_BROWNOUT;
}

bool resetWasPanic() {
  return esp_reset_reason() == ESP_RST_PANIC;
}

const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXTERNAL";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

// ── 힙 ──
void logHeapInfo(const char* tag) {
  Serial.print("[HEAP] ");
  Serial.print(tag);
  Serial.print(" internal free=");
  Serial.print((unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.print(" largest=");
  Serial.print((unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  Serial.print(" psram free=");
  Serial.println((unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
