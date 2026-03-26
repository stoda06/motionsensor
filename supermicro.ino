// ================================================================
// ESP32-S3 — Hardware Watchdog Supervisor (FINAL FIXED)
// ================================================================

#include <Arduino.h>
#include "esp_timer.h"

// ================= Pins =================
#define KICK_INPUT_PIN   8
#define RESET_OUTPUT_PIN 9

// ================= Config =================
#define KICK_TIMEOUT_MS        15000
#define RESET_PULSE_MS           500
#define POST_RESET_LOCKOUT_MS  20000
#define SERIAL_REPORT_MS        5000

// ================= Globals =================
volatile uint64_t lastKickTimeUs = 0;
volatile bool     lastKickState  = false;

unsigned long lastReport     = 0;
unsigned long resetStartedAt = 0;
unsigned long resetCount     = 0;

bool inReset   = false;
bool inLockout = false;

// ================= ISR =================
void IRAM_ATTR onKickChange() {
  lastKickState  = digitalRead(KICK_INPUT_PIN);
  lastKickTimeUs = esp_timer_get_time();
}

// ================= Safe Time Read =================
unsigned long getKickAgeMs() {
  uint64_t nowUs;
  uint64_t lastKickCopy;

  noInterrupts();
  lastKickCopy = lastKickTimeUs;
  interrupts();

  nowUs = esp_timer_get_time();

  unsigned long age = (nowUs - lastKickCopy) / 1000;

  if (age > 60000) {
    Serial.println("[WDT] ⚠️ Ignoring corrupt timestamp");
    return 0;
  }

  return age;
}

// ================= Reset =================
void triggerReset() {
  resetCount++;
  inReset = true;
  resetStartedAt = millis();

  Serial.printf(
    "[WDT] ⚡ RESET triggered (#%lu) — no kick for %lums\n",
    resetCount,
    getKickAgeMs()
  );

  digitalWrite(RESET_OUTPUT_PIN, LOW);
}

void releaseReset() {
  digitalWrite(RESET_OUTPUT_PIN, HIGH);
  inReset   = false;
  inLockout = true;

  Serial.printf(
    "[WDT] ✅ EN released — lockout %dms before resuming watch\n",
    POST_RESET_LOCKOUT_MS
  );
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== WATCHDOG SUPERVISOR BOOT ===");

  pinMode(RESET_OUTPUT_PIN, OUTPUT);
  digitalWrite(RESET_OUTPUT_PIN, HIGH);

  pinMode(KICK_INPUT_PIN, INPUT_PULLDOWN);

  lastKickState  = digitalRead(KICK_INPUT_PIN);
  lastKickTimeUs = esp_timer_get_time();

  attachInterrupt(digitalPinToInterrupt(KICK_INPUT_PIN), onKickChange, CHANGE);

  Serial.printf("[WDT] Watching GPIO%d → reset GPIO%d\n",
                KICK_INPUT_PIN, RESET_OUTPUT_PIN);

  Serial.printf("[WDT] Timeout: %dms | Reset pulse: %dms | Lockout: %dms\n",
                KICK_TIMEOUT_MS, RESET_PULSE_MS, POST_RESET_LOCKOUT_MS);
}

// ================= Loop =================
void loop() {
  unsigned long now = millis();
  unsigned long kickAge = getKickAgeMs();

  if (inReset && (now - resetStartedAt >= RESET_PULSE_MS)) {
    releaseReset();
    return;
  }

  if (inLockout) {
    if (now - resetStartedAt >= RESET_PULSE_MS + POST_RESET_LOCKOUT_MS) {
      inLockout      = false;
      lastKickTimeUs = esp_timer_get_time();
      Serial.println("[WDT] 👁️ Lockout complete — resuming watch");
    }
    return;
  }

  if (!inReset && kickAge >= KICK_TIMEOUT_MS) {
    Serial.printf("[WDT] ⚠️ No kick for %lums (state=%s) — triggering reset\n",
                  kickAge,
                  lastKickState ? "HIGH" : "LOW");
    triggerReset();
    return;
  }

  if (now - lastReport >= SERIAL_REPORT_MS) {
    lastReport = now;
    Serial.printf("[WDT] OK — last kick %lums ago | resets: %lu | pin: %s\n",
                  kickAge,
                  resetCount,
                  lastKickState ? "HIGH" : "LOW");
  }
}
