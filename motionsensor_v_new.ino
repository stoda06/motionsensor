// ================================================================
// ESP32-S3 — Motion Sensor + MQTT + Syslog + NTP + OTA
// WiFi self-managed with configurable retry delay
// Reintegrated build with watchdog-safe timing and staged startup
// ================================================================

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <time.h>
// Reduce PubSubClient socket timeout from 15s default to prevent WDT resets
#define MQTT_SOCKET_TIMEOUT 3
#include <PubSubClient.h>
#include "esp_wifi.h"
#include "arduino_secrets.h"

// ================= Logging =================
#define LOG_SERIAL Serial

// ================= Emojis =================
#define EMO_HEARTBEAT   "💓"
#define EMO_MOTION      "🔥"
#define EMO_MQTT_SEND   "📤"
#define EMO_RELAY       "🔌"
#define EMO_FLASH       "✨"
#define EMO_WIFI_OK     "✅"
#define EMO_WIFI_WARN   "⚠️"
#define EMO_WIFI_RETRY  "🔄"
#define EMO_NTP         "⏱️"
#define EMO_OK          "✅"
#define EMO_SCAN        "🔍"

// ================= WiFi Credentials =================
#define WIFI_SSID      SECRET_SSID
#define WIFI_PASSWORD  SECRET_PASS

// ================= Configuration =================
#define WIFI_CONNECT_TIMEOUT_MS  15000
#define WIFI_RETRY_DELAY_MS      15000
#define WIFI_RECONNECT_DELAY_MS  500
#define MQTT_RETRY_DELAY_MS      5000

// ================= OTA =================
#define OTA_HOSTNAME  "motion-sensor"

// ================= PIR =================
#define PIR_PIN          5
#define PIR_ARM_DELAY_MS 30000
#define PIR_ACTIVE_HIGH  1

// ================= Relay =================
#define RELAY_PIN         42
#define RELAY_ACTIVE_HIGH 1

// ================= Watchdog Kick =================
#define WDT_KICK_PIN          15
#define WDT_KICK_INTERVAL_MS  2000
#define WDT_DEBUG_INTERVAL    10000
#define WDT_MAX_DELAY_MS      5000

// ================= MQTT =================
#define MQTT_HOST  "192.168.1.100"
#define MQTT_PORT  1883
#define MQTT_TOPIC "front/motion"

// ================= Syslog =================
#define SYSLOG_HOST       "192.168.1.100"
#define SYSLOG_PORT       514
#define SYSLOG_LOCAL_PORT 5140

// ================= Timing =================
#define HEARTBEAT_INTERVAL 60
#define RELAY_SOLID_MS     (2UL * 60UL * 1000UL)
#define RELAY_FLASH_MS     (2UL * 60UL * 1000UL)
#define RELAY_FLASH_INT    500

// ================= Globals =================
WiFiUDP      udpSyslog;
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long bootMs          = 0;
unsigned long lastHeartbeat   = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastWdtKick     = 0;

// ================= Feature State =================
bool syslogReady      = false;
bool timeSynchronized = false;
bool ntpConfigured    = false;
bool mqttReady        = false;
bool otaStarted       = false;

// ================= Staggered Init =================
unsigned long initStartMs       = 0;
bool wifiInitStarted            = false;
bool servicesInitStarted        = false;

// ================= Watchdog Debug =================
unsigned long lastWdtLog     = 0;
unsigned long lastWdtEdgeMs  = 0;
bool          lastWdtState   = false;

// ================= Loop timing debug =================
unsigned long lastLoopStart = 0;

// ================= WiFi retry tracking =================
unsigned long lastWifiAttempt = 0;
bool wifiWasConnected         = false;

// ================= PIR =================
bool pirArmed   = false;
bool lastMotion = false;

// ================= Relay =================
bool relayActive              = false;
bool relayOutput              = false;
unsigned long relayStartMs    = 0;
unsigned long lastFlashToggle = 0;

// ================= Forward Decls =================
void kickWatchdog();
void logMsg(const String &msg);
void setupOTA();
void setRelay(bool on, bool silent = false);

// ================= Safe Delay =================
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    kickWatchdog();
    delay(1);
  }
}

// ================= Watchdog Kick =================
void kickWatchdog() {
  unsigned long now = millis();

  if (now - lastWdtKick >= WDT_KICK_INTERVAL_MS) {
    lastWdtKick = now;

    bool newState = !digitalRead(WDT_KICK_PIN);
    digitalWrite(WDT_KICK_PIN, newState);

    unsigned long delta = now - lastWdtEdgeMs;
    lastWdtEdgeMs = now;

    if (delta > WDT_MAX_DELAY_MS) {
      logMsg("⚠️ WDT DELAY anomaly: " + String(delta) + " ms");
    }

    lastWdtState = newState;
  }

  if (now - lastWdtLog >= WDT_DEBUG_INTERVAL) {
    lastWdtLog = now;

    logMsg(
      String("🧠 WDT status: state=") +
      (lastWdtState ? "HIGH" : "LOW") +
      " last_toggle_ms=" + String(lastWdtEdgeMs) +
      " now=" + String(now)
    );
  }
}

// ================= Time / Logging =================
String ts() {
  struct tm t;
  if (!timeSynchronized || !getLocalTime(&t, 0)) {
    return "1970-01-01T00:00:00";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
  return String(buf);
}

void logMsg(const String &msg) {
  String ip = (WiFi.status() == WL_CONNECTED)
                ? WiFi.localIP().toString()
                : "0.0.0.0";

  String line = ts() + " " + ip + " [motion-sensor] [INFO] " + msg;
  LOG_SERIAL.println(line);

  if (!syslogReady || WiFi.status() != WL_CONNECTED) return;

  udpSyslog.beginPacket(SYSLOG_HOST, SYSLOG_PORT);
  udpSyslog.print("<134>");
  udpSyslog.print(line);
  udpSyslog.endPacket();
}

// ================= Relay =================
void setRelay(bool on, bool silent) {
  relayOutput = on;
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? on : !on);
  if (!silent) {
    logMsg(String(EMO_RELAY) + " Relay set " + (on ? "ON" : "OFF"));
  }
}

// ================= NTP =================
void checkTimeSync() {
  if (timeSynchronized || !ntpConfigured) return;
  time_t t;
  time(&t);
  if (t > 1600000000) {
    timeSynchronized = true;
    logMsg(String(EMO_OK) + " NTP sync OK");
  }
}

// ================= WiFi =================
void connectWiFi() {
  logMsg("=== connectWiFi() start ===");

  WiFi.disconnect(true);
  safeDelay(100);

  WiFi.mode(WIFI_OFF);
  safeDelay(300);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setHostname("motion-sensor");
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

  safeDelay(100);

  logMsg("MAC: " + WiFi.macAddress());

  // Optional scan path retained but disabled by default for stability
  bool useScan = false;

  int bestChan = 0;
  uint8_t bestBSSID[6];
  bool found = false;

  if (useScan) {
    logMsg(String(EMO_SCAN) + " Scanning for: " + WIFI_SSID);

    int n = WiFi.scanNetworks();
    kickWatchdog();

    if (n == 0) {
      logMsg(String(EMO_WIFI_WARN) + " No networks found");
      return;
    }

    int bestRSSI = -999;

    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID) {
        logMsg(String(EMO_WIFI_OK) + " Found \"" + WIFI_SSID + "\" RSSI: "
               + String(WiFi.RSSI(i)) + " dBm  CH:" + String(WiFi.channel(i))
               + "  BSSID:" + WiFi.BSSIDstr(i));

        if (WiFi.RSSI(i) > bestRSSI) {
          bestRSSI = WiFi.RSSI(i);
          bestChan = WiFi.channel(i);
          memcpy(bestBSSID, WiFi.BSSID(i), 6);
        }
        found = true;
      }
    }

    WiFi.scanDelete();

    if (!found) {
      logMsg(String(EMO_WIFI_WARN) + " \"" + WIFI_SSID + "\" not found in scan");
      return;
    }
  }

  logMsg(String(EMO_WIFI_RETRY) + " Connecting to WiFi: " + WIFI_SSID);

  if (useScan && found) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, bestChan, bestBSSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  WiFi.setAutoReconnect(false);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    safeDelay(500);
    LOG_SERIAL.print(".");
  }

  LOG_SERIAL.println();

  if (WiFi.status() == WL_CONNECTED) {
    logMsg(String(EMO_WIFI_OK) + " WiFi connected, IP "
           + WiFi.localIP().toString());

    wifiWasConnected = true;
    servicesInitStarted = true;
    initStartMs = millis();

    logMsg("WiFi stage complete — services will start shortly");
  } else {
    logMsg(String(EMO_WIFI_WARN) + " WiFi connect failed, will retry in "
           + String(WIFI_RETRY_DELAY_MS / 1000) + "s");
  }

  logMsg("=== connectWiFi() end ===");
}

void manageWiFi() {
  unsigned long now = millis();

  if (!wifiInitStarted) {
    if (now - initStartMs < 3000) return;
    wifiInitStarted = true;
    logMsg("Starting WiFi stage...");
  }

  if (WiFi.status() == WL_CONNECTED) return;

  if (wifiWasConnected) {
    logMsg(String(EMO_WIFI_WARN) + " WiFi lost");
    wifiWasConnected = false;
  }

  if (now - lastWifiAttempt >= WIFI_RETRY_DELAY_MS) {
    lastWifiAttempt = now;
    connectWiFi();
  }
}

// ================= OTA =================
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);

#ifdef OTA_PASSWORD
  ArduinoOTA.setPassword(OTA_PASSWORD);
#endif

  ArduinoOTA.onStart([]() {
    logMsg("⬆️ OTA update starting — suspending relay");
    setRelay(false);
  });

  ArduinoOTA.onEnd([]() {
    logMsg("⬆️ OTA update complete, rebooting");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPct = 0;
    unsigned int pct = (total == 0) ? 0 : (progress / (total / 100 == 0 ? 1 : total / 100));
    if (pct != lastPct && pct % 10 == 0) {
      logMsg("⬆️ OTA progress: " + String(pct) + "%");
      lastPct = pct;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    logMsg(String(EMO_WIFI_WARN) + " OTA error: " + String(error));
  });

  ArduinoOTA.begin();
  otaStarted = true;
  logMsg("⬆️ OTA ready — hostname: " + String(OTA_HOSTNAME));
}

// ================= MQTT =================
void manageMqtt() {
  if (WiFi.status() != WL_CONNECTED || !mqttReady) return;

  if (!mqtt.connected()) {
    if (millis() - lastMqttAttempt < MQTT_RETRY_DELAY_MS) return;
    lastMqttAttempt = millis();

    String cid = "motion-" + WiFi.macAddress();
    kickWatchdog();
    unsigned long mqttConnStart = millis();
    if (mqtt.connect(cid.c_str())) {
      kickWatchdog();
      logMsg(String(EMO_OK) + " MQTT connected");
      mqtt.publish("motion/status", "online", true);
    } else {
      kickWatchdog();
      if (millis() - mqttConnStart > 1000) {
        logMsg("⚠️ MQTT connect blocked: " + String(millis() - mqttConnStart) + " ms");
      }
      logMsg(String(EMO_WIFI_WARN) + " MQTT connect failed (rc="
             + String(mqtt.state()) + ") will retry in "
             + String(MQTT_RETRY_DELAY_MS / 1000) + "s");
    }
  }

  checkTimeSync();
}

// ================= Relay Logic =================
void startRelayCycle() {
  relayActive     = true;
  relayStartMs    = millis();
  lastFlashToggle = millis();
  setRelay(true);
  logMsg(String(EMO_RELAY) + " Relay cycle started — solid for " +
         String(RELAY_SOLID_MS / 1000) + "s");
}

void updateRelay() {
  if (!relayActive) return;

  unsigned long elapsed = millis() - relayStartMs;
  if (elapsed < RELAY_SOLID_MS) return;

  if (elapsed < RELAY_SOLID_MS + RELAY_FLASH_MS) {
    if (millis() - lastFlashToggle >= RELAY_FLASH_INT) {
      if (lastFlashToggle == relayStartMs) {
        logMsg(String(EMO_FLASH) + " Relay entering flash phase — " +
               String(RELAY_FLASH_MS / 1000) + "s");
      }
      lastFlashToggle = millis();
      setRelay(!relayOutput, true);
    }
    return;
  }

  relayActive = false;
  setRelay(false);
  logMsg("🛑 Relay cycle complete");
}

// ================= Setup =================
void setup() {
  delay(5000);

  bootMs = millis();
  initStartMs = millis();

  Serial.begin(115200);

  // Init WDT pin BEFORE safeDelay so kickWatchdog() can use it
  pinMode(WDT_KICK_PIN, OUTPUT);
  digitalWrite(WDT_KICK_PIN, LOW);
  lastWdtEdgeMs = millis();
  lastWdtKick   = millis();

  safeDelay(500);

  pinMode(PIR_PIN, INPUT_PULLDOWN);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, !RELAY_ACTIVE_HIGH);

  logMsg("System initialized (staggered startup)");

  lastWifiAttempt = millis();
}

// ================= Loop =================
void loop() {
  unsigned long now = millis();

  unsigned long loopDelta = now - lastLoopStart;
  lastLoopStart = now;

  if (loopDelta > 1000) {
    logMsg("⚠️ LOOP DELAY: " + String(loopDelta) + " ms");
  }

  if (millis() - lastWdtKick > 4000) {
    logMsg("🔥 ABOUT TO MISS WDT");
  }

  kickWatchdog();
  manageWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    if (servicesInitStarted) {
      if (!mqttReady && now - initStartMs > 2000) {
        logMsg("Starting services (MQTT/NTP/syslog)");

        udpSyslog.begin(SYSLOG_LOCAL_PORT);
        mqtt.setServer(MQTT_HOST, MQTT_PORT);
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");

        ntpConfigured = true;
        syslogReady   = true;
        mqttReady     = true;
      }

      if (!otaStarted && now - initStartMs > 5000) {
        setupOTA();
      }
    }

    // OTA with timing instrumentation
    unsigned long otaStart = millis();
    kickWatchdog();
    ArduinoOTA.handle();
    kickWatchdog();
    if (millis() - otaStart > 1000) {
      logMsg("⚠️ OTA blocking: " + String(millis() - otaStart) + " ms");
    }

    manageMqtt();

    if (mqtt.connected()) {
      unsigned long mqttStart = millis();
      mqtt.loop();
      if (millis() - mqttStart > 1000) {
        logMsg("⚠️ MQTT blocking: " + String(millis() - mqttStart) + " ms");
      }
    }
  }

  if (!pirArmed && millis() > PIR_ARM_DELAY_MS) {
    pirArmed = true;
    logMsg("👁️ PIR armed");
  }

  if (pirArmed) {
    bool motion = (digitalRead(PIR_PIN) == PIR_ACTIVE_HIGH);

    if (motion && !lastMotion) {
      logMsg(String(EMO_MOTION) + " Motion detected!");

      if (mqtt.connected()) {
        mqtt.publish(MQTT_TOPIC, "Motion detected");
        logMsg(String(EMO_MQTT_SEND) + " MQTT sent");
      }

      startRelayCycle();
    }

    lastMotion = motion;
  }

  updateRelay();

  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL * 1000UL) {
    lastHeartbeat = millis();

    if (mqtt.connected()) {
      mqtt.publish("motion/heartbeat", String(WiFi.RSSI()).c_str());
    }

    logMsg(String(EMO_HEARTBEAT) + " Heartbeat");
  }

  safeDelay(50);
}
