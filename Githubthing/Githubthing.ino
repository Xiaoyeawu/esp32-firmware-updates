#include <WiFiManager.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

#define LED_BUILTIN_PIN 2
#define BUTTON_BUILTIN_PIN 0

unsigned long buttonPressStart = 0;
bool ledState = false;
bool pairingMode = false;
bool resettingWiFi = false;

WiFiManager wm;

// Your HTTPS server hosting firmware
const char* versionURL = "https://your-server.com/version.txt";
const char* firmwareURL = "https://your-server.com/firmware.bin";
const char* currentVersion = "1.0.0";  // Change when you release a new build

void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32 WROVER starting...");
  delay(500);

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);
  pinMode(BUTTON_BUILTIN_PIN, INPUT_PULLUP);

  // Button hold at boot → reset WiFi
  if (digitalRead(BUTTON_BUILTIN_PIN) == LOW) {
    Serial.println("[BOOT] Button held → Clearing WiFi credentials...");
    wm.resetSettings();
    delay(1000);
  }

  // If WiFi connects quickly, skip AP mode
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 3000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Starting AP for pairing...");
    pairingMode = true;
    wm.setConfigPortalBlocking(false); // Non-blocking mode
    wm.startConfigPortal("ESP32_Setup", "12345678");
  }

  ArduinoOTA.setPassword("19980719");
  ArduinoOTA.setHostname("ESP32_WROVER");
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready for updates.");
}

void loop() {
  ArduinoOTA.handle();
  wm.process(); // Keep WiFiManager running in non-blocking mode

  if (resettingWiFi) {
    blinkLED(50); // Rapid blink during reset
  }
  else if (pairingMode && wm.getConfigPortalActive()) {
    doubleFlashPattern(100, 300); // Pairing mode double flash
  }
  else if (WiFi.status() != WL_CONNECTED) {
    blinkLED(200); // Disconnected
  }
  else {
    digitalWrite(LED_BUILTIN_PIN, HIGH); // Connected solid
  }

  // If AP mode finishes and WiFi connects → exit pairing mode
  if (pairingMode && !wm.getConfigPortalActive() && WiFi.status() == WL_CONNECTED) {
    pairingMode = false;
    Serial.println("[WiFi] Connected! IP: " + WiFi.localIP().toString());
  }

  // Button handling
  if (digitalRead(BUTTON_BUILTIN_PIN) == LOW) {
    if (buttonPressStart == 0) {
      buttonPressStart = millis();
      Serial.println("[BUTTON] Press detected...");
    }
    if (millis() - buttonPressStart > 3000 && !resettingWiFi) {
      Serial.println("[BUTTON] 3s hold → Resetting WiFi...");
      resettingWiFi = true;
      wm.resetSettings();
      delay(1500);
      ESP.restart();
    }
  } else {
    if (buttonPressStart > 0 && !resettingWiFi) {
      Serial.println("[BUTTON] Released before 3s.");
    }
    buttonPressStart = 0;
  }
}

void blinkLED(unsigned long interval) {
  static unsigned long previousMillis = 0;
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN_PIN, ledState);
  }
}

void doubleFlashPattern(unsigned long flashDelay, unsigned long pauseDelay) {
  static unsigned long lastMillis = 0;
  static int step = 0;
  unsigned long now = millis();

  switch (step) {
    case 0:
      digitalWrite(LED_BUILTIN_PIN, HIGH);
      lastMillis = now;
      step = 1;
      break;
    case 1:
      if (now - lastMillis >= flashDelay) {
        digitalWrite(LED_BUILTIN_PIN, LOW);
        lastMillis = now;
        step = 2;
      }
      break;
    case 2:
      if (now - lastMillis >= flashDelay) {
        digitalWrite(LED_BUILTIN_PIN, HIGH);
        lastMillis = now;
        step = 3;
      }
      break;
    case 3:
      if (now - lastMillis >= flashDelay) {
        digitalWrite(LED_BUILTIN_PIN, LOW);
        lastMillis = now;
        step = 4;
      }
      break;
    case 4:
      if (now - lastMillis >= pauseDelay) {
        step = 0;
      }
      break;
  }
}
