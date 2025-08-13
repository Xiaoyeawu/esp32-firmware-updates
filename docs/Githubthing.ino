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
int resetButtonHandleTime = 60 * 1000;

bool updateNeeded = false;

unsigned long lastCheck = 0;
unsigned long checkInterval = 1 * 60* 1000; // 1 hour in ms
String lastAvailableVersion = "";

WiFiManager wm;

// Your HTTPS server hosting firmware (use RAW GitHub URLs)
const char* currentVersion = "1.0.0.16"; // Only changes when you compile a new bin
const char* versionURL = "https://raw.githubusercontent.com/Xiaoyeawu/esp32-firmware-updates/main/docs/version.txt";
const char* firmwareBaseURL = "https://raw.githubusercontent.com/Xiaoyeawu/esp32-firmware-updates/main/docs/releases/";
String firmwareURL = String(firmwareBaseURL) + "firmware.bin";


void performFirmwareUpdate(const char* firmwareURL) {
  HTTPClient http;
  http.begin(firmwareURL);
  int httpCode = http.GET();

  if (httpCode == 200) {
    int contentLength = http.getSize();
    WiFiClient* stream = http.getStreamPtr();

    if (Update.begin(contentLength)) {
      size_t written = Update.writeStream(*stream);
      if (written == contentLength && Update.end()) {
        Serial.println("[OTA] Update complete. Rebooting...");
        ESP.restart();
      } else {
        Serial.println("[OTA] Update failed.");
      }
    } else {
      Serial.println("[OTA] Not enough space for update.");
    }
  } else {
    Serial.printf("[OTA] Failed to fetch firmware, HTTP code: %d\n", httpCode);
  }
  http.end();
}

bool isNewerVersion(String current, String available) {
  current.replace(".", "");
  available.replace(".", "");
  return available.toInt() > current.toInt();
}


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
  // Serial.println("[OTA] Ready for updates.");
  // checkForUpdate();
  Serial.print("Firmware version : ");
  Serial.println(currentVersion);
}

void loop() {
  ArduinoOTA.handle();
  wm.process(); // Keep WiFiManager running in non-blocking mode

  unsigned long now = millis();
if ((now - lastCheck >= checkInterval || lastCheck == 0) && WiFi.status() == WL_CONNECTED) {
    lastCheck = now;
    
    String availableVersion = getAvailableVersion();
    Serial.println(availableVersion);
    if (availableVersion != lastAvailableVersion) {
        Serial.print("[OTA] Available version from server: ");
        Serial.println(availableVersion);
        lastAvailableVersion = availableVersion;
    }

    if (isNewerVersion(currentVersion, availableVersion)) {
        Serial.println("[OTA] New firmware is available!");
        Serial.println("[OTA] Type 'update' in Serial Monitor to start update.");
        updateNeeded = true;
        firmwareURL = String(firmwareBaseURL) + availableVersion + "/firmware.bin";  // Build dynamic URL
    } else {
        Serial.println("[OTA] No newer firmware available.");
    }
}

// // Wait for user confirmation
// if (updateNeeded && Serial.available()) {
//     String cmd = Serial.readStringUntil('\n');
//     cmd.trim();
//     if (cmd.equalsIgnoreCase("")) {
//         Serial.println("[OTA] Starting update...");
//         performFirmwareUpdate(firmwareURL.c_str());
//         updateNeeded = false;
//     } else {
//         Serial.println("[OTA] Update canceled.");
//         updateNeeded = false;
//     }
// }
// Wait for user confirmation
  if (updateNeeded) {
      if (true) {
          Serial.println("[OTA] Starting update...");
          performFirmwareUpdate(firmwareURL.c_str());
          updateNeeded = false;
      } else {
          Serial.println("[OTA] Update canceled.");
          updateNeeded = false;
      }
  }


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
    blinkLED(50);
    if (buttonPressStart == 0) {
      buttonPressStart = millis();
      Serial.println("[BUTTON] Press detected...");
    }
    if (millis() - buttonPressStart > resetButtonHandleTime && !resettingWiFi) {
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

void checkForUpdate() {
  HTTPClient http;
  http.begin(versionURL);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String availableVersion = http.getString();
    availableVersion.trim(); // Remove any \n or spaces
    Serial.printf("[OTA] Current: %s, Available: %s\n", currentVersion, availableVersion.c_str());

    if (availableVersion != currentVersion) {
      Serial.println("[OTA] New firmware found, starting update...");

      // Build firmware URL dynamically
      String firmwareURL = String(firmwareBaseURL) + availableVersion + "/firmware.bin";
      performFirmwareUpdate(firmwareURL.c_str());
    } else {
      Serial.println("[OTA] Firmware is already up to date.");
    }
  } else {
    Serial.printf("[OTA] Failed to fetch version.txt, HTTP code: %d\n", httpCode);
  }
  http.end();
}

void downloadFirmware() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (https.begin(client, firmwareURL)) {
    int httpCode = https.GET();
    if (httpCode == 200) {
      int contentLength = https.getSize();
      bool canBegin = Update.begin(contentLength);
      if (canBegin) {
        Serial.println("[OTA] Updating...");
        size_t written = Update.writeStream(https.getStream());
        if (written == contentLength) {
          Serial.println("[OTA] Written complete");
        } else {
          Serial.printf("[OTA] Written only %d/%d bytes\n", written, contentLength);
        }
        if (Update.end()) {
          if (Update.isFinished()) {
            Serial.println("[OTA] Update successful, rebooting...");
            ESP.restart();
          } else {
            Serial.println("[OTA] Update failed!");
          }
        }
      } else {
        Serial.println("[OTA] Not enough space for update");
      }
    } else {
      Serial.printf("[OTA] HTTP error: %d\n", httpCode);
    }
    https.end();
  }
}

String getAvailableVersion() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (https.begin(client, versionURL)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      String version = https.getString();
      version.trim();
      https.end();
      return version;
    }
    https.end();
  }
  return currentVersion; // fallback
}

