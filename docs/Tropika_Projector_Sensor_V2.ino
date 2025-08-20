#include "ACS712.h"
#include "HomeSpan.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>

#define CURRENT_PIN 32
#define ADC_REF_VOLTAGE 3.3
#define ADC_RESOLUTION 4095
#define SENSOR_SENSITIVITY 100   // mV/A for ACS712-20A

#define FACTORY_RESET_PIN 0     // ESP32 built-in BOOT button
#define RESET_HOLD_TIME 10000   // 10 seconds in milliseconds

#define FW_VERSION "1.4"   // current firmware version
const char* versionURL = "https://raw.githubusercontent.com/Xiaoyeawu/esp32-firmware-updates/main/docs/version.txt";
const char* firmwareBaseURL = "https://raw.githubusercontent.com/Xiaoyeawu/esp32-firmware-updates/main/docs/releases/";

String firmwareURL = String(firmwareBaseURL) + String(FW_VERSION) + "/Tropika_Projector_Sensor_V2.ino.bin";


ACS712 ACS(CURRENT_PIN, ADC_REF_VOLTAGE, ADC_RESOLUTION, SENSOR_SENSITIVITY);

SpanCharacteristic *currentChar;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000; // report every 1s

// Sampling state
const int samples = 500;           // how many samples per RMS calc
const unsigned long sampleInterval = 200; // in microseconds (~5kHz)
unsigned long lastSampleTime = 0;

int sampleCount = 0;
double sumOfSquares = 0.0;

void factoryResetTask(void *pvParameters) {
  pinMode(FACTORY_RESET_PIN, INPUT_PULLUP); // built-in button is usually active-low

  for (;;) {
    static unsigned long pressStart = 0;
    if (digitalRead(FACTORY_RESET_PIN) == LOW) {  // button pressed
      if (pressStart == 0) pressStart = millis();
      else if (millis() - pressStart >= RESET_HOLD_TIME) {
        Serial.println("Factory reset triggered!");
        
        // Optional: reset HomeSpan config
        homeSpan.processSerialCommand("F");  // clears HomeKit pairing & WiFi
        delay(1000);

        ESP.restart();  // reboot ESP32
      }
    } else {
      pressStart = 0; // button released
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // check every 50ms
  }
}

void acsSensorTask (void *pvParameters) {
  for (;;){
    while (WiFi.status() == WL_CONNECTED) {
      unsigned long nowMicros = micros();

      if (nowMicros - lastSampleTime >= sampleInterval) {
        lastSampleTime = nowMicros;

        // Non-blocking raw ADC read (use analogRead directly)
        int raw = analogRead(CURRENT_PIN);
        float voltage = raw * (ADC_REF_VOLTAGE / (float)ADC_RESOLUTION);

        // convert midpoint ADC to voltage:
        float midADC = ACS.getMidPoint(); // midPoint in ADC counts (library provides)
        float midVoltage = midADC * (ADC_REF_VOLTAGE / (float)ADC_RESOLUTION);

        // sensitivity in V/A (from mV/A)
        float sensVperA = SENSOR_SENSITIVITY / 1000.0f;

        // instantaneous current (signed)
        float instA = (voltage - midVoltage) / sensVperA;

        sumOfSquares += instA * instA;
        sampleCount++;
      }

      if (sampleCount >= samples) {
        float rmsCurrent = sqrt(sumOfSquares / sampleCount);

        // reset for next round
        sampleCount = 0;
        sumOfSquares = 0;

        // apply calibration factor if you need (keep at 1.0 until calibrated)
        float scaleFactor = 10.0; // your existing factor (tune/keep as needed)
        float amps = rmsCurrent * scaleFactor;

        // update HomeKit every second
        if (millis() - lastUpdate >= updateInterval) {
          lastUpdate = millis();

          if (currentChar) currentChar->setVal(amps);

          Serial.print("RMS Current: ");
          Serial.print(amps, 2);
          Serial.println(" A");
        }
      }

      vTaskDelay(1); // yield
    }
  }
}

String latestVersion = "";  // global

bool checkUpdateAvailable() {
  HTTPClient http;
  http.begin(versionURL);        // points to version.txt
  int httpCode = http.GET();

  if (httpCode == 200) {
    latestVersion = http.getString();
    latestVersion.trim();        // remove whitespace/newlines
    Serial.println("Latest FW: " + latestVersion);
    return (latestVersion != FW_VERSION);
  }

  return false;
}

String getFirmwareURL() {
  return String(firmwareBaseURL) + latestVersion + "/Tropika_Projector_Sensor_V2.ino.bin";
}

void httpUpdate() {
  if (latestVersion == "") return;   // sanity check

  String fwURL = getFirmwareURL();
  Serial.println("[OTA] Downloading firmware from: " + fwURL);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (https.begin(client, fwURL)) {
    int httpCode = https.GET();
    if (httpCode == HTTP_CODE_OK) {
      int contentLength = https.getSize();
      if (contentLength > 0) {
        if (Update.begin(contentLength)) {
          size_t written = Update.writeStream(*https.getStreamPtr());
          Serial.println("[OTA] Written: " + String(written));
          if (Update.end() && Update.isFinished()) {
            Serial.println("[OTA] Update successful! Rebooting...");
            ESP.restart();
          } else {
            Serial.println("[OTA] Update failed.");
          }
        }
      }
    } else {
      Serial.println("[OTA] HTTP Error: " + String(httpCode));
    }
    https.end();
  }
}

// ---------- Trigger OTA via Switch ----------
struct UpdateTrigger : Service::Switch {
  SpanCharacteristic *updateNow;
  bool triggerReset = false;  // flag to reset switch safely

  UpdateTrigger() : Service::Switch() {
    updateNow = new Characteristic::On(false);
  }

  boolean update() override {
    if (updateNow->getNewVal()) {
      Serial.println("User confirmed update!");
      httpUpdate();         // run OTA update
      triggerReset = true;  // mark for reset
    }
    return true;
  }

  void loop() override {
    if (triggerReset) {
      updateNow->setVal(false);  // reset switch safely
      triggerReset = false;
    }
  }
};

// ---------- Motion Sensor to show update availability ----------
struct UpdateAvailableSensor : Service::MotionSensor {
  SpanCharacteristic *motionChar;

  UpdateAvailableSensor() : Service::MotionSensor() {
    motionChar = new Characteristic::MotionDetected(false); // false = no update
  }

  void loop() override {
    static unsigned long lastCheck = 3600000;
    if (millis() - lastCheck > 10000) {  // check every hour
      lastCheck = millis();
      if (checkUpdateAvailable()) {
        motionChar->setVal(true);  // update available
        Serial.println("Update available!");
      } else {
        motionChar->setVal(false);
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(10);

  homeSpan.setSketchVersion(FW_VERSION);
  homeSpan.setStatusPin(2);
  homeSpan.enableWebLog(1);
  //homeSpan.enableOTA("19980719");
  homeSpan.begin(Category::Sensors,"Projector Current");

  ACS.autoMidPoint();        // auto-calibrate zero offset
  ACS.suppressNoise(true);   // filter noise
  Serial.println("ACS auto-calibrate zero offset. ACS suppress noise = true");


  new SpanAccessory();
  new Service::AccessoryInformation();
    new Characteristic::Name("Projector Current");
    new Characteristic::Manufacturer("Tekovate");
    new Characteristic::Model("ESP32-ACS712");
    new Characteristic::SerialNumber("SN-001");
    new Characteristic::FirmwareRevision(FW_VERSION);
    new Characteristic::Identify();

  new Service::TemperatureSensor();
  currentChar = new Characteristic::CurrentTemperature(0.0);

  new UpdateTrigger();
  new UpdateAvailableSensor();

  xTaskCreatePinnedToCore(factoryResetTask, "FactoryReset", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(acsSensorTask, "acsSensorTask", 8192, NULL, 1, NULL, 0);
}


void loop() {
  homeSpan.poll();
}
