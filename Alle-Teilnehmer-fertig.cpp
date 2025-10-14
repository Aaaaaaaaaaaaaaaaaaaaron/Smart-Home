/*
// platformini.io
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
monitor_speed = 115200

lib_deps =
    adafruit/RTClib
    adafruit/Adafruit BME280 Library
    adafruit/Adafruit Unified Sensor
    adafruit/Adafruit GFX Library
    adafruit/Adafruit ILI9341
    knolleary/PubSubClient
*/
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_BME280.h>
#include "RTClib.h"

// ==== WLAN & MQTT ====
const char* ssid = "DEIN_WIFI_NAME";
const char* password = "DEIN_WIFI_PASSWORT";
const char* mqtt_server = "DEINE_MQTT_BROKER_IP";  // z. B. "192.168.178.20"

// ==== Objekte ====
WiFiClient espClient;
PubSubClient client(espClient);
RTC_DS3231 rtc;
Adafruit_BME280 bme;
File logFile;

// ==== Display (ILI9341) ====
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ==== SD-Karte ====
#define SD_CS 5

// ==== Timer ====
unsigned long lastSend = 0;
const unsigned long interval = 60000;  // 1 Minute

// ==== WLAN verbinden ====
void setup_wifi() {
  Serial.print(" WLAN verbinden: ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WLAN verbunden!");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());
}

// ==== MQTT reconnect ====
void reconnect() {
  while (!client.connected()) {
    Serial.print(" MQTT verbinden...");
    if (client.connect("ESP32SensorClient")) {
      Serial.println("verbunden!");
    } else {
      Serial.print(" Fehler, rc=");
      Serial.println(client.state());
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // ==== RTC ====
  if (!rtc.begin()) {
    Serial.println("RTC nicht gefunden!");
    while (1);
  }

  // ==== BME280 ====
  if (!bme.begin(0x76)) {
    Serial.println(" Kein BME280 erkannt!");
    while (1);
  }

  // ==== SD-Karte ====
  if (!SD.begin(SD_CS)) {
    Serial.println(" SD-Karte konnte nicht initialisiert werden!");
    while (1);
  }

  // ==== Display ====
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);

  // Kopfzeile CSV (nur einmal)
  if (!SD.exists("/data.csv")) {
    logFile = SD.open("/data.csv", FILE_WRITE);
    logFile.println("Datum,Uhrzeit,Temperatur,Luftfeuchte,Druck");
    logFile.close();
  }

  Serial.println(" Setup abgeschlossen!");
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long nowMillis = millis();
  if (nowMillis - lastSend >= interval) {
    lastSend = nowMillis;

    DateTime now = rtc.now();
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    // ==== 1️ Ausgabe im Serial Monitor ====
    Serial.printf("[%02d:%02d:%02d] T=%.2f°C  H=%.2f%%  P=%.2fhPa\n",
                  now.hour(), now.minute(), now.second(), temp, hum, pres);

    // ==== 2️ MQTT senden ====
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f}", temp, hum, pres);
    client.publish("smarthome/sensor/bme280", payload);

    // ==== 3️ Auf SD-Karte schreiben ====
    logFile = SD.open("/data.csv", FILE_APPEND);
    if (logFile) {
      logFile.printf("%02d.%02d.%04d,%02d:%02d:%02d,%.2f,%.2f,%.2f\n",
                     now.day(), now.month(), now.year(),
                     now.hour(), now.minute(), now.second(),
                     temp, hum, pres);
      logFile.close();
    }

    // ==== 4 Anzeige auf Display ====
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(10, 30);
    tft.printf("Zeit: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
    tft.setCursor(10, 60);
    tft.printf("Temp: %.2f C", temp);
    tft.setCursor(10, 90);
    tft.printf("Hum:  %.2f %%", hum);
    tft.setCursor(10, 120);
    tft.printf("Druck: %.2f hPa", pres);
  }
}
