/*
//platformio.ini:

[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
monitor_speed = 115200

lib_deps =
    adafruit/RTClib
    adafruit/Adafruit GFX Library
    adafruit/Adafruit ILI9341
*/

#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ==== Display-Pins ====
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starte RTC + Display...");

  // ==== I2C starten (für DS3231) ====
  Wire.begin(21, 22);

  // ==== RTC initialisieren ====
  if (!rtc.begin()) {
    Serial.println("DS3231 nicht gefunden!");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC hat Zeit verloren — stelle aktuelle Zeit!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // ==== Display initialisieren ====
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(3);

  Serial.println("System bereit!");
}

void loop() {
  DateTime now = rtc.now();

  // ==== Uhrzeit-String erstellen ====
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  // ==== Bildschirm aktualisieren ====
  tft.fillRect(60, 100, 200, 50, ILI9341_BLACK); // alten Text löschen
  tft.setCursor(60, 100);
  tft.print(timeStr);

  delay(1000);
}
