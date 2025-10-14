/*
//platformio.ini:

[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
monitor_speed = 115200

lib_deps =
    adafruit/RTClib @ ^2.1.3
    adafruit/Adafruit BusIO @ ^1.16.1
*/
#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;  // RTC-Objekt erstellen

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(" Starte DS3231-Test...");

  // I2C starten (SDA=21, SCL=22)
  Wire.begin(21, 22);

  if (!rtc.begin()) {
    Serial.println(" Konnte DS3231 nicht finden!");
    while (1);
  }

  // Wenn RTC nicht läuft, Zeit neu setzen
  if (rtc.lostPower()) {
    Serial.println(" RTC hat Strom verloren, setze neue Zeit...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  Serial.println("DS3231 erfolgreich verbunden!");
}

void loop() {
  DateTime now = rtc.now();

  // Uhrzeit ausgeben
  Serial.print("Uhrzeit: ");
  if (now.hour() < 10) Serial.print('0');
  Serial.print(now.hour());
  Serial.print(":");
  if (now.minute() < 10) Serial.print('0');
  Serial.print(now.minute());
  Serial.print(":");
  if (now.second() < 10) Serial.print('0');
  Serial.print(now.second());

  // Datum ausgeben
  Serial.print(" | Datum: ");
  Serial.print(now.day());
  Serial.print(".");
  Serial.print(now.month());
  Serial.print(".");
  Serial.println(now.year());

  delay(1000);
}


