/*
// platformio.ini:
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
monitor_speed = 115200
lib_deps = adafruit/Adafruit BME280 Library@^2.3.0
board_build.partitions = huge_app.csv
*/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C

unsigned long lastRead = 0;
const unsigned long interval = 60000; // 60 Sekunden
/*
3.3V → VCC
GND → GND
GPIO21 → SDA
GPIO22 → SCL
*/


void setup() {
  //Serial.println("Hello World");
  Serial.begin(115200);
  delay(2000);  // 2 Sekunden warten, damit der Sensor hochfahren kann
  if (!bme.begin(0x76)) { // Adresse anpassen falls nötig
    Serial.println("Sensor nicht gefunden!");
    while (1);
  }
}


void loop() {
  if (millis() - lastRead >= interval) {
    lastRead = millis();
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;
    Serial.printf("Temp: %.2f C  Hum: %.2f %%  Druck: %.2f hPa\n", temp, hum, pres);
  }
}

