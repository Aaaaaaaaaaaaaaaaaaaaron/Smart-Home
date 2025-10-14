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

/*
3.3V → VCC
GND → GND
GPIO21 → SDA
GPIO22 → SCL
*/

void setup() {
  Serial.println("Hello World");
  Serial.begin(115200);
  if (!bme.begin(0x76)) { // Adresse anpassen falls nötig
    Serial.println("Sensor nicht gefunden!");
    while (1);
  }
}

void loop() {
  Serial.print("Temperatur: "); Serial.print(bme.readTemperature()); Serial.println(" °C");
  Serial.print("Luftdruck: "); Serial.print(bme.readPressure()/100.0F); Serial.println(" hPa");
  Serial.print("Luftfeuchte: "); Serial.print(bme.readHumidity()); Serial.println(" %");
  Serial.println("----");
  delay(2000);
}
