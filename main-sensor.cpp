// ...existing code...
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// ...existing code...

// ====================
// Sensor
// ====================
Adafruit_BME280 bme; // I2C
unsigned long lastRead = 0;
const unsigned long interval = 60000; // 60 Sekunden

// ====================
// WLAN & MQTT
// ====================
const char* ssid = "iPhoneJacob";
const char* password = "jackyjack";

const char* mqtt_server = "172.20.10.3"; // alte ip: 192.168.80.1
const int mqtt_port = 1883;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// ====================
// BLE Provisioning
// ====================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_SSID_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_PASS_UUID      "ab34c9e5-8c3b-4f6b-9d49-1b2c3d4e5f60"

String prov_ssid = "";
String prov_pass = "";
bool prov_received = false;
BLECharacteristic* ssidChar;
BLECharacteristic* passChar;

class SSIDCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    prov_ssid = String(val.c_str());
    Serial.print("BLE SSID empfangen: ");
    Serial.println(prov_ssid);
    if (prov_ssid.length() && prov_pass.length()) prov_received = true;
  }
};

class PASSCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    prov_pass = String(val.c_str());
    Serial.print("BLE PASS empfangen: ");
    Serial.println(prov_pass);
    if (prov_ssid.length() && prov_pass.length()) prov_received = true;
  }
};

// ====================
// WLAN Verbindung (jetzt mit BLE Provisioning fallback)
// ====================
void startBleProvisioning() {
  BLEDevice::init("ESP32-Provisioning-BLE-Aaron"); // Aaron Hinzugefügt
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  ssidChar = pService->createCharacteristic(CHAR_SSID_UUID, BLECharacteristic::PROPERTY_WRITE);
  passChar = pService->createCharacteristic(CHAR_PASS_UUID, BLECharacteristic::PROPERTY_WRITE);

  ssidChar->setCallbacks(new SSIDCallback());
  passChar->setCallbacks(new PASSCallback());

  // Notify clients that they can write (optional)
  ssidChar->addDescriptor(new BLE2902());
  passChar->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE-Provisioning gestartet - verbinde mit 'ESP32-Provisioning-BLE' und schreibe SSID & PASS");
}

void setup_wifi() {
  Serial.println();
  Serial.print("Versuche, mit gespeicherter SSID zu verbinden: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  // 15 Sekunden normal versuchen
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWLAN verbunden");
    Serial.print("IP-Adresse: ");
    Serial.println(WiFi.localIP());
    return;
  }

  // Normalverbindung fehlgeschlagen -> BLE Provisioning starten
  Serial.println("\nWLAN-Verbindung fehlgeschlagen! Starte BLE-Provisioning...");
  startBleProvisioning();

  // Warte auf Daten über BLE (Timeout 5 Minuten)
  unsigned long provStart = millis();
  while (!prov_received && millis() - provStart < 5UL * 60UL * 1000UL) {
    delay(100);
    // prov_received wird von BLE-Callbacks gesetzt
  }

  BLEDevice::stopAdvertising();
  BLEDevice::deinit(true);

  if (prov_received) {
    Serial.println("Provisioning-Daten erhalten. Versuche Verbindung mit neuem Netzwerk...");
    Serial.print("SSID: "); Serial.println(prov_ssid);
    // verbindungsversuch mit neuen credentials
    WiFi.begin(prov_ssid.c_str(), prov_pass.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWLAN verbunden (provisioned)");
      Serial.print("IP-Adresse: ");
      Serial.println(WiFi.localIP());
      // optional: speichere neue Credentials dauerhaft (SPIFFS / Preferences) - nicht implementiert hier
      return;
    } else {
      Serial.println("\nVerbindung mit neuen Daten fehlgeschlagen. BLE-Provisioning beenden.");
    }
  } else {
    Serial.println("Kein Provisioning erhalten (Timeout).");
  }
}
// ...existing code...

// ====================
// MQTT Auto-Reconnect
// ====================
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Versuche MQTT-Verbindung...");
    if (client.connect("ESP32Client")) {
      Serial.println("Verbunden!");
    } else {
      Serial.print("Fehlercode: ");
      Serial.print(client.state());
      Serial.println(" -> erneuter Versuch in 5 Sekunden");
      delay(5000);
    }
  }
}

// ====================
// Setup
// ====================
void setup() {
  Serial.begin(115200);
  delay(2000); // Sensor hochfahren lassen

  // Sensor starten
  if (!bme.begin(0x76)) {
    Serial.println("Sensor nicht gefunden!");
    while (1);
  }

  // WLAN (mit BLE-Provisioning fallback)
  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  reconnect_mqtt(); // MQTT Verbindung aufbauen
}

// ====================
// Loop
// ====================
void loop() {
  if (!client.connected()) {
    reconnect_mqtt(); // Auto-Reconnect
  }
  client.loop(); // Verbindung aufrecht halten

  if (millis() - lastRead >= interval) {
    lastRead = millis();

    float temp = bme.readTemperature();
    float hum  = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    // Terminalausgabe
    Serial.printf("Temp: %.2f C  Hum: %.2f %%  Druck: %.2f hPa\n", temp, hum, pres);

    // MQTT-Nachricht
    char payload[100];
    snprintf(payload, sizeof(payload), "Temp: %.2f C, Hum: %.2f %%, Druck: %.2f hPa", temp, hum, pres);
    client.publish("sensor/test", payload);
  }
}
