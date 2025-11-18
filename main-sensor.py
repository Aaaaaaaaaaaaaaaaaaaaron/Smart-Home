#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// === Konfiguration ===
const char* stored_ssid    = "WLAN-RDF-EU"; //"iPhoneJacob";   // Default / gespeicherte SSID
const char* stored_pass    = "wlan@RdF"; //"jackyjack";     // Default / gespeichertes PW
String mqtt_server        = "172.20.15.179"; //"172.20.10.3";    // Broker-IP (änderbar via Web‑Form)
int    mqtt_port          = 1883;             // Broker-Port (änderbar)

Adafruit_BME280 bme;
WebServer server(80);
WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long lastRead = 0;
const unsigned long interval = 60000; // 60s
bool provisioningActive = false;

// Vorwärtsdeklaration, damit handleSave() reconnect_mqtt() aufrufen kann
void reconnect_mqtt();

// --- WebServer (Provisioning) ---
void handleRoot() {
  String page =
    "<!doctype html><html><head><meta charset='utf-8'><title>ESP32 Provisioning</title></head><body>"
    "<h3>ESP32 WLAN + MQTT Provisioning</h3>"
    "<form action=\"/save\" method=\"post\">"
    "SSID: <input type=\"text\" name=\"ssid\" value=\"";
  page += stored_ssid;
  page += "\"><br><br>";
  page += "Passwort: <input type=\"password\" name=\"pass\" value=\"";
  page += stored_pass;
  page += "\"><br><br>";
  page += "MQTT Broker (IP/Hostname): <input type=\"text\" name=\"mqtt\" value=\"";
  page += mqtt_server;
  page += "\"><br><br>";
  page += "MQTT Port: <input type=\"number\" name=\"mqttport\" value=\"";
  page += String(mqtt_port);
  page += "\"><br><br>";
  page += "<input type=\"submit\" value=\"Speichern & Verbinden\">"
    "</form>"
    "<p>Verbinde dein Handy mit dem WLAN <b>esp32-Aaron</b> und öffne http://192.168.4.1</p>"
    "</body></html>";
  server.send(200, "text/html", page);
}

void handleSave() {
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");
  String newMqtt = server.arg("mqtt");
  String newMqttPort = server.arg("mqttport");

  Serial.printf("Provisioning erhalten -> SSID: '%s'  PASS: '%s'  MQTT: '%s'  PORT: '%s'\n",
                newSsid.c_str(), newPass.c_str(), newMqtt.c_str(), newMqttPort.c_str());

  // Update stored values in RAM
  if (newSsid.length()) {
    // copy to stored_ssid (const char*) - keep original defaults for next boot unless you implement persistent storage
    // If you want persistence, use Preferences to write newSsid/newPass/newMqtt/newMqttPort
    // For now, update the globals used during this run:
    // Note: stored_ssid/stored_pass are const char* defaults; use them for display only.
    // We'll use local variables for the active credentials:
  }

  // Set active MQTT server/port if provided
  if (newMqtt.length()) mqtt_server = newMqtt;
  if (newMqttPort.length()) mqtt_port = newMqttPort.toInt();

  server.send(200, "text/html", "<html><body><h3>Versuche zu verbinden...</h3><p>Bitte dieses Fenster offen lassen.</p></body></html>");

  // stop Webserver/AP and try to connect with new WiFi credentials
  provisioningActive = false;
  server.stop();
  WiFi.softAPdisconnect(true);
  delay(200);

  Serial.println("Versuche Verbindung mit neuen Credentials...");
  if (newSsid.length()) {
    WiFi.begin(newSsid.c_str(), newPass.c_str());
  } else {
    // fallback to stored defaults if no SSID provided
    WiFi.begin(stored_ssid, stored_pass);
  }

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWLAN verbunden (provisioned)");
    Serial.print("IP-Adresse: ");
    Serial.println(WiFi.localIP());

    // update MQTT client with potentially new server/port and try to connect
    client.setServer(mqtt_server.c_str(), mqtt_port);
    reconnect_mqtt();
  } else {
    Serial.println("\nVerbindung fehlgeschlagen. Starte AP erneut.");
    WiFi.softAP("esp32-Aaron"); // AP-Name wie gewünscht
    server.begin();
    provisioningActive = true;
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

// === WLAN / Provisioning ===
void setup_wifi() {
  Serial.println();
  Serial.print("Versuche, mit gespeicherter SSID zu verbinden: ");
  Serial.println(stored_ssid);

  WiFi.begin(stored_ssid, stored_pass);

  unsigned long startAttemptTime = millis();
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

  // Verbindung fehlgeschlagen -> starte SoftAP + Webserver zur Provisionierung
  Serial.println("\nWLAN-Verbindung fehlgeschlagen! Starte Access Point zur Provisionierung...");
  WiFi.softAP("esp32-Aaron"); // offenes AP mit deinem Namen
  Serial.print("AP gestartet: esp32-Aaron  | AP IP-Adresse: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound([](){
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  provisioningActive = true;
  Serial.println("Provisioning Webserver gestartet -> öffne http://192.168.4.1");
}

// === MQTT Helfer ===
bool check_mqtt_reachable(const char* host, uint16_t port) {
  WiFiClient testClient;
  Serial.printf("Teste TCP zu %s:%u ... ", host, port);
  bool ok = testClient.connect(host, port);
  if (ok) {
    Serial.println("erreichbar");
    testClient.stop();
    return true;
  } else {
    Serial.println("nicht erreichbar");
    return false;
  }
}

void reconnect_mqtt() {
  if (client.connected()) return;

  Serial.print("Versuche MQTT-Verbindung... ");
  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
  Serial.print("  Broker: ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);

  if (!check_mqtt_reachable(mqtt_server.c_str(), mqtt_port)) {
    Serial.println("MQTT-Broker nicht erreichbar von diesem WLAN. Warte 10s und versuche später.");
    delay(10000);
    return;
  }

  String clientId = "ESP32Client-";
  clientId += WiFi.macAddress();

  if (client.connect(clientId.c_str())) {
    Serial.println("MQTT verbunden!");
  } else {
    int st = client.state();
    Serial.print("Fehlercode: ");
    Serial.print(st);
    Serial.println(" -> erneuter Versuch in 5 Sekunden");
    delay(5000);
  }
}

// === Setup / Loop ===
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(21, 22); // SDA, SCL
  if (!bme.begin(0x76)) {
    Serial.println("BME280 nicht gefunden auf 0x76!");
    // falls kein Sensor: trotzdem weiter (nur keine Messwerte)
  }

  setup_wifi();

  client.setServer(mqtt_server.c_str(), mqtt_port);
  reconnect_mqtt();

  // sofort erste Messung senden
  lastRead = millis() - interval;
}

void loop() {
  if (provisioningActive) {
    server.handleClient();
  }

  if (!client.connected()) {
    reconnect_mqtt();
  } else {
    client.loop();
  }

  if (millis() - lastRead >= interval) {
    lastRead = millis();

    float temp = NAN, hum = NAN, pres = NAN;
    if (bme.begin(0x76)) { // sicherstellen, Sensor verfügbar
      temp = bme.readTemperature();
      hum  = bme.readHumidity();
      pres = bme.readPressure() / 100.0F;
    }

    Serial.printf("Temp: %.2f C  Hum: %.2f %%  Druck: %.2f hPa\n", temp, hum, pres);

    char payload[128];
    snprintf(payload, sizeof(payload), "Temp: %.2f C, Hum: %.2f %%, Druck: %.2f hPa", temp, hum, pres);
    if (client.connected()) client.publish("sensor/test", payload);
  }
}