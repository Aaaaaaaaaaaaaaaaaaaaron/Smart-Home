#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <Preferences.h>
#include <esp_sleep.h> // <-- neu: für Deep Sleep

/*
  Smart-home-sensor Firmware - kurze Übersicht (Kommentare im Code):

  - Startet entweder als Cold Boot (Provisions-AP wird geöffnet) oder
    nach Wakeup aus Deep Sleep (versucht kurz, die gespeicherten WLAN/MQTT
    Einstellungen zu nutzen und eine Verbindung aufzubauen).
  - Provisioning: WebServer auf SoftAP stellt ein Formular bereit, um
    SSID/Passwort/MQTT-Server zu setzen. Werte werden in NVS (Preferences)
    gespeichert.
  - Sensor: BME280 wird ausgelesen. Werte werden in CSV auf der SD-Karte
    gespeichert (falls SD verfügbar) und per MQTT an Topic `sensor/test`
    gesendet.
  - MQTT: Verbindung wird kurz aufgebaut, Publish durchgeführt, anschließend
    wird kurz gewartet (Settle-Zeit), dann wird sauber getrennt und das
    Gerät geht in Deep Sleep.
  - Deep Sleep: sorgt für niedrigen Verbrauch; Gerät wacht nach Timer auf
    und startet Zyklus neu.

  Die folgenden Kommentare in Funktionen erklären die Details an Ort und Stelle.
*/

// === Konfiguration ===
const char* AP_SSID = "esp32-Aaron";
// AP password will be read from NVS into `ap_pass`; do not hardcode it here

// SD-Karte Pins
#define SD_CS   5
#define SD_SCK  18
#define SD_MOSI 23
#define SD_MISO 19

// LED Pin (für Success-Indikator nach MQTT Publish)
#define LED_PIN 16

Adafruit_BME280 bme;
bool bme_ok = false; // sensor init status
bool credsLoaded = false; // NVS creds cached flag
WebServer server(80);
WiFiClient wifiClient;
PubSubClient client(wifiClient);
Preferences prefs;

unsigned long lastRead = 0;
const unsigned long interval = 60000; // 60s
// Wait up to this many ms after a successful MQTT publish before entering deep sleep
const unsigned long MQTT_PUBLISH_SETTLE_MS = 1000; // 1s (ensure publish reaches broker)
// Fixed deep sleep duration in milliseconds
const unsigned long DEEP_SLEEP_MS = 30000; // 30s
bool provisioningActive = false;
unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 10000; // Alle 10s WLAN prüfen

// SD-Karte
File dataFile;
const char* csv_filename = "/sensor_data.csv";
bool sd_ok = false; // Status ob SD erfolgreich gemountet

// Runtime credentials (loaded from NVS or empty)
String rt_ssid = "";
String rt_pass = "";
String rt_mqtt = "";
String ap_pass = "";
int    rt_mqtt_port = 1883;

// Forward (ersetze alte Deklaration)
bool reconnect_mqtt();
void startProvisioningAP();
void stopProvisioningAP();
bool load_credentials_from_nvs();
bool store_credentials_to_nvs(const String &ssid, const String &pass, const String &mqtt, int port);
// neue Vorwärtsdeklarationen
void attemptConnect(const String &ssid, const String &pass, const String &mqtt, int port);
void handleStatus();
void handleConfirm();
void blink_led(int duration_ms); // <-- neue Forward-Deklaration

// --- Hilfsfunktion: SD-Root auflisten (Debug) ---
void listSdRoot() {
  Serial.println("[DBG] listSdRoot() entering");
  if (!sd_ok) {
    Serial.println("[DBG] SD not mounted");
    return;
  }
  File root = SD.open("/");
  if (!root) {
    Serial.println("[ERR] Could not open SD root");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("[ERR] Root is not a directory");
    root.close();
    return;
  }
  Serial.println("SD Root:");
  File file = root.openNextFile();
  while (file) {
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print("  \t");
    Serial.println(file.size());
    file.close();
    file = root.openNextFile();
  }
  root.close();
  Serial.println("[DBG] listSdRoot() done");
}

// --- SD-Karte Initialisierung (robuster, mehrere Frequenzen / Fallback) ---
//
// Diese Funktion versucht, die SD-Karte auf den standard-VSPI-Pins zu
// initialisieren. Bei Erfolg wird `sd_ok` auf true gesetzt und gegebenenfalls
// eine CSV-Datei mit Header angelegt. Die Initialisierung versucht mehrere
// SPI-Frequenzen (Fallback) weil manche SD-Module mit hohen Frequenzen
// Probleme machen.
//
// Wichtige Aspekte:
// - SD kann ausfallen (Hardware / Verkabelung / Formatting) -> dann werden
//   Schreibversuche übersprungen und es wird trotzdem normal weitergemacht.
// - SD-Zugriffe sollten möglichst kurz gehalten werden (wenig I/O während
//   des Awake-Intervalls).

void init_sd_card() {
  Serial.println("[DBG] init_sd_card() starting");
  Serial.printf("[DBG] SPI pins: SCK=%d MISO=%d MOSI=%d CS=%d\n", SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  SPI.end();
  SPI.begin(); // default VSPI pins (SCK=18,MOSI=23,MISO=19)
  delay(100);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(50);

  const uint32_t freqs[] = {4000000, 1000000, 400000, 250000};
  sd_ok = false;
  for (size_t i = 0; i < sizeof(freqs)/sizeof(freqs[0]); ++i) {
    Serial.printf("[DBG] Versuch SD.begin mit %u Hz...\n", (unsigned)freqs[i]);
    if (SD.begin(SD_CS, SPI, freqs[i])) {
      sd_ok = true;
      Serial.printf("[INFO] SD.begin erfolgreich mit %u Hz\n", (unsigned)freqs[i]);
      break;
    } else {
      Serial.printf("[WARN] SD.begin mit %u Hz fehlgeschlagen\n", (unsigned)freqs[i]);
      delay(50);
    }
  }
  if (!sd_ok) {
    Serial.println("[DBG] Fallback: SD.begin(CS) versuchen...");
    if (SD.begin(SD_CS)) {
      sd_ok = true;
      Serial.println("[INFO] SD.begin(CS) erfolgreich (Fallback)");
    } else {
      Serial.println("[ERR] SD.begin(CS) ebenfalls fehlgeschlagen");
    }
  }
  if (!sd_ok) {
    Serial.println("[ERR] SD-Karte konnte nicht initialisiert werden!");
    Serial.println("[HINT] Prüfe: Pins korrekt verdrahtet? SD-Karte FAT32 formatiert? VCC=3.3V?");
    Serial.printf("[DBG] CS=%d, SCK=%d, MOSI=%d, MISO=%d\n", SD_CS, SD_SCK, SD_MOSI, SD_MISO);
    return;
  }

  Serial.println("[INFO] SD-Karte erfolgreich initialisiert");
  if (!SD.exists(csv_filename)) {
    dataFile = SD.open(csv_filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Zeitstempel,Temperatur (C),Luftfeuchtigkeit (%),Druck (hPa)");
      dataFile.close();
      Serial.println("[INFO] CSV-Header geschrieben");
    } else {
      Serial.println("[ERR] Konnte CSV-Datei nicht erstellen");
    }
  } else {
    Serial.println("[DBG] CSV-Datei existiert bereits");
  }
}

// --- Daten in CSV schreiben (prüft sd_ok) ---
//
// Diese Funktion schreibt einen Zeitstempel und die gemessenen Sensorwerte
// im CSV-Format auf die SD-Karte. Falls die SD-Karte nicht gemountet ist
// (`sd_ok == false`) wird die Funktion sofort verlassen. Dadurch wird
// vermieden, dass Probleme mit der SD-Karte den Messzyklus oder den
// Deep-Sleep Ablauf blockieren.

void write_sensor_to_csv(float temp, float hum, float pres) {
  Serial.println("[DBG] write_sensor_to_csv() called");
  if (!sd_ok) {
    Serial.println("[WARN] CSV nicht geschrieben: SD nicht gemountet");
    return;
  }
  if (!SD.exists(csv_filename)) {
    Serial.println("[DBG] CSV-Datei nicht vorhanden - erstelle neu");
    dataFile = SD.open(csv_filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Zeitstempel,Temperatur (C),Luftfeuchtigkeit (%),Druck (hPa)");
      dataFile.close();
      Serial.println("[INFO] CSV-Header neu geschrieben");
    } else {
      Serial.println("[ERR] Fehler beim (Wieder)Erstellen der CSV-Datei");
      return;
    }
  }
  dataFile = SD.open(csv_filename, FILE_APPEND);
  if (dataFile) {
    unsigned long timestamp = millis() / 1000;
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(temp, 2);
    dataFile.print(",");
    dataFile.print(hum, 2);
    dataFile.print(",");
    dataFile.println(pres, 2);
    dataFile.close();
    Serial.printf("[INFO] Daten in CSV gespeichert: ts=%lu T=%.2f H=%.2f P=%.2f\n",
                  timestamp, temp, hum, pres);
  } else {
    Serial.println("[ERR] Fehler beim Öffnen der CSV-Datei zum Schreiben");
  }
}

// --- NVS (Preferences) simple storage ---
//
// Hier werden die WLAN/MQTT-Konfigurationswerte persistiert (NVS via
// Preferences). `store_credentials_to_nvs` schreibt die Werte und
// `load_credentials_from_nvs` liest sie ein. Wir cachen das Laden mit
// `credsLoaded`, damit wiederholte Zugriffe zur Laufzeit nicht mehrfach
// die NVS API aufrufen (spart Zeit beim Aufwachen).

bool store_credentials_to_nvs(const String &ssid, const String &pass, const String &mqtt, int port) {
  Serial.println("[DBG] store_credentials_to_nvs() storing values to NVS");
  prefs.begin("creds", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("mqtt", mqtt);
  prefs.putInt("port", port);
  prefs.end();
  Serial.printf("[INFO] Stored SSID='%s' MQTT='%s' PORT=%d\n", ssid.c_str(), mqtt.c_str(), port);
  return true;
}

bool load_credentials_from_nvs() {
  Serial.println("[DBG] load_credentials_from_nvs() attempting to read NVS");
  prefs.begin("creds", true);
  if (!prefs.isKey("ssid")) { prefs.end(); Serial.println("[DBG] No credentials in NVS"); return false; }
  rt_ssid = prefs.getString("ssid", "");
  rt_pass = prefs.getString("pass", "");
  rt_mqtt = prefs.getString("mqtt", "");
  if (prefs.isKey("ap_pass")) ap_pass = prefs.getString("ap_pass", ""); else ap_pass = "";
  rt_mqtt_port = prefs.getInt("port", 1883);
  prefs.end();
  credsLoaded = true;
  Serial.printf("[INFO] Loaded from NVS: SSID='%s' MQTT='%s' PORT=%d\n", rt_ssid.c_str(), rt_mqtt.c_str(), rt_mqtt_port);
  return true;
}

// --- WebServer (Provisioning) ---
void handleRoot() {
  Serial.println("[DBG] HTTP GET / requested");
  // Lade nur beim ersten Aufruf die gespeicherten Werte (prefill im Formular)
  // Die HTML-Seite ist schlicht - das Formular sendet SSID/Passwort/MQTT.
  // Wichtig: Wir zeigen aus Sicherheitsgründen das gespeicherte AP-Passwort
  // nur wenn es in NVS existiert; sonst wird "offenes Netzwerk" angezeigt.
  if (!credsLoaded) load_credentials_from_nvs(); // ok if returns false
  String page =
    "<!doctype html><html><head><meta charset='utf-8'><title>ESP32 Provisioning</title></head><body>"
    "<h3>ESP32 WLAN + MQTT Provisioning</h3>"
    "<form action=\"/save\" method=\"post\">"
    "SSID: <input type=\"text\" name=\"ssid\" value=\"";
  page += rt_ssid;
  page += "\"><br><br>";
  page += "Passwort: <input type=\"password\" name=\"pass\" value=\"";
  page += rt_pass;
  page += "\"><br><br>";
  page += "MQTT Broker (IP/Hostname): <input type=\"text\" name=\"mqtt\" value=\"";
  page += rt_mqtt;
  page += "\"><br><br>";
  page += "MQTT Port: <input type=\"number\" name=\"mqttport\" value=\"";
  page += String(rt_mqtt_port);
  page += "\"><br><br>";
  page += "<input type=\"submit\" value=\"Speichern & Verbinden\">";
  page += "</form>";
  page += "<p>Verbinde dein Handy mit dem WLAN <b>esp32-Aaron</b> ";
  if (ap_pass.length() > 0) {
    page += "(Passwort: ";
    page += ap_pass;
    page += ") ";
  } else {
    page += "(offenes Netzwerk) ";
  }
  page += "und oeffne http://192.168.4.1</p>";
  page += "</body></html>";
  server.send(200, "text/html", page);
  Serial.println("[DBG] Served provisioning page");
}

// Ersetze handleSave mit diesem (speichert, startet Connect-Versuch, AP bleibt offen)
void handleSave() {
  Serial.println("[DBG] HTTP POST /save received");
  String newSsid = server.arg("ssid");
  String newPass = server.arg("pass");
  String newMqtt = server.arg("mqtt");
  String newMqttPort = server.arg("mqttport");

  Serial.printf("[DBG] Received -> SSID: '%s'  PASS: '%s'  MQTT: '%s'  PORT: '%s'\n",
                newSsid.c_str(), newPass.c_str(), newMqtt.c_str(), newMqttPort.c_str());

  int port = (newMqttPort.length()) ? newMqttPort.toInt() : 1883;
  // store in NVS
  if (store_credentials_to_nvs(newSsid, newPass, newMqtt, port)) {
    Serial.println("[INFO] Credentials saved to NVS");
  } else {
    Serial.println("[ERR] Failed to save credentials to NVS");
  }

  // Startet Verbindungsversuch, aber schließt AP nicht automatisch
  attemptConnect(newSsid, newPass, newMqtt, port);

  // Antwortseite mit Link zur Statusseite und Hinweis, dass AP offen bleibt
  String page = "<html><body><h3>Gespeichert.</h3>"
                "<p>Es wurde versucht zu verbinden. Der Access Point bleibt geoeffnet, bis du die Verbindung bestaetigst.</p>"
                "<p><a href=\"/status\">Verbindungsstatus anzeigen</a></p>"
                "<p>Wenn alles passt, klicke auf <b>Bestaetigen & AP schliessen</b> auf der Statusseite.</p>"
                "</body></html>";
  server.send(200, "text/html", page);
}

// Neue Funktion: versucht Verbindung (kurzer, blockierender Versuch), schließt AP NICHT
//
// Diese Routine wird beim Provisionieren verwendet: sie übernimmt die
// eingegebenen SSID/Passwort/MQTT-Daten, versucht innerhalb eines kurzen
// Timeouts (hier 5s) eine WLAN-Verbindung aufzubauen und, falls erfolgreich,
// eine MQTT-Verbindung aufzubauen. Der AP bleibt offen — der Benutzer muss
// auf der Statusseite die Verbindung bestätigen, damit der AP geschlossen
// wird. Vorteil: Nutzer sieht sofort, ob die Verbindung funktioniert.

void attemptConnect(const String &ssid, const String &pass, const String &mqtt, int port) {
  unsigned long t0 = millis();
  Serial.printf("[DBG] attemptConnect(): SSID='%s' MQTT='%s' PORT=%d\n", ssid.c_str(), mqtt.c_str(), port);
  Serial.println("[DBG] WiFi.begin()");
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  // shorter timeout to avoid long boot times
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
    delay(150);
    Serial.print("[DBG].");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[INFO] WLAN verbunden (provisioned)");
    Serial.printf("[INFO] IP-Adresse: %s\n", WiFi.localIP().toString().c_str());
    // set runtime values but DO NOT close AP here
    rt_ssid = ssid; rt_pass = pass; rt_mqtt = mqtt; rt_mqtt_port = port;
    client.setServer(rt_mqtt.c_str(), rt_mqtt_port);
    bool mqtt_ok = reconnect_mqtt();
    Serial.printf("[DBG] attemptConnect duration=%lums mqtt_ok=%d\n", millis()-t0, mqtt_ok ? 1 : 0);
  } else {
    Serial.printf("[WARN] WLAN Verbindung fehlgeschlagen (attemptConnect, %lums). AP bleibt aktiv.", millis()-t0);
    Serial.println();
  }
}

// Statusseite zeigt aktuellen Verbindungsstatus und bietet Bestätigungsbutton
void handleStatus() {
  Serial.println("[DBG] HTTP GET /status requested");
  String page = "<html><body><h3>Verbindungsstatus</h3>";
  page += "<p>WiFi Status: ";
  page += String(WiFi.status());
  page += " (";
  if (WiFi.status() == WL_CONNECTED) page += "verbunden";
  else page += "nicht verbunden";
  page += ")</p>";
  if (WiFi.status() == WL_CONNECTED) {
    page += "<p>IP: " + WiFi.localIP().toString() + "</p>";
  }
  page += "<p>MQTT Server: " + rt_mqtt + " Port: " + String(rt_mqtt_port) + "</p>";
  page += "<form action=\"/confirm\" method=\"post\">";
  page += "<input type=\"submit\" value=\"Bestaetigen & AP schliessen\">";
  page += "</form>";
  page += "<p><a href=\"/\">Zurueck</a></p></body></html>";
  server.send(200, "text/html", page);
  Serial.println("[DBG] Served status page");
}

// Confirm-Handler: schließt AP nur wenn WiFi verbunden ist (sonst Fehlerseite)
void handleConfirm() {
  Serial.println("[DBG] HTTP POST /confirm received");
  if (WiFi.status() == WL_CONNECTED) {
    stopProvisioningAP();
    String page = "<html><body><h3>Provisioning bestaetigt.</h3><p>AP wurde gestoppt. Geraet laeuft im konfigurierten Netzwerk.</p></body></html>";
    server.send(200, "text/html", page);
    Serial.println("[INFO] Provisioning confirmed, AP closed by user.");
  } else {
    String page = "<html><body><h3>Provisioning konnte nicht bestaetigt werden.</h3><p>ESP ist nicht mit dem angegebenen WLAN verbunden.</p><p><a href=\"/status\">Zurueck zum Status</a></p></body></html>";
    server.send(200, "text/html", page);
    Serial.println("[WARN] User attempted to confirm but WiFi is not connected.");
  }
}

// --- Provisioning AP control ---
void startProvisioningAP() {
  Serial.println("[DBG] startProvisioningAP() called");
  WiFi.mode(WIFI_AP_STA);
  bool ok;
  if (ap_pass.length() > 0) {
    ok = WiFi.softAP(AP_SSID, ap_pass.c_str());
    Serial.printf("[INFO] AP start requested with WPA2 password, result=%d\n", ok);
  } else {
    ok = WiFi.softAP(AP_SSID);
    Serial.printf("[INFO] AP start requested as OPEN network, result=%d\n", ok);
  }
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[INFO] AP IP: %s\n", apIP.toString().c_str());
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/confirm", HTTP_POST, handleConfirm);
  server.onNotFound([](){
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  provisioningActive = true;
  Serial.println("[DBG] Provisioning webserver started");

}

void stopProvisioningAP() {
  Serial.println("[DBG] stopProvisioningAP() called");
  server.stop();
  WiFi.softAPdisconnect(true);
  provisioningActive = false;
  Serial.println("[DBG] AP stopped");
}

// --- Try to connect and stop AP on success ---
void attemptConnectAndMaybeStopAP(const String &ssid, const String &pass, const String &mqtt, int port) {
  Serial.printf("[DBG] attemptConnectAndMaybeStopAP(): SSID='%s' MQTT='%s' PORT=%d\n", ssid.c_str(), mqtt.c_str(), port);
  Serial.println("[DBG] WiFi.begin()");
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print("[DBG].");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[INFO] WLAN verbunden (provisioned)");
    Serial.printf("[INFO] IP-Adresse: %s\n", WiFi.localIP().toString().c_str());
    rt_ssid = ssid; rt_pass = pass; rt_mqtt = mqtt; rt_mqtt_port = port;
    client.setServer(rt_mqtt.c_str(), rt_mqtt_port);
    if (reconnect_mqtt()) {
      stopProvisioningAP();
    } else {
      Serial.println("[WARN] MQTT connect failed during provisioning; keeping AP active");
    }
  } else {
    Serial.println("[WARN] WLAN Verbindung fehlgeschlagen. Provisioning bleibt aktiv.");
    // keep AP running (server already running)
  }
}

// === WLAN Auto-Reconnect (wenn disconnected) ===
//
// Diese Funktion wird regelmäßig aus loop() aufgerufen und versucht, die
// gespeicherten Credentials zu verwenden, um das WLAN wiederherzustellen.
// Falls keine Credentials vorhanden sind, startet sie den Provisioning-AP.
// Timeouts sind bewusst kurz gewählt (einige Sekunden), damit das Aufwachen
// nach Deep Sleep nicht zu lange dauert.

void check_and_reconnect_wifi() {
  if (millis() - lastWiFiCheck < wifiCheckInterval) return;
  lastWiFiCheck = millis();

  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WARN] WLAN-Verbindung verloren! Versuche Reconnect...");
  // try to load saved creds and reconnect
  if (!load_credentials_from_nvs()) {
    Serial.println("[DBG] Keine gespeicherten Credentials, Provisioning aktiv.");
    if (!provisioningActive) startProvisioningAP();
    return;
  }
  Serial.printf("[DBG] Trying saved SSID='%s'\n", rt_ssid.c_str());
  WiFi.begin(rt_ssid.c_str(), rt_pass.c_str());
  unsigned long start = millis();
  // shorter reconnect timeout
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
    delay(200);
    Serial.print("[DBG].");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[INFO] WLAN wiederverbunden!");
    Serial.print("[INFO] IP-Adresse: ");
    Serial.println(WiFi.localIP());
    client.setServer(rt_mqtt.c_str(), rt_mqtt_port);
    unsigned long t0 = millis();
    if (!reconnect_mqtt()) {
      Serial.println("[WARN] MQTT connect failed after reconnect attempt");
    }
    Serial.printf("[DBG] reconnect attempt duration=%lums\n", millis()-t0);
    return;
  }
  Serial.println("[WARN] Reconnect fehlgeschlagen. Starte AP zur Provisionierung...");
  if (!provisioningActive) startProvisioningAP();
}

// === MQTT Helfer ===
//
// MQTT-Verbindungen werden kurz aufgebaut. Ursprünglich gab es einen
// expliziten TCP-Check (check_mqtt_reachable), welcher jedoch blockierend
// war und beim Aufwachen die Awake-Zeit stark verlängert hat. Stattdessen
// versuchen wir nun direkt, den MQTT-Client zu verbinden und geben nach
// kurzer Zeit auf, falls die Verbindung nicht zustande kommt.
//
// Hinweis: Die Funktion `reconnect_mqtt()` versucht `client.connect()` und
// wartet nur sehr kurz (kleiner Stabilisation-Timeout), um die Awake-Zeit
// möglichst klein zu halten.
bool check_mqtt_reachable(const char* host, uint16_t port) {
  // Diese Funktion ist jetzt ein no-op / Debug-Hook, um vorhandenen Code
  // kompatibel zu halten. Der eigentliche Verbindungsversuch erfolgt am
  // MQTT-Client selbst (schneller und weniger blockierend).
  Serial.printf("[DBG] check_mqtt_reachable(%s,%u) - SKIPPED (fast-path)\n", host, port);
  return true;
}

bool reconnect_mqtt() {
  // Versucht, den MQTT-Client zu verbinden. Wir bauen die Verbindung auf
  // und warten danach nur sehr kurz (kurzer Stabilisationszeitraum), damit
  // der Aufwachzeitraum klein bleibt. Falls connect fehlschlägt, geben
  // wir false zurück; auf diese Weise wird der Zyklus nicht lange blockiert
  // und es folgt ein ordentlicher Deep Sleep.
  Serial.println("[DBG] reconnect_mqtt() called");
  if (client.connected()) { Serial.println("[DBG] MQTT already connected"); return true; }
  if (rt_mqtt.length() == 0) { Serial.println("[DBG] No MQTT server configured"); return false; }
  Serial.print("Versuche MQTT-Verbindung... ");
  Serial.print("Broker: ");
  Serial.print(rt_mqtt);
  Serial.print(":");
  Serial.println(rt_mqtt_port);

  // quick attempt to connect (no blocking reachability check)
  String clientId = "ESP32Client-";
  clientId += WiFi.macAddress();

  Serial.printf("[DBG] MQTT clientId=%s\n", clientId.c_str());
  if (!client.connect(clientId.c_str())) {
    int st = client.state();
    Serial.printf("[ERR] MQTT connect failed, state=%d\n", st);
    // single short blink to indicate MQTT server not reachable
    blink_led(150);
    return false;
  }

  // wait a little for the client to become active, but keep it short
  unsigned long t0 = millis();
  while (!client.connected() && millis() - t0 < 800) {
    client.loop();
    delay(20);
  }
  if (client.connected()) {
    Serial.println("[INFO] MQTT verbunden!");
    return true;
  } else {
    Serial.println("[ERR] MQTT connection did not stabilize after connect");
    // single short blink to indicate MQTT server didn't stabilise
    blink_led(150);
    return false;
  }
}

// --- LED Blink Funktion ---
void blink_led(int duration_ms) {
  Serial.printf("[LED] Blinken für %d ms\n", duration_ms);
  digitalWrite(LED_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(LED_PIN, LOW);
}

// === Setup / Loop ===
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[BOOT] Starting Smart-home-sensor firmware");

  // LED initialisieren (GPIO 16, Output)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Low = LED aus
  Serial.printf("[LED] LED auf GPIO %d initialisiert\n", LED_PIN);

  Wire.begin(21, 22); // SDA, SCL
  bme_ok = bme.begin(0x76);
  if (!bme_ok) {
    Serial.println("[WARN] BME280 nicht gefunden auf 0x76!");
  } else {
    Serial.println("[INFO] BME280 initialisiert");
  }

  // init SD (optional)
  init_sd_card();

  // Load existing prefs for pre-fill (but still start provisioning)
  if (load_credentials_from_nvs()) {
    Serial.println("[INFO] Prefill credentials loaded from NVS");
  } else {
    Serial.println("[INFO] No saved credentials in NVS");
  }

  // Decide whether to start the provisioning AP.
  // If we woke from a Deep Sleep timer wakeup, skip starting the AP
  // (only start AP on cold boot / normal boot so confirmation makes sense).
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.printf("[BOOT] Wakeup reason: %d\n", (int)wakeup_reason);
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("[BOOT] Woke from Deep Sleep - not starting provisioning AP");
    // Try a quick reconnect: WLAN + MQTT so device can operate after wake
    if (!credsLoaded) load_credentials_from_nvs();
    if (rt_ssid.length() > 0) {
      unsigned long t0_total = millis();
      Serial.println("[BOOT] Attempting quick WiFi reconnect after wakeup");
      WiFi.mode(WIFI_STA);
      WiFi.begin(rt_ssid.c_str(), rt_pass.c_str());
      unsigned long t0 = millis();
      // shorter quick reconnect window
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 3000) {
        delay(150);
        Serial.print(".");
      }
      Serial.println();
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[BOOT] WiFi reconnected after wakeup (%lums)\n", millis()-t0);
        client.setServer(rt_mqtt.c_str(), rt_mqtt_port);
        unsigned long t_mqtt = millis();
        if (!reconnect_mqtt()) {
          Serial.println("[WARN] MQTT connect after wakeup failed");
        }
        Serial.printf("[BOOT] total wake reconnect duration=%lums\n", millis()-t0_total);
      } else {
        Serial.println("[WARN] WiFi reconnect after wakeup failed");
      }
    }
  } else {
    // start provisioning AP on cold boot so user can change creds after reset
    startProvisioningAP();
  }

  client.setServer(rt_mqtt.c_str(), rt_mqtt_port);
  Serial.println("[BOOT] setup() done");
}

void loop() {
  // periodic wifi check/auto-reconnect
  check_and_reconnect_wifi();

  // LED blinken solange keine WLAN-Verbindung besteht (auch während Provisioning)
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Wenn Provisioning aktiv, Webserver bedienen und nicht in Sleep gehen
  if (provisioningActive) {
    server.handleClient();
    // kurz warten, damit HTTP-Anfragen bedient werden können
    delay(10);
    return;
  }

  // MQTT handling
  if (!client.connected()) {
    reconnect_mqtt();
  } else {
    client.loop();
  }

  if (millis() - lastRead >= interval) {
    unsigned long cycleStart = millis();
    lastRead = millis();
    Serial.printf("[CYCLE] Start sensor cycle at %lu\n", cycleStart);

    float temp = NAN, hum = NAN, pres = NAN;
    if (bme_ok) {
      temp = bme.readTemperature();
      hum  = bme.readHumidity();
      pres = bme.readPressure() / 100.0F;
    } else {
      Serial.println("[WARN] BME280 not initialized, skipping read");
    }

    Serial.printf("[SENSOR] Temp: %.2f C  Hum: %.2f %%  Druck: %.2f hPa\n", temp, hum, pres);

    // Schreibe in CSV-Datei auf SD-Karte
    if (!isnan(temp) && !isnan(hum) && !isnan(pres)) {
      write_sensor_to_csv(temp, hum, pres);
    } else {
      Serial.println("[WARN] Sensorwerte ungültig, nicht gespeichert/publiziert");
    }

    char payload[128];
    snprintf(payload, sizeof(payload), "Temp: %.2f C, Hum: %.2f %%, Druck: %.2f hPa", temp, hum, pres);
    bool publish_ok = false;
    if (client.connected()) {
      publish_ok = client.publish("sensor/test", payload);
      Serial.printf("[MQTT] Publish %s -> %d\n", payload, publish_ok ? 1 : 0);
    } else {
      Serial.println("[MQTT] Not connected, skipping publish");
    }

    // LED zweimal blinken wenn erfolgreich publiziert
    //
    // Kurze visuelle Bestätigung: zwei kurze Blinks signalisieren einen
    // erfolgreichen Publish. Da wir unmittelbar danach die Verbindung
    // trennen und in Deep Sleep gehen, sind die Blinks kurz gehalten.
    if (publish_ok) {
      // two short blinks to indicate successful publish
      blink_led(150);
      delay(150);
      blink_led(150);
    }

    // Give MQTT a short time to settle/send data so deep sleep doesn't interrupt
    //
    // Wir rufen `client.loop()` mehrfach auf und warten eine kurze Zeit,
    // damit der Publish-Puffer tatsächlich über das WLAN zum Broker kommt.
    // Die Wartezeit ist bewusst kurz (Einstellung via MQTT_PUBLISH_SETTLE_MS),
    // um den Awake-Zeitraum minimal zu halten.
    if (publish_ok && client.connected()) {
      Serial.printf("[MQTT] Waiting up to %lu ms for publish to settle...\n", MQTT_PUBLISH_SETTLE_MS);
      unsigned long t0 = millis();
      while (millis() - t0 < MQTT_PUBLISH_SETTLE_MS) {
        client.loop();
        if (!client.connected()) { Serial.println("[MQTT] Client disconnected while waiting"); break; }
        delay(50);
      }
      Serial.println("[MQTT] Proceeding to disconnect and sleep");
    }

    // --- Energiesparmodus: Deep Sleep zwischen Messungen ---
    // Nur schlafen, wenn Provisioning nicht aktiv (sonst bleibt AP verfügbar)
    Serial.println("[SLEEP] Vorbereitung zum Deep Sleep...");

    // Sauber trennen
    //
    // Wir trennen MQTT und WLAN bewusst sauber, damit beim nächsten
    // Aufwachen ein schneller Reconnect möglich ist und keine hängenden
    // Sockets/Verbindungen übrig bleiben.
    if (client.connected()) {
      client.disconnect();
      Serial.println("[SLEEP] MQTT disconnected");
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[SLEEP] WiFi abgeschaltet");

    // kleine Pause damit alle Serial-Ausgaben übertragen sind
    Serial.printf("[SLEEP] Gehe jetzt für %lu ms in den Deep Sleep\n", interval);
    Serial.flush();

    // Deep Sleep: use fixed deep sleep duration (DEEP_SLEEP_MS)
    unsigned long awake_ms = millis() - cycleStart;
    unsigned long sleep_ms = DEEP_SLEEP_MS;
    Serial.printf("[CYCLE] Cycle finished in %lums, sleeping for %lums (fixed)\n", awake_ms, sleep_ms);
    uint64_t sleep_us = (uint64_t)sleep_ms * 1000ULL; // ms -> µs
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();

    // esp_deep_sleep_start() endet nie - nach Wakeup wird setup() neu ausgeführt
  }

  // short idle debug
  static unsigned long dbgTick = 0;
  if (millis() - dbgTick > 30000) {
    dbgTick = millis();
    Serial.printf("[STATUS] WiFi=%d MQTT=%d Provisioning=%d FreeHeap=%d\n",
                  WiFi.status(), client.connected(), provisioningActive, ESP.getFreeHeap());
  }
}