# Smart-Home
Sensor/ Display-Komponente die über MQTT verbunden sind 

Die Sensor-Komponente soll ein Sensoren-Modul über I2C ansteuern und regelmäßig 
(jede Minute) die aktuellen Messdaten an einen MQTT-Broker senden.
Außerdem sollen die Daten auf einer SD-Karte als CSV-Datei gespeichert werden.

Die Display Komponente soll die aktuellen Messdaten über MQTT empfangen
und auf einem Display (SPI) anzeigen. Außerdem soll ein RTC-Modul (I2C) 
eingebunden werden, um die aktuelle Uhrzeit anzuzeigen.

Für beide Komponenten soll ein Prototyp entwickelt werden, der die geforderten Anforde􀀘
rungen erfüllt. Je ein Teampartner ist Hauptverantwortlich für eine Komponente.
• Die oben beschriebene Funktionalität soll implementiert werden.
• Die Art des Sensors und des Displays ist frei wählbar, solange die vorgegebene Schnitt􀀘
stelle (I2c/SPI) eingehalten wird.
• Die Komponenten sollen über WiFi􀀘Provisioning in ein bestehendes WLAN􀀘Netzwerk
eingebunden werden können.
• Verliert die Komponente die Verbindung zum WLAN, soll sie sich automatisch wieder
verbinden können.
• Die Adresse des MQTT􀀘Brokers soll konfigurierbar sein, um verschiedene Broker nutzen
zu können. (Bedeutet während des Betriebs kann der Broker gewechselt werden ohne
den ESP32 neu zu flashen.)
• Passwörter und andere sensible Daten sollen sicher gespeichert werden. (Nicht im
Quellcode!) Stickwort: NVS Storage
• Ein MQTT􀀘Broker (z.B Mosquitto) soll eingerichtet werden, um die Kommunikation zwi􀀘
schen den Komponenten zu ermöglichen. Oder alternativ ein Cloud􀀘Dienst (z.B. Adafruit
IO, AWS IoT, …).
• Ein Gehäuse für die Komponenten soll entworfen und gedruckt werden, um die Hardware
zu schützen und eine ansprechende Optik zu bieten. Die SD􀀘Karte soll über einen Slot
zugänglich sein, um die Daten einfach auslesen zu können.
• Die Komponenten sollen die UART Schnittstelle nutzen, um Debugging􀀘Informationen
auszugeben.
• Die Komponenten sollen so konzipiert sein, dass sie leicht erweitert werden können, um
zukünftige Funktionen hinzuzufügen.
• Beide Komponenten sollen mobil einsetzbar sein, d.h. sie sollen mit einem Akku betrieben
werden können.
• Ein sinnvolles Energiemanagement soll implementiert werden (z.B. Sleep􀀘Modus, um
den Stromverbrauch zu minimieren).
• Die Komponenten sollen eine visuelle Rückmeldung über den Verbindungsstatus zum
WLAN und MQTT􀀘Broker geben.
