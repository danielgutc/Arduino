#include <SoftwareSerial.h>

#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include <MeAuriga.h>
#include <ArduinoJson.h>

MeBluetooth bluetooth(PORT_16);

void setup() {
  Serial.begin(115200);
  bluetooth.begin(115200);
  Serial.println("✅ Bluetooth JSON println mode with randomized data");
  randomSeed(analogRead(0));  // Seed randomness
}

void loop() {
  // Simulated sensor data
  float temperature = random(200, 300) / 10.0;  // 20.0 to 30.0 °C
  int distance = random(50, 150);              // 50 to 150 cm
  float battery = random(370, 420) / 100.0;     // 3.70 to 4.20 V
  unsigned long timestamp = millis() / 1000;

  // Create JSON
  StaticJsonDocument<128> doc;
  doc["ts"] = timestamp;
  doc["temp"] = temperature;
  doc["dist"] = distance;
  doc["bat"] = battery;

  // Serialize and send
  String json;
  serializeJson(doc, json);
  bluetooth.println(json);

  delay(1000);
}
