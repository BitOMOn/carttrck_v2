#include "BluetoothSerial.h"
#include "ArduinoJson.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup() { //general setup
  Serial.begin(115200);
  SerialBT.begin("Carttrack-v2");
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(2, OUTPUT);
  }

void loop() {

  // json compiler
  DynamicJsonDocument doc(1024);
  doc["error"] = "NULL";
  doc["status"] = "NULL";
  doc["speed"] = random(0, 60);
  doc["rpm"] = random(0, 500);
  doc["power_MAIN"] = random(0, 1200);
  doc["voltage_MAIN"] = random(45, 60);
  doc["current_MAIN"] = random(0, 60);
  doc["vesc_amphour"] = random(0, 100);
  doc["vesc_watthour"] = random(0, 100);
  doc["vesc_tach"] = random(0, 100);
  doc["vesc_distance"] = random(0, 1000);
  doc["vesc_batpercentage"] = random(0, 100);
  doc["battery_temperature"] = random(20, 70);
  char jsonpkt[1024];
  serializeJson(doc, jsonpkt);
  SerialBT.printf(jsonpkt);
  Serial.println(jsonpkt);

  delay(250);
  
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  
  if (SerialBT.available()) {
    String readString;
    while (SerialBT.available()) {
    delay(10);

    char c = SerialBT.read();
    if (c == ',') {
      break;
    }
    readString += c; 
  } //makes the string readString  

  if (readString.length() >0) {
    Serial.println(readString.c_str());
  }
  }
 delay(500);
}
