#include "SoftwareSerial.h"
#include "BluetoothSerial.h"
#include "ArduinoJson.h"
#include "VescUart.h"

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       36 // ESP32 pin GIOP36 (ADC0) connected to LM35

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) //ble and json
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
VescUart vesc;

SoftwareSerial vescSerial(13, 15);

float avgMotorCurrent;
float avgInputCurrent;
float dutyCycleNow;
int rpm;
float inpVoltage;
float ampHours;
float wattHours;
float wattHoursCharged;
float tachometer;
float tachometerAbs;
float tempMosfet;
float tempMotor;

float tempBattery;

float velocity;
float batpercentage;
int power;



void setup() { //general setup
  Serial.begin(115200);
  SerialBT.begin("Carttrack-v2");
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  vescSerial.begin(19200);
  vesc.setSerialPort(&vescSerial);
  }

void loop() {

    int LM35_adcVal = analogRead(PIN_LM35); // measure battery temprature
    float milliVolt = LM35_adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    tempBattery = milliVolt / 10;

    if (tempBattery > 59){ // 60C battery waring light
        digitalWrite(2, HIGH);
      } else {
        digitalWrite(2, LOW);
      }

    if ( vesc.getVescValues() ) {

      avgMotorCurrent = (vesc.data.avgMotorCurrent);
      avgInputCurrent = (vesc.data.avgInputCurrent);
      dutyCycleNow = (vesc.data.dutyCycleNow);
      rpm = (vesc.data.rpm)/7 ; // motor pole pairs
      inpVoltage = (vesc.data.inpVoltage);
      ampHours = (vesc.data.ampHours);
      wattHours = (vesc.data.wattHours);
      wattHoursCharged = (vesc.data.wattHoursCharged);
      tachometer = (vesc.data.tachometer)/42; // the number of motor poles multiplied by 3
      tachometerAbs = (vesc.data.tachometerAbs)/42; // the number of motor poles multiplied by 3
      tempMosfet = (vesc.data.tempMosfet);
      tempMotor = (vesc.data.tempMotor);
      
      velocity = rpm*3.142*(60/1000)*0.72*(16/185); // Motor RPM x Pi x (seconds in a minute / meters in a mile) x Wheel diameter x (motor pulley / wheelpulley)
      batpercentage = ((inpVoltage-38.4)/12)*100; // ((Battery voltage - minimum voltage) / number of cells) x 100
      power = avgInputCurrent*inpVoltage;

  }
  else
  {
    Serial.println("Failed to get vesc data!");
  }

  if (inpVoltage < 38.4) { // low voltage indicator
      digitalWrite(3, HIGH);
    } else {
      digitalWrite(3, LOW);
    }


  // json compiler
  DynamicJsonDocument doc(1024);
  doc["avgMotorCurrent"] = avgMotorCurrent;
  doc["avgInputCurrent"] = avgInputCurrent;
  doc["dutyCycleNow"] = dutyCycleNow;
  doc["rpm"] = rpm;
  doc["inpVoltage"] = inpVoltage;
  doc["ampHours"] = ampHours;
  doc["wattHours"] = wattHours;
  doc["wattHoursCharged"] = wattHoursCharged;
  doc["tachometer"] = tachometer;
  doc["tachometerAbs"] = tachometerAbs;
  doc["tempMosfet"] = tempMosfet;
  doc["tempMotor"] = tempMotor;
  doc["velocity"] = velocity;
  doc["batpercentage"] = batpercentage;
  doc["power"] = power;
  doc["tempBattery"] = tempBattery;
  char jsonpkt[1024];
  serializeJson(doc, jsonpkt);
  SerialBT.printf(jsonpkt);
  Serial.println(jsonpkt);
  
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
