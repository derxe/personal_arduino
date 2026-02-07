
//#define USE_SOFTWARE_WIRE        // comment/uncomment as needed
//#define USE_SOFTWARE_WIRE_ESP32   // use ESP32_SoftWire
// (if both are commented out, hardware Wire is used)

/*
const int PIN_VIN      = 17;   // power to AHT20 (optional, can use 3V3 instead)
//const int PIN_GND      = 40;   // "ground" via GPIO (optional, can use real GND)
const int i2c_SDA_PIN  = 34;   // your custom SDA pin
const int i2c_SCL_PIN  = 21;   // your custom SCL pin
*/

// ############
// DEAFULT pins are SDA=33 SCL=35

//const int PIN_VIN      = 40;   // power to AHT20 (optional, can use 3V3 instead)
const int i2c_SDA_PIN  = 11;   // your custom SDA pin
const int i2c_SCL_PIN  = 12;   // your custom SCL pin


#include <Arduino.h>
#include "driver/gpio.h"  // for gpio_set_drive_capability

#include <Wire.h>


// common pin config
#define SDA_PIN i2c_SDA_PIN
#define SCL_PIN i2c_SCL_PIN

#ifdef USE_SOFTWARE_WIRE
  #include <SoftwareWire.h>
  SoftwareWire myWire(SDA_PIN, SCL_PIN);   // SoftwareWire needs pins in ctor
  #define ACTIVE_WIRE myWire

#elif defined(USE_SOFTWARE_WIRE_ESP32)
  #include "ESP32_SoftWire.h"
  SoftWire myWire;                         // ESP32_SoftWire: pins set in begin()
  #define ACTIVE_WIRE myWire

#else
  #include <Wire.h>
  #define ACTIVE_WIRE  Wire
  #define ACTIVE_WIRE2 Wire1
#endif


void doScan() {
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

  //Serial.printf("SDA=%d SCL=%d\n", (int)SDA, (int)SCL);

  for (byte i = 8; i < 120; i++) {
    Serial.print(".");
    Serial1.print(".");
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.println();
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");

      Serial1.println();
      Serial1.print("Found address: ");
      Serial1.print(i, DEC);
      Serial1.print(" (0x");
      Serial1.print(i, HEX);
      Serial1.println(")");
      count++;
      delay(1);
    }
  }

  Serial.println("Done.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");

  Serial1.println("Done.");
  Serial1.print("Found ");
  Serial1.print(count, DEC);
  Serial1.println(" device(s).");
}


void doScan2() {
  Serial.println("I2C scanner 2. Scanning ...");
  byte count = 0;

  //Serial.printf("SDA=%d SCL=%d\n", (int)SDA, (int)SCL);

  for (byte i = 8; i < 120; i++) {
    Serial.print(".");
    Serial1.print(".");
    Wire1.beginTransmission(i);
    if (Wire1.endTransmission() == 0) {
      Serial.println();
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");

      Serial1.println();
      Serial1.print("Found address: ");
      Serial1.print(i, DEC);
      Serial1.print(" (0x");
      Serial1.print(i, HEX);
      Serial1.println(")");
      count++;
      delay(1);
    }
  }

  Serial.println("Done.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");

  Serial1.println("Done.");
  Serial1.print("Found ");
  Serial1.print(count, DEC);
  Serial1.println(" device(s).");
}

//#define AS600_POWER_PIN 4

void setup() {
  //pinMode(AS600_POWER_PIN, OUTPUT);
  //digitalWrite(AS600_POWER_PIN, HIGH);
  //gpio_set_drive_capability((gpio_num_t) AS600_POWER_PIN, GPIO_DRIVE_CAP_3);

 // pinMode(PIN_VIN, OUTPUT);  digitalWrite(PIN_VIN, HIGH);
  //pinMode(PIN_GND, OUTPUT);  digitalWrite(PIN_GND, LOW);
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8N1, 7, 9);

  Wire.begin(1, 2, 400000);
  pinMode(3, OUTPUT);  digitalWrite(3, HIGH);

  Wire1.begin(11, 8, 400000);
  pinMode(9, OUTPUT);  digitalWrite(9, HIGH);

  //Wire1.begin(37, 39, 400000);
  //Wire1.begin(7, 8, 400000);
  //pinMode(9, OUTPUT);  digitalWrite(9, HIGH);
}

void loop() {
  doScan();
  doScan2();
  delay(1000);
  static int on = 1;
  pinMode(15, OUTPUT);  digitalWrite(15, on? HIGH : LOW);
  on = !on;
}

