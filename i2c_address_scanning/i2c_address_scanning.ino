
//#define USE_SOFTWARE_WIRE        // comment/uncomment as needed
//#define USE_SOFTWARE_WIRE_ESP32   // use ESP32_SoftWire
// (if both are commented out, hardware Wire is used)

/*
const int PIN_VIN      = 17;   // power to AHT20 (optional, can use 3V3 instead)
//const int PIN_GND      = 40;   // "ground" via GPIO (optional, can use real GND)
const int i2c_SDA_PIN  = 34;   // your custom SDA pin
const int i2c_SCL_PIN  = 21;   // your custom SCL pin
*/

const int PIN_VIN      = 40;   // power to AHT20 (optional, can use 3V3 instead)
const int i2c_SDA_PIN  = 36;   // your custom SDA pin
const int i2c_SCL_PIN  = 38;   // your custom SCL pin


#include <Arduino.h>
#include "driver/gpio.h"  // for gpio_set_drive_capability

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
  #define ACTIVE_WIRE Wire
#endif


void doScan() {
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

#if defined(USE_SOFTWARE_WIRE)
  // SoftwareWire: pins already set in constructor
  ACTIVE_WIRE.begin();

#elif defined(USE_SOFTWARE_WIRE_ESP32)
  // ESP32_SoftWire: pins & speed already set in setup(), this just (re)enables
  ACTIVE_WIRE.begin(SDA_PIN, SCL_PIN, 400000);

#else
  // hardware Wire on ESP32: set pins here
  ACTIVE_WIRE.begin(SDA_PIN, SCL_PIN);
#endif

  for (byte i = 8; i < 120; i++) {
    Serial.print(".");
    ACTIVE_WIRE.beginTransmission(i);
    if (ACTIVE_WIRE.endTransmission() == 0) {
      Serial.println();
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);
    }
  }

  Serial.println("Done.");
  Serial.print("Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
}

//#define AS600_POWER_PIN 4

void setup() {
  //pinMode(AS600_POWER_PIN, OUTPUT);
  //digitalWrite(AS600_POWER_PIN, HIGH);
  //gpio_set_drive_capability((gpio_num_t) AS600_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(PIN_VIN, OUTPUT);  digitalWrite(PIN_VIN, HIGH);
  //pinMode(PIN_GND, OUTPUT);  digitalWrite(PIN_GND, LOW);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
}

void loop() {
  doScan();
  delay(1000);
}

