//#define USE_SOFTWARE_WIRE   // comment this line to use hardware Wire

#ifdef USE_SOFTWARE_WIRE
  #include <SoftwareWire.h>
  #define SDA_PIN 6
  #define SCL_PIN 5
  SoftwareWire myWire(SDA_PIN, SCL_PIN);
  #define ACTIVE_WIRE myWire
#else
  #include <Wire.h>
  #define SDA_PIN 10
  #define SCL_PIN 11
  #define ACTIVE_WIRE Wire
#endif

void doScan() {
  Serial.println("I2C scanner. Scanning ...");
  byte count = 0;

#ifdef USE_SOFTWARE_WIRE
  ACTIVE_WIRE.begin();
#else
  ACTIVE_WIRE.begin(); 
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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void loop() {
  doScan();
  delay(1000);
}
