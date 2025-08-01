#include <Wire.h>

#define SDA_PIN 10
#define SCL_PIN 11

void doScan() {
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  // 0x27 0x3F
  //Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Serial.print(".");
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) {
      Serial.println();
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);
    }
  }
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}

void setup() {

  Serial.begin (115200);
  while (!Serial) delay(10);


}  // end of setup

void loop() {
  doScan();
  delay(1000);
}
