#include <Wire.h>


#define VANE_POWER_PIN    1
#define AS600_POWER_MOS_PIN 6
#define SDA_PIN 10
#define SCL_PIN 8




void setup() {
  pinMode(VANE_POWER_PIN, OUTPUT); digitalWrite(VANE_POWER_PIN, HIGH);  
  gpio_set_drive_capability((gpio_num_t) VANE_POWER_PIN, GPIO_DRIVE_CAP_3);
  pinMode(AS600_POWER_MOS_PIN, OUTPUT);
  delay(10);

  Serial.begin (115200);
  while (!Serial) delay(10);

  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  // 0x27 0x3F
  Wire.begin(SDA_PIN, SCL_PIN);
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
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
}  // end of setup

void loop() {}
