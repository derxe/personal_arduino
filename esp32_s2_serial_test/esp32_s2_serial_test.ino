/*
  ASCII table

  Prints out byte values in all possible formats:
  - as raw binary values
  - as ASCII-encoded decimal, hex, octal, and binary values

  For more on ASCII, see http://www.asciitable.com and http://en.wikipedia.org/wiki/ASCII

  The circuit: No external hardware needed.

  created 2006
  by Nicholas Zambetti <http://www.zambetti.com>
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/ASCIITable
*/

#include <BMP280_DEV.h> 

BMP280_DEV bmp280;    

#define LED_PIN 15

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("ESP32 S2 start");

  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X16);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_16);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startForcedConversion();

  Serial.println("bmp280 init finish");

  pinMode(LED_PIN, OUTPUT);

  // prints title with ending line break
  
}

float readAltitude() {
  bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if in SLEEP_MODE)

  float temperature, pressure, altitude;
  // Wait until the measurement is complete
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {
    
    }  

  return altitude;
}


long lastPing = millis();
long lastLedOn = millis();
long lastLedOff = millis();

void loop() {
  long now = millis();
  if(now - lastPing > 20) {
    bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if in SLEEP_MODE)
    float temperature, pressure, altitude;
    // Wait until the measurement is complete
    while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  

    Serial.print(pressure);
    Serial.println(";");

    if(now - lastLedOff > 200) {
      lastLedOff = now;
      lastLedOn = now;
      digitalWrite(LED_PIN, HIGH);
    }

    lastPing = now;
  }

  if(digitalRead(LED_PIN) && now - lastLedOn > 50) {
      digitalWrite(LED_PIN, LOW);
  }
}
