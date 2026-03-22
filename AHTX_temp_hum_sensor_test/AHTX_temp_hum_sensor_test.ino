#define TX_PIN  39
#define RX_PIN  37 
#define Serial Serial1

#include "AHT20_SoftI2C.h"

#define AHT20_1_PWR_PIN   10
#define AHT20_1_SDA_PIN   8
#define AHT20_1_SCL_PIN   9

#define AHT20_2_PWR_PIN   12
#define AHT20_2_SDA_PIN   11
#define AHT20_2_SCL_PIN   13

AHT20SoftI2C aht1(AHT20_1_SDA_PIN, AHT20_1_SCL_PIN, AHT20_1_PWR_PIN, 1);
AHT20SoftI2C aht2(AHT20_2_SDA_PIN, AHT20_2_SCL_PIN, AHT20_2_PWR_PIN, 2);

void setup() {
  Serial.begin(115200, SERIAL_8N1, TX_PIN, TX_PIN);
  while (!Serial) delay(1);

  // Power sensor from GPIOs (optional)
  pinMode(15, OUTPUT);

  Serial.println("Init AHT20 (soft I2C)...");
  aht1.init();
  aht2.init();
}

// toggle power on and off 
void loop2() {
  aht1.setPower(true);
  digitalWrite(15, HIGH);
  Serial.println("power ON");
  delay(3000);

  aht1.setPower(false);
  digitalWrite(15, LOW);
  Serial.println("power OFF");
  delay(3000);
}

bool readTempHum(AHT20SoftI2C &ahtSensor, float &t, float &h) {
  ahtSensor.setPower(true);

  if (!ahtSensor.checkIsConnected()) {
    Serial.printf("Temp sensor %d: is not connected!\r\n", ahtSensor.id);
    return false;
  }

  delay(10);
  int readStatus = 0;
  int retries = 10;
  for(int i=0; i<retries; i++) {
    readStatus = ahtSensor.aht20_read(t, h);
    //Serial.printf("Read status: %d\r\n", readStatus);
    if(readStatus == 1) break; 
    delay(30);
  }

  ahtSensor.setPower(false);
  
  return readStatus == 1;
}

void loop() {
  float t = NAN;
  float h = NAN;

  if(readTempHum(aht1, t, h)) {
    Serial.printf("temp1: T = %s, H = %s %%\r\n", String(t, 2), String(h, 2));
  } 

  if(readTempHum(aht2, t, h)) {
    Serial.printf("temp2: T = %s, H = %s %%\r\n", String(t, 2), String(h, 2));
  } 

  Serial.println();

  delay(300);
}












