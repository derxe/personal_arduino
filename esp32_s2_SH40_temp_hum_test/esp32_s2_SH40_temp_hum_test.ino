#define TX_PIN  39
#define RX_PIN  37 
#define Serial Serial1

#include "SH40_SoftI2C.h"

#define SH40_1_PWR_PIN   10
#define SH40_1_SDA_PIN   8
#define SH40_1_SCL_PIN   9

#define SH40_2_PWR_PIN   12
#define SH40_2_SDA_PIN   11
#define SH40_2_SCL_PIN   13

SH40SoftI2C sht1(SH40_1_SDA_PIN, SH40_1_SCL_PIN, SH40_1_PWR_PIN, 1);
SH40SoftI2C sht2(SH40_2_SDA_PIN, SH40_2_SCL_PIN, SH40_2_PWR_PIN, 2);

void setup() {
  Serial.begin(115200, SERIAL_8N1, TX_PIN, TX_PIN);
  while (!Serial) delay(1);

  // Power sensor from GPIOs (optional)
  pinMode(15, OUTPUT);

  Serial.println("Init SH40 (soft I2C)...");
  sht1.init();
  sht2.init();
}

// toggle power on and off 
void loop2() {
  sht1.setPower(true);
  digitalWrite(15, HIGH);
  Serial.println("power ON");
  delay(3000);

  sht1.setPower(false);
  digitalWrite(15, LOW);
  Serial.println("power OFF");
  delay(3000);
}

bool readTempHum(SH40SoftI2C &sensor, float &t, float &h) {
  sensor.setPower(true);

  if (!sensor.checkIsConnected()) {
    Serial.printf("Temp sensor %d: is not connected!\r\n", sensor.id);
    return false;
  }

  delay(10);
  int readStatus = 0;
  int retries = 10;
  for(int i=0; i<retries; i++) {
    readStatus = sensor.read(t, h);
    //Serial.printf("Read status: %d\r\n", readStatus);
    if(readStatus == 1) break; 
    delay(30);
  }

  sensor.setPower(false);
  
  return readStatus == 1;
}

void loop() {
  float t = NAN;
  float h = NAN;

  if(readTempHum(sht1, t, h)) {
    Serial.printf("temp1: T = %s, H = %s %% | status=%d (%s)\r\n",
      String(t, 2), String(h, 2), sht1.getLastReadStatus(), sht1.getLastReadStatusDescription());
  } else {
    Serial.printf("temp1: read failed | status=%d (%s)\r\n",
      sht1.getLastReadStatus(), sht1.getLastReadStatusDescription());
  }

  if(readTempHum(sht2, t, h)) {
    Serial.printf("temp2: T = %s, H = %s %% | status=%d (%s)\r\n",
      String(t, 2), String(h, 2), sht2.getLastReadStatus(), sht2.getLastReadStatusDescription());
  } else {
    Serial.printf("temp2: read failed | status=%d (%s)\r\n",
      sht2.getLastReadStatus(), sht2.getLastReadStatusDescription());
  }

  Serial.println();

  delay(300);
}










