#define TX_PIN  39
#define RX_PIN  37 
#define Serial Serial1

#include "temp_sensor.h"

#define TEMP_1_PWR_PIN   10
#define TEMP_1_SDA_PIN   8
#define TEMP_1_SCL_PIN   9

#define TEMP_2_PWR_PIN   12
#define TEMP_2_SDA_PIN   11
#define TEMP_2_SCL_PIN   13

AHT20 temp1(TEMP_1_SDA_PIN, TEMP_1_SCL_PIN, TEMP_1_PWR_PIN, 1);
SH40 temp2(TEMP_2_SDA_PIN, TEMP_2_SCL_PIN, TEMP_2_PWR_PIN, 2);

void setup() {
  Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  while (!Serial) delay(1);

  // Power sensor from GPIOs (optional)
  pinMode(15, OUTPUT);

  Serial.println("Init temp sensors (soft I2C)...");
  temp1.init();
  temp2.init();
}

// toggle power on and off 
void loop2() {
  temp1.setPower(true);
  digitalWrite(15, HIGH);
  Serial.println("power ON");
  delay(3000);

  temp2.setPower(false);
  digitalWrite(15, LOW);
  Serial.println("power OFF");
  delay(3000);
}

bool readTempHum(TempSensor &sensor, float &t, float &h) {
  sensor.setPower(true);

  if (!sensor.checkIsConnected()) {
    Serial.printf("Temp sensor %d: is not connected!\r\n", sensor.id);
    sensor.setPower(false);
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

  //sensor.setPower(false);
  
  return readStatus == 1;
}

void loop() {
  float t = NAN;
  float h = NAN;

  if(readTempHum(temp1, t, h)) {
    Serial.printf("temp1: T = %s, H = %s %% | status=%d (%s)\r\n",
      String(t, 2), String(h, 2), temp1.getLastReadStatus(), temp1.getLastReadStatusDescription());
  } else {
    Serial.printf("temp1: read failed | status=%d (%s)\r\n",
      temp1.getLastReadStatus(), temp1.getLastReadStatusDescription());
  }

  if(readTempHum(temp2, t, h)) {
    Serial.printf("temp2: T = %s, H = %s %% | status=%d (%s)\r\n",
      String(t, 2), String(h, 2), temp2.getLastReadStatus(), temp2.getLastReadStatusDescription());
  } else {
    Serial.printf("temp2: read failed | status=%d (%s)\r\n",
      temp2.getLastReadStatus(), temp2.getLastReadStatusDescription());
  }

  Serial.println();

  delay(300);
}








