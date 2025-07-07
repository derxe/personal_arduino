#include "Button2.h"
#include "esp_timer.h"
#include "AS5600.h"
#include <Wire.h>

#include <TinyPICO.h>

TinyPICO tp = TinyPICO();

#define Serial_print(x)    do { SerialDBG.print(x); /* Serial1.print(x);*/ } while (0)
#define Serial_println(x)  do { SerialDBG.println(x); /* Serial1.println(x);*/ } while (0)
#define Serial_write(x)    do { SerialDBG.write(x); /*Serial1.write(x);*/ } while (0)

#define AT_RX_PIN 36
#define AT_TX_PIN 35
#define SerialAT Serial1

#define DBG_RX_PIN 32
#define DBG_TX_PIN 33
#define SerialDBG Serial2


#define GPRS_ON_PIN      15 // mosfet pin
#define GPRS_POWER_PIN   25 // power pin on the sim board

void setup() {
  //Serial.begin(115200);
  //
  SerialDBG.begin(115200, SERIAL_8N1, DBG_TX_PIN, DBG_RX_PIN);
  SerialAT.begin(9600, SERIAL_8N1, AT_TX_PIN, AT_RX_PIN); // 33, 34);

  pinMode(GPRS_ON_PIN, INPUT); // off

  Serial_println("Waiting for RUN...");
  Serial_println("Done init");
  delay(400);

  setCpuFrequencyMhz(80);


}

void turnOffModule() {
  pinMode(GPRS_ON_PIN, INPUT);
  //digitalWrite(GPRS_ON_PIN, HIGH);
  Serial_println("gprs high -> turning off");
}

void loop() {
  // put your main code here, to run repeatedly:

  updateSerial();
  delay(1);
  //esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start(); // sleep 

  static unsigned long lastBlinkTime = 0;
  static int blinkCount = 0;
  #define BLINK_DURATION 2
  #define BLINK_INTERVAL 300
  unsigned long now = millis();
  if (now - lastBlinkTime >= BLINK_DURATION) {
    lastBlinkTime = now;

    switch(blinkCount) {
      case 0:
        tp.DotStar_SetPower(true);
        tp.DotStar_SetPixelColor(255, 0, 0);
        break;

      case 1:
        tp.DotStar_SetPower(false);
        break;
    }
    blinkCount = (blinkCount + 1) % (BLINK_INTERVAL / BLINK_DURATION);
  }
}

void turnOnModule() {
  pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW);
  Serial_println("turning on");
  delay(1000);
  Serial_println("Turning on GPRS module!");
  //digitalWrite(GPRS_POWER_PIN, LOW);
  Serial_println("waiting 1s ...");
  delay(1000);
  //digitalWrite(GPRS_POWER_PIN, HIGH);
  Serial_println("Done");
}

bool updateSerial() {
  static String inputBuffer = "";

  while (SerialDBG.available()) {
    char c = SerialDBG.read();
    //Serial_print("read:'");
    Serial_print(c);
    //Serial_println("'");
    inputBuffer += c;

    if (c == '\n' || c == '\r') {
      Serial_println();
      Serial_print("Command:'");
      Serial_print(inputBuffer[0]);
      Serial_println("'");
      SerialDBG.flush();

      if (inputBuffer[0] == 'o' || inputBuffer[0] == 'O') {
        turnOnModule();
      } else if (inputBuffer[0] == 'x' || inputBuffer[0] == 'X') {
        turnOffModule();
      } else if (inputBuffer[0] == 'h' || inputBuffer[0] == 'H') {
        return false;

        unsigned long start = millis();
        //runHttpGetHot();
        Serial_println("Finished hot hot hot");
        Serial_print("Duration:"); Serial_print((millis()-start)/1000); Serial_println("s");
      } else {
        SerialAT.write(inputBuffer.c_str());       //Forward what Serial received to Software Serial Port
      }
      
      inputBuffer = "";
    }
  }

  while (SerialAT.available()) {
    SerialDBG.write(SerialAT.read());  //Forward what Software Serial received to Serial Port
  }

  return true;
}
