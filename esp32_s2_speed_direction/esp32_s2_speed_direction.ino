//
//    FILE: AS5600_position_speed.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo compare performance with update flag
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.

#include "AS5600.h"
#include <Wire.h>
#include <WiFi.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>


#define BUZZER_PIN 11  // Change this to your actual LED pin
#define ROTATION_SENSOR_PIN 3
#define PHOTORESISTOR_PIN 14

#define LED_PIN 4
#define ANALOG_PIN 1



#define Serial_print(x)    do { Serial.print(x); Serial1.print(x); } while (0)
#define Serial_println(x)  do { Serial.println(x); Serial1.println(x); } while (0)

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;

#define SAMPLES  15   
float values[SAMPLES];                                                       
unsigned long times[SAMPLES];  
int samples_index = 0;
float smooth_value = 0;
float prev_smooth_value = 0;

bool detectQuickChange(float val){
  // record a new sample                                                         
  values[samples_index] = val;                                 
  times[samples_index] = micros();                                             
   unsigned long stime = times[(samples_index+1)%SAMPLES];      // oldest sample                          
  float N1 = 0;                                                        
  float N2 = 0;                                          
  float N3 = 0;                                                   
  float D1 = 0;                                             
  float D2 = 0;                      
                                                                      
  for(int i=0; i<SAMPLES; i++)                  
  {
    int cc = (samples_index + i) % SAMPLES;
    N1+=(times[cc]-stime)*values[cc];                                          
    N2+=(times[cc]-stime);                                                  
    N3+=(values[cc]);                                                        
    D1+=(times[cc]-stime)*(times[cc]-stime);                                  
    D2+=(times[cc]-stime);                                                  
  }
  samples_index = (samples_index+1) % SAMPLES;
  
  float valuea = ((SAMPLES*N1)-N2*N3)/(SAMPLES*D1-D2*D2)*500;
  //Serial.print("valuea:");
  //Serial.print(valuea);
  valuea = valuea*valuea*valuea*valuea*valuea*valuea*valuea ;

  
  
  float alfa = 0.9;
  smooth_value = valuea*(1-alfa) + smooth_value*alfa;
  smooth_value = constrain(smooth_value, -5000, 10000) - 2000;
  
  //Serial.print("zero:");
  //Serial.print(0);
  //Serial.print(" dvesto: ");
  //Serial.print(200);
  //Serial.print(" smoo:");
  //Serial.println(smooth_value);

  bool quickChange = (prev_smooth_value < 0 && smooth_value > 0);
  prev_smooth_value = smooth_value;

  //Serial.print(" change:");
  //Serial.print(quickChange*100);
  //Serial.println();

  return quickChange;
}



// Define a custom TwoWire instance
#define SDA_GPIO 12
#define SCL_GPIO 13
TwoWire MyWire(0);  // ESP32-S2 only has I2C_NUM_0
AS5600 as5600(&MyWire);   

void setup()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ROTATION_SENSOR_PIN, INPUT);
  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 7, 6);

  Serial_println(__FILE__);
  Serial_print("AS5600_LIB_VERSION: ");
  Serial_println(AS5600_LIB_VERSION);
  Serial_println();

  MyWire.begin(SDA_GPIO, SCL_GPIO, 1000000); // 1 Mhz

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600.setPowerMode(3);
  as5600.setPowerMode(0);

  Serial_println(as5600.getAddress());

  //  as5600.setAddress(0x40);  //  AS5600L only
/*
  int b; 
  do {
    b = as5600.isConnected();
    Serial_print("Connect: ");
    Serial_println(b);
    delay(500);
  } while(!b);
 */ 
  delay(1000);

  setCpuFrequencyMhz(80);
}

float rps = 0; // rotations per second
unsigned long lastDetection = 0; 
unsigned long lastPrint = 0;
unsigned long ledOnTime = 0; 
int lastRotationRead = 0;
int allRotationCount = 0;
int rotationCount = 0;

void photoresistorLogSpeed() {

}

int readRotationState() {
  int start_value = analogRead(ANALOG_PIN);
  digitalWrite(LED_PIN, LOW);
  //delayMicroseconds(250);
  esp_sleep_enable_timer_wakeup(1000);
  int diff_value = - start_value + analogRead(ANALOG_PIN);
  digitalWrite(LED_PIN, HIGH);

  //Serial_print(diff_value);
  //Serial_print(" ");

  return diff_value > 100;
}


int speedSum = 0;
int speedCount = 0;
int rotationStateToCount = 0; // counting number of 0 in the row 
int rotationsCount = 0;
int rotationTriggered = 0;

void loop()
{
  static uint32_t lastTimeRead = 0;
  static uint32_t lastTimePrint = 0;
  static uint32_t lastTimeReadPhoto = 0;
  uint32_t now = millis();
  
  esp_sleep_enable_timer_wakeup(6 * 1000);
  esp_light_sleep_start();

  uint32_t t0 = micros(); 

  /*
  float lightValue = analogRead(PHOTORESISTOR_PIN);
  if(detectQuickChange(lightValue)) {
    allRotationCount ++;
    rotationCount ++; 
    rps += 1000.0f / (now - lastDetection);
    lastDetection = now; 
  }
  */

  //if (now - lastTimeReadPhoto >= 5)
  //{
    lastTimeReadPhoto = now;
    
    int rotationState = readRotationState();
    //Serial_print(rotationState);

    if (rotationStateToCount == rotationState) {
      rotationsCount += 1;
    } else {
      rotationsCount = 0;
    }

    if(rotationsCount == 3 && rotationTriggered == 0) {
      //rotationTriggered = 1;
      rotationsCount = 0;
      rotationStateToCount = !rotationStateToCount;

      //Serial_println();
      allRotationCount ++;
      rotationCount ++; 
      rps += 1000.0f / (now - lastDetection);
      lastDetection = now; 
    
      Serial_print("|");
    }
    
    
  //}

  
  /*
  
  if (now - lastTimeReadPhoto >= 2)
  {
    lastTimeReadPhoto = now;
    
    float lightValue = analogRead(PHOTORESISTOR_PIN);
    if(detectQuickChange(lightValue)) {
      allRotationCount ++;
      rotationCount ++; 
      rps += 1000.0f / (now - lastDetection);
      lastDetection = now; 
      
      //Serial.print(" rps: ");  
      //Serial.print(rps);
      //Serial.print("Rotations:");
      //Serial.print(rotationCount);
      //Serial.print(" ");
      //Serial.print(allRotationCount);
      //Serial.println();
    }  
  }


  if (now - lastTimeRead >= 5)
  {
    lastTimeRead = now;
    as5600.readAngle();
    int speed = -as5600.getAngularSpeed(AS5600_MODE_DEGREES, false);
    speedSum += speed;
    speedCount += 1;
    //as5600.setPowerMode(2);
    //Serial_print("speed:"); 
    //Serial_println(speed/360.0);
  }
  */

  uint32_t t1 = micros();  // End timing

  //Serial_print(" | Duration compute (us): ");
  //Serial_println(t1 - t0);

  if(now - lastTimePrint >= 1000) {
    lastTimePrint = now;
    int speed = 0;
    if(speedCount != 0) {
      speed = speedSum / speedCount;
    }
    speedSum = 0;
    speedCount = 0;

    float sped =  rotationCount == 0? 0 : rps / rotationCount;
    rotationCount = 0; 
    rps = 0;

    Serial_print("speed:"); 
    Serial_print(sped);
    //Serial_print("\t");
    //Serial_print(0);
    //Serial_print("\t");
    //Serial_print(1);
    //Serial_print("\tmagnet:");
    //Serial_println(speed / 360.0);
    Serial_println();


  }
}


//  -- END OF FILE --
