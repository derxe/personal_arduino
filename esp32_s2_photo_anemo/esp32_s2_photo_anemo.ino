//
//    FILE: AS5600_position_speed.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo compare performance with update flag
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#define Serial_print(x)    do { Serial.print(x); Serial1.print(x); } while (0)
#define Serial_println(x)  do { Serial.println(x); Serial1.println(x); } while (0)

#define LED_PIN 9
#define ANALOG_PIN 8


void IRAM_ATTR onTimer() {
  // This function runs every 5 ms
  // Keep it short! Set a flag, toggle a pin, etc.
}


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //WiFi.mode(WIFI_OFF);
  //esp_wifi_stop();

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
  Serial_println("Program start!");
  setCpuFrequencyMhz(80);
}

float rps = 0; // rotations per second
unsigned long lastDetection = 0; 
unsigned long lastPrint = 0;
unsigned long ledOnTime = 0; 
int lastRotationRead = 0;
int allRotationCount = 0;
int rotationCount = 0;

int readRotationState() {
  int start_value = analogRead(ANALOG_PIN);
  digitalWrite(LED_PIN, LOW);
  //delayMicroseconds(1000);
  esp_sleep_enable_timer_wakeup(1000);
  esp_light_sleep_start();
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
  static uint32_t lastTimePrint = 0;
  static uint32_t lastTimeReadPhoto = 0;
  uint32_t now = millis();
  
  esp_sleep_enable_timer_wakeup(10 * 1000);
  esp_light_sleep_start();

  uint32_t t0 = micros(); 


  if (now - lastTimeReadPhoto >= 0)
  {
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
  }


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
    Serial_println();
  }
}

