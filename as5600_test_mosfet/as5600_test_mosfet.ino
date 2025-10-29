
#include "AS5600.h"
#include <Wire.h>

// Define a custom TwoWire instance
#define SDA_GPIO 5
#define SCL_GPIO 7
#define AS600_POWER_MOS_PIN 4
AS5600 as5600;   

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define DIRECTIONS_LOG_LEN 20 
int directions_log[DIRECTIONS_LOG_LEN]; // speed logged each second for a short interval
volatile int directions_log_i = 0;

#define DIRECTIONS_AVG_LEN (60 / DIRECTIONS_LOG_LEN * 60 * 3) // averaged speed from the speeds_log array, each time the speeds_log array fills up this array saves the value
int directions_avg_i = 0;
uint16_t directions_avg[DIRECTIONS_AVG_LEN];


int directionReadTimerInterval = 10*1000; // 10 ms
// read speed every second
void IRAM_ATTR onReadDirection(void* arg) {
  static int directionReadCount = 0; 
  int angle = 0;
  Serial.flush();
  switch(directionReadCount) {
    case 1:
      pinMode(AS600_POWER_MOS_PIN, OUTPUT);
      break;
    
    case 2:
      angle = as5600.readAngle()*360 / 4096;

      portENTER_CRITICAL_ISR(&timerMux);
      directions_log[directions_log_i++] = angle;
      if (directions_log_i == DIRECTIONS_LOG_LEN) {
        // the log is full so we calculate average and save the measurement into the avg log
        directions_log_i = 0;
        directions_avg[directions_avg_i++] = average_direction(directions_log, DIRECTIONS_LOG_LEN);
      }
      portEXIT_CRITICAL_ISR(&timerMux);

      pinMode(AS600_POWER_MOS_PIN, HIGH);
      break;

    case 100:
      directionReadCount = 0;
      break;

  }

  directionReadCount++;
}

float average_direction(const int* directions_log, size_t len) {
  float sum_sin = 0.0;
  float sum_cos = 0.0;

  for (size_t i = 0; i < len; i++) {
    float radians = directions_log[i] * DEG_TO_RAD;
    sum_cos += cos(radians);
    sum_sin += sin(radians);
  }

  float avg_angle = atan2(sum_sin, sum_cos) * RAD_TO_DEG;
  if (avg_angle < 0) avg_angle += 360.0;  // Normalize to [0, 360)
  return avg_angle;
}

String getDirections() {
  uint16_t directions_avg_copy[DIRECTIONS_AVG_LEN];
  uint16_t directions_copy_len = 0;

  portENTER_CRITICAL(&timerMux);
  directions_copy_len = directions_avg_i;
  memcpy(directions_avg_copy, directions_avg, directions_copy_len * sizeof(directions_avg[0]));
  portEXIT_CRITICAL(&timerMux);

  String directions = "";
  for(int i=0; i<directions_copy_len; i++) {
    directions += String(directions_avg_copy[i]) + ";";
  }

  return directions; 
}

void setup()
{
  Serial.begin(115200);
   esp_log_level_set("i2c.master", ESP_LOG_NONE); 

  pinMode(AS600_POWER_MOS_PIN, OUTPUT);
  digitalWrite(AS600_POWER_MOS_PIN, LOW); 

  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin(SDA_GPIO, SCL_GPIO); // 1 Mhz
  Wire.setClock(1000000UL);
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600.setPowerMode(3);
  as5600.setPowerMode(0);
  Serial.println(as5600.getAddress());

/*
  esp_timer_handle_t readDirection_timer;
  const esp_timer_create_args_t readDirection_args = {
    .callback = &onReadDirection,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readDirection_timer"
  };
  esp_timer_create(&readDirection_args, &readDirection_timer);
  esp_timer_start_periodic(readDirection_timer, directionReadTimerInterval);
*/
  //as5600.setAddress(0x40);  //  AS5600L only
  /*
  int b; 
  do {
    b = as5600.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);
    delay(500);
  } while(!b);
  */
}

void loop()
{
  static uint32_t lastTimeRead = 0;
  uint32_t start, end;


  uint32_t now = millis();
  if (now - lastTimeRead >= 200)
  {
    lastTimeRead = now;

    /*
    for(int i=0; i < DIRECTIONS_LOG_LEN; i++) {
      Serial.print(directions_log[i]); Serial.print(";");
    }
    start = micros();
    float avg = average_direction(&directions_log[0], DIRECTIONS_LOG_LEN);
    end = micros();
    Serial.print("avg:"); Serial.print(avg);
    Serial.print(" Duration:"); Serial.print(end - start);
    Serial.print(getDirections());
    Serial.println();
    */

    start = micros();
    digitalWrite(AS600_POWER_MOS_PIN, HIGH);
    int detectMagnet = 0;
    delay(5);
    int countFailedMagnetDetection = 0;
    for(int i=0; i<50 && detectMagnet == 0; i++) {
      detectMagnet = as5600.detectMagnet();
      countFailedMagnetDetection += 1;
      delayMicroseconds(100);
    }

    Serial.print(countFailedMagnetDetection); Serial.print(":");

    delayMicroseconds(100);

    delay(10);
    
    
    int angle = as5600.readAngle()*360 / 4096;
    delay(4);
    int angle1 = as5600.readAngle()*360 / 4096;
    delay(4);
    int angle2 = as5600.readAngle()*360 / 4096;
    delay(4);
    int angle3 = as5600.readAngle()*360 / 4096;
    end = micros();

    Serial.print(" Angle: "); Serial.print(angle); Serial.print(","); Serial.print(angle1); Serial.print(","); Serial.print(angle2); Serial.print(","); Serial.print(angle3);
    Serial.print(" Duration:"); Serial.print(end - start);
    Serial.print(" readMagnitude:"); Serial.print(as5600.readMagnitude());
    Serial.print(" detectMagnet:"); Serial.print(as5600.detectMagnet());
    Serial.print(" magnetTooStrong:"); Serial.print(as5600.magnetTooStrong());
    Serial.print(" magnetTooWeak:"); Serial.print(as5600.magnetTooWeak());
    Serial.println();

    digitalWrite(AS600_POWER_MOS_PIN, LOW);

  }
  
}


//  -- END OF FILE --
