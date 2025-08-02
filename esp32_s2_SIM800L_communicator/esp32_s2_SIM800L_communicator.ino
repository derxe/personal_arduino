#include <SoftwareSerial.h>
#include "Button2.h"
#include "esp_timer.h"
#include <SerialMod0.h>
#include "AS5600.h"
#include <Wire.h>
#include <TimeLib.h>


#define UART_DEBUG_TX  13
#define UART_DEBUG_RX  14
SerialMod0 SerialUart0(UART_NUM_0, UART_DEBUG_TX, UART_DEBUG_RX); 
#define SerialDBG SerialUart0
#define Serial_print(x)    do { SerialDBG.print(x); /* Serial1.print(x);*/ } while (0)
#define Serial_println(x)  do { SerialDBG.println(x); /* Serial1.println(x);*/ } while (0)
#define Serial_write(x)  do { SerialDBG.write(x); /*Serial1.write(x);*/ } while (0)

#define RX_PIN 37
#define TX_PIN 35
//SoftwareSerial SerialAT(TX_PIN, RX_PIN);  // SIM800L <-> Arduino
#define SerialAT Serial1

#define BUTTON_PIN       3
//#define BUTTON_PIN_2     3
#define GPRS_ON_PIN     16
#define GPRS_POWER_PIN  39
#define V_BATT_PIN       12
#define V_SOALR_PIN      11
#define BOARD_LED_PIN    15
#define SPIN_LED_PIN     9

#define VANE_POWER_PIN    1
#define HAL_SENSOR_PIN    4

//#define TIMER_PIN 14

// Define a custom TwoWire instance
#define SDA_GPIO             10
#define SCL_GPIO             8      
#define AS600_POWER_PIN  6
AS5600 as5600; 


/*
ORANGE    10  SDA_GPIO
YELLOW    8   SCL_GPIO
GREEN     6   AS600_POWER_PIN
BROWN     4   HAL_SENSOR_PIN
RED       1   VANE_POWER_PIN

3v3  red
GND  white
as5600 power green
scl yellow
sda orange
hal pin brown

*/

#define LED_PIN 9
#define ANALOG_PIN 123

Button2 button;
Button2 button2;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastNow = 0;

void IRAM_ATTR onReadDirection(void* arg);
void IRAM_ATTR onReadHal(void* arg);
void IRAM_ATTR onReadSpeed(void* arg);
void IRAM_ATTR onBlinkLed(void* arg);
void IRAM_ATTR onSpinLed(void* arg);
void tap(Button2& btn);

#define DEEP_SLEEP_DURATION  3600ULL * 1000000 // value in microseconds so: one hour
//#define DEEP_SLEEP_DURATION  10 * 1000 * 1000  // 10 seconds
RTC_DATA_ATTR time_t timeBeforeSleep = 0;      // stores last time before deep sleep

void printVersionAndCompileDate() {
  Serial.print("Compiled on ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);
}

void setup() {
  //Serial.begin(115200);
  SerialDBG.begin(115200);
  Serial_println();
  Serial_println("### PROGRAM START! ###");
  Serial_print("Compiled on "); Serial_println(__DATE__ " " __TIME__);
  Serial_print("Version:"); Serial_println("0.1");

  if(timeBeforeSleep == 0) {
    Serial_println("Fresh start. Normal boot");
  } else {
    // timeWas set before sleep so we can check if is time to wake up already!
    setTime(timeBeforeSleep + DEEP_SLEEP_DURATION / 1000000);
    Serial_print("Woke up from deep sleep. Current time:"); Serial_println(getFormattedTimeString());
    evaluateIfDeepSleep();
  }

  SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // 33, 34);

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(GPRS_ON_PIN, INPUT); // off
  pinMode(GPRS_POWER_PIN, OUTPUT); digitalWrite(GPRS_POWER_PIN, HIGH); // off
  pinMode(BOARD_LED_PIN, OUTPUT);   digitalWrite(BOARD_LED_PIN, HIGH); // on
  pinMode(SPIN_LED_PIN, OUTPUT);   digitalWrite(SPIN_LED_PIN, LOW); // off
  //pinMode(TIMER_PIN, OUTPUT); digitalWrite(TIMER_PIN, LOW);
  pinMode(VANE_POWER_PIN, OUTPUT); digitalWrite(VANE_POWER_PIN, HIGH);  
  gpio_set_drive_capability((gpio_num_t) VANE_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(AS600_POWER_PIN, OUTPUT); digitalWrite(AS600_POWER_PIN, LOW);  
  //gpio_set_drive_capability((gpio_num_t) AS600_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);

  button.begin(BUTTON_PIN);
  button.setPressedHandler(tap);

  //button2.begin(BUTTON_PIN_2);
  //button2.setPressedHandler(tap2);

  // init the as5600 chip so we can read wind direction 
  Wire.begin(SDA_GPIO, SCL_GPIO); // 1 Mhz
  Wire.setClock(1000000UL);

  setCpuFrequencyMhz(80);

  const esp_timer_create_args_t hal_timer_args = {
    .callback = &onReadHal,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "hal_timer"
  };
  esp_timer_handle_t hal_timer;
  esp_timer_create(&hal_timer_args, &hal_timer);
  esp_timer_start_periodic(hal_timer, 4*1000);  // every 4 ms

  esp_timer_handle_t blinkLed_timer;
  const esp_timer_create_args_t blinkLed_args = {
    .callback = &onBlinkLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "blinkLed_timer"
  };
  esp_timer_create(&blinkLed_args, &blinkLed_timer);
  esp_timer_start_periodic(blinkLed_timer, 40 * 1000);  // 40 ms

  esp_timer_handle_t spinLed_timer;
  const esp_timer_create_args_t spinLed_args = {
    .callback = &onSpinLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "spinLed_timer"
  };
  esp_timer_create(&spinLed_args, &spinLed_timer);
  esp_timer_start_periodic(spinLed_timer, 20 * 1000); // 10 ms 

  esp_timer_handle_t readSpeed_timer;
  const esp_timer_create_args_t readSpeed_args = {
    .callback = &onReadSpeed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readSpeed_timer"
  };
  esp_timer_create(&readSpeed_args, &readSpeed_timer);
  esp_timer_start_periodic(readSpeed_timer, 1000 * 1000); // 1s

  esp_timer_handle_t readDirection_timer;
  const esp_timer_create_args_t readDirection_args = {
    .callback = &onReadDirection,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readDirection_timer"
  };
  esp_timer_create(&readDirection_args, &readDirection_timer);
  esp_timer_start_periodic(readDirection_timer, 10*1000); // 10 ms

  Serial_println("Done init!");
}


#define DIRECTIONS_LOG_LEN 20 
int directions_log[DIRECTIONS_LOG_LEN]; // speed logged each second for a short interval
volatile int directions_log_i = 0;

#define DIRECTIONS_AVG_LEN (60 / DIRECTIONS_LOG_LEN * 60 * 3) // averaged speed from the speeds_log array, each time the speeds_log array fills up this array saves the value
int directions_avg_i = 0;
uint16_t directions_avg[DIRECTIONS_AVG_LEN];

void IRAM_ATTR onReadDirection(void* arg) {
  static int directionReadCount = 0; 
  int angle = 0;
  Serial.flush();
  switch(directionReadCount) {
    case 1:
      digitalWrite(AS600_POWER_PIN, HIGH);
      break;
    
    case 2:
      angle = as5600.readAngle()*360 / 4096;
      //Serial_print("Read angle: "); Serial_println(angle);

      portENTER_CRITICAL_ISR(&timerMux);
      directions_log[directions_log_i++] = angle;
      if (directions_log_i == DIRECTIONS_LOG_LEN) {
        // the log is full so we calculate average and save the measurement into the avg log
        directions_log_i = 0;
        directions_avg[directions_avg_i++] = average_direction(directions_log, DIRECTIONS_LOG_LEN);
      }
      portEXIT_CRITICAL_ISR(&timerMux);

      digitalWrite(AS600_POWER_PIN, LOW);
      break;

    case 100:
      directionReadCount = 0;
      break;

  }
  directionReadCount++;
}

String getDirections() {
  uint16_t directions_avg_copy[DIRECTIONS_AVG_LEN];
  uint16_t directions_copy_len = 0;

  portENTER_CRITICAL(&timerMux);
  directions_copy_len = directions_avg_i;
  memcpy(directions_avg_copy, directions_avg, directions_copy_len * sizeof(directions_avg[0]));
  portEXIT_CRITICAL(&timerMux);

  if(directions_copy_len == 0) {
    return "";
  }

  String directions = String(directions_avg_copy[0]);
  for (int i = 1; i < directions_copy_len; i++) {
    directions += ",";
    directions += String(directions_avg_copy[i]);
  }

  return directions; 
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

void IRAM_ATTR onBlinkLed(void* arg) {
  static int nOnBlinkLedcalls = 0;
  nOnBlinkLedcalls ++;

  if(nOnBlinkLedcalls % (1000/40) == 0) {
    digitalWrite(BOARD_LED_PIN, HIGH);    
    //Serial_print("on");
  } else {
    digitalWrite(BOARD_LED_PIN, LOW); 
  }
}

volatile int rotation_detected_blink = 0;
void IRAM_ATTR onSpinLed(void* arg) {
  if(rotation_detected_blink == 1) {
    rotation_detected_blink = 0;
    digitalWrite(SPIN_LED_PIN, HIGH);    
    //Serial_print("on");
  } else {
    digitalWrite(SPIN_LED_PIN, LOW); 
  }
}

float rps = 0;
int rotationCount = 0;

#define SPEEDS_LOG_LEN 20 
uint16_t speeds_log[SPEEDS_LOG_LEN]; // speed logged each second for a short interval
volatile int speeds_log_i = 0;

#define SPEEDS_AVG_LEN (60 / SPEEDS_LOG_LEN * 60 * 3) // averaged speed from the speeds_log array, each time the speeds_log array fills up this array saves the value
uint32_t speeds_avg_time_start = 0; // when whas the first measurement loged into the speeds_avg array
int speed_avg_i = 0;
uint16_t speeds_avg[SPEEDS_AVG_LEN];
uint16_t speeds_max[SPEEDS_AVG_LEN];

volatile int timerState = 0;
volatile int photo_start_value = 0;
volatile uint32_t lastDetection = 0;


volatile int lastHalSensorRead = -1;
void IRAM_ATTR onReadHal(void* arg) {
  //digitalWrite(TIMER_PIN, HIGH);

  // Example logic: latch HAL sensor if triggered
  int halSensorRead = digitalRead(HAL_SENSOR_PIN);
  if (halSensorRead != lastHalSensorRead && halSensorRead == LOW) {
    uint32_t now = micros() / 1000;

    portENTER_CRITICAL_ISR(&timerMux);
    rotationCount ++;                    // number of rotations counted, used to average rps every second in a diferent interupt
    rotation_detected_blink = 1;

    //Serial_print("|");
    rps += 1000.0f / (now - lastDetection);
    portEXIT_CRITICAL_ISR(&timerMux);
    lastDetection = now; 
  }
  lastHalSensorRead = halSensorRead;
  //digitalWrite(TIMER_PIN, LOW); 
}

// read speed every second
void IRAM_ATTR onReadSpeed(void* arg) {
  portENTER_CRITICAL_ISR(&timerMux);
  float speed =  rotationCount == 0? 0 : rps / rotationCount;
  rotationCount = 0; 
  rps = 0;

  speeds_log[speeds_log_i] = int(speed * 10);
  if (speeds_log_i < SPEEDS_LOG_LEN){
    speeds_log_i ++;
    portEXIT_CRITICAL_ISR(&timerMux);

    //Serial_print("Speed: "); Serial_println(speed);
  } else {
    // the log is full so we calculate average and save the measurement into the avg log
    uint16_t maxSpeed = 0;
    int avgSpeedSum = 0;
    for(int i=0; i<SPEEDS_LOG_LEN; i++) {
      maxSpeed = max(maxSpeed, speeds_log[i]);
      avgSpeedSum += speeds_log[i]; 
    }
    int avgSpeed = avgSpeedSum / SPEEDS_LOG_LEN;
    if(speeds_avg_time_start == 0) speeds_avg_time_start = lastNow;
    speeds_avg[speed_avg_i] = avgSpeed;
    speeds_max[speed_avg_i] = maxSpeed;
    speed_avg_i ++;
    speeds_log_i = 0;
    portEXIT_CRITICAL_ISR(&timerMux);

    //Serial_print("Updated avg:"); Serial_print(avgSpeed);
    //Serial_print(", max:"); Serial_print(maxSpeed);
    //Serial_print(", i:"); Serial_print(speed_avg_i);
    //Serial_println();
  }
}

uint32_t lastSend = 0;
uint32_t lastPrint = 0;

bool isDeepSleepTime() {
  if(timeStatus() == timeNotSet) return false; // how can we sleep if we dont know what the time is!

  // we sleep at night ofcorse! from 8 PM to 6 AM
  if(20 <= hour() && hour() <= 24) return true;
  if(0 <= hour() && hour() < 6) return true;

  return false;
}

void evaluateIfDeepSleep() {
  if(isDeepSleepTime()) {
    Serial_print("Current time is:"); Serial_println(getFormattedTimeString());
    Serial_print("It is time to go deep sleep for: "); Serial_print(DEEP_SLEEP_DURATION/(1000000*60)); 
    Serial_print(" minutes!");
    delay(500); // delay for all the Serial_prints to finish 

    timeBeforeSleep = now();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    esp_deep_sleep_start();  // after this, it won't return here — will restart from setup()
  }
}

void loop() {
  button.loop();
  evaluateIfDeepSleep();

  if(millis() - lastSend > 10*60*1000) {
    Serial_print("10 min passed doing send");
    fullCycleSend();
    lastSend = millis();
  }

  lastNow = millis();

  //updateSerial();
  //delay(1);
  
  esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start(); // 1 seconds sleep 
}

double read_batt_v() {
  return analogRead(V_BATT_PIN) * 0.00065; 
}

double read_solar_v() {
  return analogRead(V_SOALR_PIN) * 0.0095;
}

void turnOnModule() {
  Serial1.println("turning on");

  for(int i=0; i<40;i++){
    pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW);
    delayMicroseconds(50*i);
    pinMode(GPRS_ON_PIN, INPUT);
    delay(3);
  }
  pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW);

  delay(1000);

  Serial_println("Turning on GPRS module!");
  digitalWrite(GPRS_POWER_PIN, LOW);
  Serial_println("waiting 1s ...");
  delay(1000);
  digitalWrite(GPRS_POWER_PIN, HIGH);
  Serial_println("Done");
}

void turnOffModule() {
  pinMode(GPRS_ON_PIN, INPUT);
  //digitalWrite(GPRS_ON_PIN, HIGH);
  Serial_println("gprs high -> turning off");
}

float vbatt = -1;
unsigned long httpGetStart = 0;
int signalStrength = -1;
int regDuration = -1;
int gprsRegDuration = -1;
//void tap(Button2& btn) {
//  fullCycleSend();
//}

void fullCycleSend() {
  //SerialAT.begin(9600, SERIAL_8N1, TX_PIN, RX_PIN); // 33, 34);
  vbatt = read_batt_v();
  signalStrength = -1;
  regDuration = -1;
  gprsRegDuration = -1;
  turnOnModule();
    
  httpGetStart = millis();
  runHttpGetHot();
  Serial_println("Finished hot hot hot");
  Serial_print("Duration:"); Serial_print((millis()-httpGetStart)/1000); Serial_println("s");

  turnOffModule();
  //Serial1.begin(9600, SERIAL_8N1, 33, 34);
  delay(1000);
}

void printDiagnosticInfo() {
  Serial_print("v batt:"); Serial_println(String(read_batt_v(), 2));
  Serial_print("v solar:"); Serial_println(String(read_solar_v(), 2));
  Serial_println("avg=" + getWindSpeeds());
  Serial_println("dirs=" + getDirections());

  Serial_print("Winds");
  for(int i=0; i < SPEEDS_LOG_LEN; i++) {
    Serial_print(speeds_log[i]); Serial_print(";");
  }
  Serial_println();

  Serial_print("Directions:");
  for(int i=0; i < DIRECTIONS_LOG_LEN; i++) {
    Serial_print(directions_log[i]); Serial_print(";");
  }
  Serial_println();
}

void tap(Button2& btn) {
  printDiagnosticInfo();

  fullCycleSend();
}

String inputBuffer = "";
bool updateSerial() {
  delay(100);
  
  while (SerialDBG.available()) {
    char c = SerialDBG.read();
    SerialDBG.write(c);
    //Serial_print("read:'");
    //Serial_print(c);
    //Serial_println("'");

    if (c == '\n' || c == '\r') {
      Serial_print("Command:'");
      Serial_print(inputBuffer);
      Serial_print("'\r\n");

      if (inputBuffer[0] == 'o' || inputBuffer[0] == 'O') {
        turnOnModule();
      } else if (inputBuffer[0] == 'x' || inputBuffer[0] == 'X') {
        turnOffModule();
      } else if (inputBuffer[0] == 's' || inputBuffer[0] == 'S') {
        fullCycleSend();
      } else if (inputBuffer[0] == 'i' || inputBuffer[0] == 'I') {
        printDiagnosticInfo();
      } else {
        SerialAT.println(inputBuffer.c_str());
        //SerialAT.write(inputBuffer.c_str());       //Forward what Serial received to Software Serial Port
      }
      
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  String atResponse = "";
  while (SerialAT.available()) {
    //Forward what Software Serial received to Serial Port
    char atRead = SerialAT.read();
    atResponse += atRead;
  }
  if(atResponse.length() > 0) {
    SerialDBG.print("AT:");
    SerialDBG.print(atResponse);
    SerialDBG.println();
  }

  return true;
}

String sendCommand(const String& command, int timeoutMs = 1000, String expectedResponse = "OK") {
  SerialAT.println(command);
  //Serial_print("#Command: ");
  //Serial_println(command);

  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      response += c;
      //Serial_write(c);

      // Early exit if we detect "OK"
      if (response.indexOf(expectedResponse) != -1) {
        //Serial_println(";");
        return response;
      }
    }
    delay(1); // Yield to avoid tight spinning
  }
  //Serial_println("norsps?");
  return ""; // how can I return "failed string"
}

const char* regStatusToStr(int status) {
  switch (status) {
    case 0: return "Not registered, not searching";
    case 1: return "Registered, home network";
    case 2: return "Not registered, searching";
    case 3: return "Registration denied";
    case 4: return "Unknown";
    case 5: return "Registered, roaming";
    default: return "Invalid status";
  }
}

bool parseCSQResponse(const String& response) {
  int idx = response.indexOf("+CSQ:");
  if (idx == -1) return false;

  // Extract substring starting after "+CSQ:"
  String sub = response.substring(idx + 5);
  sub.trim(); // remove leading/trailing whitespace

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int rssi = sub.substring(0, commaIdx).toInt();
  Serial_print("s:");
  Serial_print(rssi);
  signalStrength = rssi;
  return rssi > 0 && rssi < 99; // 99 = unknown or no signal
}

bool parseCGREGResponse(const String& response) {
  int idx = response.indexOf("REG:");
  if (idx == -1) return false;

  String sub = response.substring(idx + 4);
  sub.trim();

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int status = sub.substring(commaIdx + 1).toInt();

  //Serial_print("GPRS registration status: ");
  //Serial_println(regStatusToStr(status));
  Serial_print(status);
  return status == 1 || status == 5; // Registered (home or roaming)
}

String getFormattedTimeString() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  return String(buf);
}

bool parseCCLKResponse(const String& response) {
  // expected response example: +CCLK: "25/08/01,00:19:52+08"
  int idx = response.indexOf("CCLK:");
  if (idx == -1) return false;

  int quoteStart = response.indexOf('"', idx);
  int quoteEnd = response.indexOf('"', quoteStart + 1);
  if (quoteStart == -1 || quoteEnd == -1) return false;

  String datetime = response.substring(quoteStart + 1, quoteEnd);  // "25/08/01,00:19:52+08"

  // Split by delimiters: / , :
  int year   = datetime.substring(0, 2).toInt() + 2000;
  int month  = datetime.substring(3, 5).toInt();
  int day    = datetime.substring(6, 8).toInt();
  int hour   = datetime.substring(9, 11).toInt();
  int minute = datetime.substring(12, 14).toInt();
  int second = datetime.substring(15, 17).toInt();

  // Check for default time (00/01/01 etc.)
  if (year < 2023) return false;  // Reject obviously invalid time

  // save the time inside the TimeLib library
  setTime(hour, minute, second, day, month, year);

  // Print the parsed time
  Serial_print("Got date:"); Serial_println(getFormattedTimeString());
  
  return true;
}

bool parseHTTPREADResponse(const String& response) {
  int startIdx = response.indexOf("+HTTPREAD:");
  if (startIdx == -1) {
    Serial_println("No +HTTPREAD header found");
    return false;
  }

  // Find the first line break after the header
  int dataStart = response.indexOf('\n', startIdx);
  if (dataStart == -1) return false;

  // Trim until actual data
  String data = response.substring(dataStart + 1);
  data.trim();

  // Remove any trailing "OK" or extra content
  int okIdx = data.indexOf("OK");
  if (okIdx != -1) {
    data = data.substring(0, okIdx);
    data.trim();
  }

  Serial_print("HTTP payload: ");
  Serial_println(data);
  return true;
}

bool waitForResponse(const String& command,
                     unsigned long timeoutS,
                     bool (*parseFn)(const String&),
                     unsigned long retryDelay = 1000) {
  String response = "";
  unsigned long start = millis();
  bool responseOk = false;

  do {
    response = sendCommand(command);
    responseOk = parseFn ? parseFn(response) : (response != "");

    if (!responseOk) {
      if (millis() - start > timeoutS*1000) {
        Serial_println("No response within timeout, aborting.");
        return false;
      }
      delay(retryDelay); // wait before retrying
    }
  } while (!responseOk);

  Serial_print(command); Serial_print(" done. Duration:"); Serial_print((millis() - start)/1000.0); Serial_println("s");
  return true;
}

String zeros(int length) {
  String result;
  result.reserve(length);
  for (int i = 0; i < length; ++i) {
    result += '0';
  }
  return result;
}


String sendDataCommand() {
  String url = "http://138.199.212.169:4123/efiG1YOenDEmsN6/save";
  url += "?vbatIde=" + String(vbatt, 2);
  url += "&vbatGprs=" + String(read_batt_v(), 2);
  url += "&vsol=" + String(read_solar_v(), 1);
  url += "&dur=" + String((millis() - httpGetStart) / 1000.0, 1);
  url += "&signal=" + String(signalStrength);
  url += "&regDur=" + String(regDuration / 1000.0, 1);
  url += "&gprsRegDur=" + String(gprsRegDuration / 1000.0, 1);
  url += "&winds=" + zeros(10);

  String command = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  return command;
}

int speed_avg_i_on_send = 0;
String getWindSpeeds() {
  uint16_t speeds_avg_copy[SPEEDS_AVG_LEN];
  uint16_t speeds_max_copy[SPEEDS_AVG_LEN];
  uint16_t speeds_copy_len = 0;

  portENTER_CRITICAL(&timerMux);
  speeds_copy_len = speed_avg_i;
  speed_avg_i_on_send = speed_avg_i; // we save the index on when we send the data so we can see if there is any new data when we restart the index after successful send 
  memcpy(speeds_avg_copy, speeds_avg, speeds_copy_len * sizeof(speeds_avg[0]));
  memcpy(speeds_max_copy, speeds_max, speeds_copy_len * sizeof(speeds_max[0]));
  portEXIT_CRITICAL(&timerMux);

  // TODO add when the data was added ike the time when the first data was recorder

  if(speeds_copy_len == 0) {
    return "avg=;max=";
  }

  String windSpeeds = "avg=";
  windSpeeds += String(speeds_avg_copy[0]);
  for(int i=1; i<speeds_copy_len; i++) {
    windSpeeds += ",";
    windSpeeds += String(speeds_avg_copy[i]);
  }

  windSpeeds += ";max=";
  windSpeeds += String(speeds_max_copy[0]);
  for(int i=0; i<speeds_copy_len; i++) {
    windSpeeds += ",";
    windSpeeds += String(speeds_max_copy[i]);
  }
  
  return windSpeeds; 
}

String getPostBody() {
  String body = "";
  body += "vbatIde=" + String(vbatt, 2) + ";";
  body += "vbatGprs=" + String(read_batt_v(), 2) + ";";
  body += "vsol=" + String(read_solar_v(), 1) + ";";
  body += "dur=" + String((millis() - httpGetStart) / 1000.0, 1) + ";";
  body += "signal=" + String(signalStrength) + ";";
  body += "regDur=" + String(regDuration / 1000.0, 1) + ";";
  body += "gprsRegDur=" + String(gprsRegDuration / 1000.0, 1) + ";";
  body += "dirs=" + getDirections() + ";";
  body += getWindSpeeds() + ";"; 

  return body;
}

String waitForHttpActionResponse(unsigned long timeoutMs) {
  unsigned long start = millis();
  String line = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      Serial_write(c);
      if (c == '\n') {
        if (line.startsWith("+HTTPACTION:")) {
          return line;
        }
        line = "";  // reset for next line
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  Serial_println("Timeout!");
  return "";  // timeout
}

bool sendPOST() {
  if (sendCommand("AT+HTTPINIT", 1000) == "") return false;
  if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
  if (sendCommand("AT+HTTPPARA=\"URL\",\"http://138.199.212.169:4123/efiG1YOenDEmsN6/save\"", 500) == "") return false;

  String postData = getPostBody();
  if (sendCommand("AT+HTTPDATA=" + String(postData.length()) + ",10000", 500, "DOWNLOAD") == "") return false;
  if (sendCommand(postData, 1000) == "") return false;
  Serial_println("Done sending postData!");

  if (sendCommand("AT+HTTPACTION=1", 5000) == "") return false; // 0 = GET, 1 = POST, 2 = HEAD
  String actionResult = waitForHttpActionResponse(10000);  // wait up to 10s

  if (actionResult.indexOf(",2") > 0) {
    return true;
  } else {
    return false;
  }

  return true;
}

bool sendGet() {
  // Step 3: Initialize HTTP
  if (sendCommand("AT+HTTPINIT", 1000) == "") return false;
  if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
  if (sendCommand(sendDataCommand(), 1000) == "") return false;

  // Step 4: Perform HTTP GET
  if (sendCommand("AT+HTTPACTION=0", 5000) == "") return false; // 0 = GET, 1 = POST, 2 = HEAD

  return true;
}


void runHttpGetHot() {
  Serial_println("\n\nExecuting HTTP GET HOT...");
  
  if (!waitForResponse("AT", 10, nullptr)) return;

  if (!waitForResponse("AT+CSQ", 20, parseCSQResponse, 2000)) return;
  unsigned long start = millis();
  if (!waitForResponse("AT+CREG?", 120, parseCGREGResponse, 2000)) return;
  regDuration = millis() - start;
  
  start = millis(); 
  if (!waitForResponse("AT+CGREG?", 120, parseCGREGResponse, 2000)) return;
  gprsRegDuration = millis() - start;

  if (!waitForResponse("AT+CCLK?", 10, parseCCLKResponse, 2000)) return;

  // Step 1: Configure GPRS connection
  if (sendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000) == "") return;
  if (sendCommand("AT+SAPBR=3,1,\"APN\",\"internet.simobil.si\"") == "") return;
  if (sendCommand("AT+SAPBR=3,1,\"USER\",\"simobil\"") == "") return;
  if (sendCommand("AT+SAPBR=3,1,\"PWD\",\"internet\"", 2000) == "") return;

  // Step 2: Open GPRS bearer
  if (sendCommand("AT+SAPBR=1,1", 15000) == "") return;
  if (sendCommand("AT+SAPBR=2,1", 1000) == "") return;

  if (!sendPOST()) return;
  waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

  // Step 5: Cleanup
  sendCommand("AT+HTTPTERM");     // Terminate HTTP service
  sendCommand("AT+SAPBR=0,1");    // Close GPRS bearer

  Serial_print("Success!");

  portENTER_CRITICAL(&timerMux);
  Serial_print("We need to move: "); 
  Serial_print(speed_avg_i - speed_avg_i_on_send);
  Serial_print(" data ...");
  for(int i=speed_avg_i_on_send; speed_avg_i_on_send + i < speed_avg_i; i++) {
    speeds_avg[i] = speeds_avg[speed_avg_i_on_send + i];
    speeds_max[i] = speeds_max[speed_avg_i_on_send + i];
  }
  speeds_avg_time_start = millis();
  speed_avg_i = 0;
  directions_avg_i = 0;
  portEXIT_CRITICAL(&timerMux);
}

