#include "FS.h"
#include "LittleFS.h"
#include "Button2.h"

Button2 button;
Button2 button2;
void tap(Button2& btn);
void tap2(Button2& btn);
#define BUTTON_PIN        9
#define BUTTON_PIN_2      11

#define HAL_POWER_PIN     5
#define HAL_SENSOR_PIN    7

#define ANEMO_PIN         12

#define TIMER_PIN         4

#define UART_DEBUG_TX     21
#define UART_DEBUG_RX     17

#define BLINK_LED_PIN     15
#define SPIN_LED_PIN      3
#define ANEMO_LED_PIN     2

//#define Serial Serial1


void setup() {
  //Serial1.begin(115200, SERIAL_8N1, UART_DEBUG_RX, UART_DEBUG_TX);
  Serial.begin(115200);
  //while(!Serial) delay(1000);
  pinMode(HAL_POWER_PIN, OUTPUT); digitalWrite(HAL_POWER_PIN, HIGH);  
  gpio_set_drive_capability((gpio_num_t) HAL_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(ANEMO_PIN, INPUT_PULLUP);
  pinMode(TIMER_PIN, OUTPUT); digitalWrite(TIMER_PIN, LOW);
  pinMode(SPIN_LED_PIN, OUTPUT); digitalWrite(SPIN_LED_PIN, LOW);    
  pinMode(ANEMO_LED_PIN, OUTPUT); digitalWrite(ANEMO_LED_PIN, LOW);
  pinMode(BLINK_LED_PIN, OUTPUT); digitalWrite(BLINK_LED_PIN, HIGH);         


  button.begin(BUTTON_PIN);
  button.setPressedHandler(tap);

  button2.begin(BUTTON_PIN_2);
  button2.setPressedHandler(tap2);

  bool fsOk = false;
  fsOk = LittleFS.begin();
  if(!fsOk) {
    Serial.println("LittleFs mount failed formatting ...");
    fsOk = LittleFS.begin(true);
  }

  while(!fsOk) {
    Serial.println("LittleFS Mount Failed");
    delay(1000);
  }
  Serial.println("LittleFS success");
}

void saveValues() {
  File file = LittleFS.open("/values.txt", "w");
  file.println("hello world");
  file.close();
}

int lastHalSensorRead = -1;
int rotation_detected_blink = 1;
uint32_t rotation_detected_blink_time = 0;
int rotationCount = 0;
float rps = 0;
uint32_t lastDetection = 0;

void readHal() {
  //digitalWrite(TIMER_PIN, HIGH);

  // Example logic: latch HAL sensor if triggered
  int halSensorRead = digitalRead(HAL_SENSOR_PIN);
  if (halSensorRead != lastHalSensorRead && halSensorRead == LOW) {
    uint32_t now = micros() / 1000;

    rotationCount ++;                    // number of rotations counted, used to average rps every second in a diferent interupt
    Serial.print("|");
    rotation_detected_blink = 1;
    digitalWrite(SPIN_LED_PIN, HIGH);
    rotation_detected_blink_time = millis();
    rps += 1000.0f / (now - lastDetection);
    lastDetection = now; 
  }
  lastHalSensorRead = halSensorRead; 

  //digitalWrite(TIMER_PIN, LOW);
}

int lastAnemoSensorRead = 0;
int anemoCount = 0;
int anemoBlinkCount = 0;
int anemo_detected_blink = 0;
uint32_t anemo_detected_blink_time = 0;
void readAnemo() {
  // Example logic: latch HAL sensor if triggered
  int anemoSensorRead = digitalRead(ANEMO_PIN);
  if (anemoSensorRead != lastAnemoSensorRead && anemoSensorRead == LOW) {
    uint32_t now = micros() / 1000;

    anemoCount ++;                    // number of rotations counted, used to average rps every second in a diferent interupt
    //Serial.print("!");
    //rps += 1000.0f / (now - lastDetection);
    //lastDetection = now; 
    if(anemoBlinkCount > 100) {
      anemoBlinkCount = 0;
      anemo_detected_blink = 1;
      anemo_detected_blink_time = millis();
      digitalWrite(ANEMO_LED_PIN, HIGH);
    }
    anemoBlinkCount ++;

  }
  lastAnemoSensorRead = anemoSensorRead; 
}



float readSpeed() {
  float speed =  rotationCount == 0? 0 : rps / rotationCount;
  rotationCount = 0; 
  rps = 0;

  Serial.print("Speed: "); Serial.println(speed);
  return speed;
}

float readAnemoSpeed() {
  float anemoSpeed = anemoCount*14.3/300;
  Serial.print("Anemo: "); Serial.println(anemoSpeed, 2);
  anemoCount = 0; 
  return anemoSpeed;
}

static void printWholeFile(const char* path) {
  File r = LittleFS.open(path, "r");
  if (!r) { Serial.println("values.txt not found"); return; }
  while (r.available()) Serial.write(r.read());
  r.close();
}

float logSpeeds[1000];
float anemoSpeeds[1000];
int logi = 0;

void tap(Button2& btn) {
  if (!LittleFS.begin()) { Serial.println("LittleFS mount failed"); return; }

  // append a new line of pairs: log,anemo;log,anemo;...<newline>
  File file = LittleFS.open("/values.txt", "a");
  if (!file) { Serial.println("open for append failed"); return; }

  for (size_t i = 0; i < logi; ++i) {
    file.print(logSpeeds[i], 2);
    file.print(',');
    file.print(anemoSpeeds[i], 2);
    file.print(';');
  }
  file.print('\n');
  file.close();

  // print the whole file after saving
  Serial.println("----- values.txt -----");
  printWholeFile("/values.txt");
  Serial.println("\n----------------------");
  logi = 0;
}

void tap2(Button2& btn) {
  if (!LittleFS.begin()) { Serial.println("LittleFS mount failed"); return; }
  if (LittleFS.exists("/values.txt")) {
    LittleFS.remove("/values.txt");
    Serial.println("values.txt removed");
  } else {
    Serial.println("values.txt did not exist");
  }
}


void loop() {
  button.loop();
  button2.loop();

  readHal();
  readAnemo();

  uint32_t now = millis();
  if(rotation_detected_blink==1 && now - rotation_detected_blink_time > 50) {
    rotation_detected_blink = 0;
    digitalWrite(SPIN_LED_PIN, LOW);

  }

  if(anemo_detected_blink==1 && now - anemo_detected_blink_time > 50) {
    anemo_detected_blink = 0;
    digitalWrite(ANEMO_LED_PIN, LOW);
  }

  static uint32_t lastTimeBlink = 0;
  static int blinkState = 0;
  if(now - lastTimeBlink > 500) {
    lastTimeBlink = now;
    digitalWrite(BLINK_LED_PIN, blinkState? HIGH : LOW);
    blinkState = !blinkState;    
  }

  static uint32_t lastSpeedReed = 0;

  if(now - lastSpeedReed > 1*1000) {
    lastSpeedReed = now;
    float speed = readSpeed();
    float anemoSpeed = readAnemoSpeed();

    logSpeeds[logi] = speed;
    anemoSpeeds[logi] = anemoSpeed;
    logi += 1;
  }
}
