#include "Button2.h"
#include "driver/pcnt.h"

#define PCNT_UNIT      PCNT_UNIT_0
#define PCNT_INPUT_PIN 4

#include <TinyPICO.h>
TinyPICO tp = TinyPICO();


void setup_pcnt() {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = PCNT_INPUT_PIN,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,
    .counter_l_lim = 0,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_set_filter_value(PCNT_UNIT, 100);
  pcnt_filter_enable(PCNT_UNIT);

  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}


#define MOTOR_PIN_A 26
#define MOTOR_PIN_B 25
#define BUTTON_PIN   14
#define BUTTON_PIN_2 15
Button2 button;
Button2 button2;

bool isShowingSpeed = false;
unsigned long showingSpeedStart = 0;

void buttonChanged(Button2& btn) {
  //Serial.println("BUtton changed!");
  if(button.isPressed())       setMotorPower(100);
  else if(button2.isPressed()) setMotorPower(-100);
  else setMotorPower(0);
}

volatile uint32_t pulseCount = 0;
#define SIGNAL_PIN  32

void IRAM_ATTR onTimer(void* arg) {
  readPin();
}

void readPin() {
    // Sample the pin and count rising edges
  static bool lastState = 0;
  bool current = digitalRead(SIGNAL_PIN);
  //Serial.print(current);
  if (current != lastState) {
    pulseCount++;
  }
  lastState = current;
}

hw_timer_t* timer = NULL;
void setup() {
  const esp_timer_create_args_t signal_timer_args = {
    .callback = &onTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "signal_timer"
  };

  esp_timer_handle_t signal_timer;
  esp_timer_create(&signal_timer_args, &signal_timer);
  esp_timer_start_periodic(signal_timer, 400);  // every 1000 µs = 1 ms = 1 kHz
  pinMode(SIGNAL_PIN, INPUT_PULLUP);

  analogWriteResolution(MOTOR_PIN_A, 8);
  analogWriteResolution(MOTOR_PIN_B, 8); 
  analogWriteFrequency(MOTOR_PIN_A, 20000);
  analogWriteFrequency(MOTOR_PIN_B, 20000);

  button.begin(BUTTON_PIN);
  button.setChangedHandler(buttonChanged);

  button2.begin(BUTTON_PIN_2);
  button2.setChangedHandler(buttonChanged);

  //setup_pcnt();

  tp.DotStar_SetPower(false);
  Serial.begin(115200);
  //while(!Serial) delay(1);
  Serial.println("Program strat");
}

int speedToMotorPower(int speed) {
  if(speed <= 15) return 0; // we cant really spin that slowly so we just return 0

  // posible speeds from 30 to 100 other speeds are not really achivable 
  double a = 0.00012159;
  double b =-0.00629095;
  double c = 0.26476363;
  double speedD = speed;

  int motorPower = a * speedD*speedD*speedD + b*speedD*speedD + c*speedD + 15;

  Serial.print("Motor power:"); Serial.print(motorPower); 
  Serial.print(" for speed:"); Serial.print(speed);
  Serial.println();
  
  return motorPower;
}

// motorPower from -100 to +100
void setMotor(int motorPower) {
  //Serial.print("motorPower:"); Serial.println(motorPower);
  motorPower = constrain(motorPower, -100, 100);
  int pwmValue = map(abs(motorPower), 0, 100, 0, 255);

  if (motorPower > 0) {
    analogWrite(MOTOR_PIN_A, pwmValue);
    analogWrite(MOTOR_PIN_B, 0);
  } else {
    analogWrite(MOTOR_PIN_A, 0);
    analogWrite(MOTOR_PIN_B, pwmValue);
  }
  if (motorPower == 0) delay(0);
}


int desiredMotorPower = 0;

void setMotorPower(int motorPower) {
  //Serial.print("Setting motorPower:"); Serial.println(motorPower);
  desiredMotorPower = constrain(motorPower, -100, 100);
}

int currentMotorPower = 0;
int setMotorSpeed = 0;

void updateMotorPower() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 10) return;
  lastUpdate = millis();
 
  // Step size: how fast the speed changes (1 unit per update = ~100 units/sec)
  const int step = 1;

  int nextMotorPower = currentMotorPower;

  if (desiredMotorPower > currentMotorPower)
    nextMotorPower += min(step, desiredMotorPower - currentMotorPower);
  else if (desiredMotorPower < currentMotorPower)
    nextMotorPower -= min(step, currentMotorPower - desiredMotorPower);

  if (nextMotorPower != currentMotorPower) {
    currentMotorPower = nextMotorPower;
    setMotor(currentMotorPower);
  }
}

static bool blinkLedState = false;

void loop() {
  button.loop();
  button2.loop();
  updateMotorPower();
  readSpeedFromSerial();

  static unsigned long lastCountPrint = 0;
  if(isShowingSpeed && millis() - lastCountPrint >= 100) {
    lastCountPrint = millis();
    
    unsigned long timestamp = millis() - showingSpeedStart;

    static int prevCount = 0;
    static int distanceCount = 0;
    static int prevDirection = 1;

    int16_t count = pulseCount;

    /*
    pcnt_get_counter_value(PCNT_UNIT, &count);
    */
    int direction = 1;
    if(currentMotorPower == 0) direction = prevDirection;
    else if(currentMotorPower < 0) direction = -1;
    else direction = 1; 
    prevDirection = direction;

    int speed = direction * (count - prevCount);
    distanceCount += speed;
    prevCount = count;

    if(timestamp/1000 % 2 == blinkLedState) {
      blinkLedState = !blinkLedState;
      tp.DotStar_SetPower(blinkLedState);
      tp.DotStar_SetPixelColor(255, 0, 0);
    }


    Serial.print(timestamp);
    Serial.print(":speed:"); Serial.print(speed);
    Serial.print(":dist:"); Serial.print(distanceCount);
    Serial.print(":motor_power:"); Serial.print(currentMotorPower);
    Serial.print(":motor_speed:"); Serial.print(setMotorSpeed);
    Serial.println();
  }
}


void readSpeedFromSerial() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        int speed = input.toInt();  // Handles signs too
        setMotorSpeed = speed;
        int motorPower = speedToMotorPower(speed);
        setMotorPower(motorPower);
        input = "";
      } else {
        isShowingSpeed = !isShowingSpeed;
        tp.DotStar_SetPower(isShowingSpeed);
        blinkLedState = false;
        showingSpeedStart = millis();
      }
    } else if (isDigit(c) || c == '-' || c == '+') {
      input += c;
    } else {
      input = ""; // Reset on invalid character
    }
  }
}