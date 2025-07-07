/////////////////////////////////////////////////////////////////

#include "Button2.h"

/////////////////////////////////////////////////////////////////

#define BUTTON_A_PIN  2
#define BUTTON_B_PIN  3
#define BUTTON_C_PIN  4

#define RELAY_1_PIN   7  // Light Control
#define RELAY_2_PIN   8  // Fan Speed Level 1
#define RELAY_3_PIN   9  // Fan Speed Level 2
#define RELAY_4_PIN   10 // Fan Speed Level 3

#define RELAY_ON      LOW
#define RELAY_OFF     HIGH

/////////////////////////////////////////////////////////////////

Button2 buttonA, buttonB, buttonC;
bool relay1State = false;
bool fanState = false; // Fan starts OFF
int fanPowerLevel = 1; // Start at level 1

/////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n\nMultiple Buttons Demo");
  
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(RELAY_4_PIN, OUTPUT);

  digitalWrite(RELAY_1_PIN, RELAY_OFF);    // Light off initially
  digitalWrite(RELAY_2_PIN, RELAY_OFF);     // Fan starts at level 1
  digitalWrite(RELAY_3_PIN, RELAY_OFF);
  digitalWrite(RELAY_4_PIN, RELAY_OFF);

  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(toggleLight);

  buttonB.begin(BUTTON_B_PIN);
  buttonB.setTapHandler(toggleFan);
  
  buttonC.begin(BUTTON_C_PIN);
  buttonC.setTapHandler(cycleFanPower);
}

/////////////////////////////////////////////////////////////////

void loop() {
  buttonA.loop();
  buttonB.loop();
  buttonC.loop();
}

/////////////////////////////////////////////////////////////////

void toggleLight(Button2& btn) {
   relay1State = !relay1State;
   digitalWrite(RELAY_1_PIN, relay1State ? RELAY_ON : RELAY_OFF);
   Serial.println("Light toggled");
}

void toggleFan(Button2& btn) {
   fanState = !fanState;
   if (fanState) {
       setRelaysForFan();
   } else {
       digitalWrite(RELAY_2_PIN, RELAY_OFF);
       digitalWrite(RELAY_3_PIN, RELAY_OFF);
       digitalWrite(RELAY_4_PIN, RELAY_OFF);
       Serial.println("Fan turned OFF");
   }
}

void cycleFanPower(Button2& btn) {
   if (fanState) {
       fanPowerLevel = (fanPowerLevel % 3) + 1;

       setRelaysForFan();

       Serial.print("Fan power level set to ");
       Serial.println(fanPowerLevel);
   }
}

void setRelaysForFan() {
    // Check the current state of the relays
    bool allOff = digitalRead(RELAY_2_PIN) == RELAY_OFF &&
                  digitalRead(RELAY_3_PIN) == RELAY_OFF &&
                  digitalRead(RELAY_4_PIN) == RELAY_OFF;

    // Turn off all relays
    digitalWrite(RELAY_2_PIN, RELAY_OFF);
    digitalWrite(RELAY_3_PIN, RELAY_OFF);
    digitalWrite(RELAY_4_PIN, RELAY_OFF);

    // Skip delay if they were already all off
    if (!allOff) {
        delay(200);
    }

    // Set relays based on fan power level
    digitalWrite(RELAY_2_PIN, fanPowerLevel == 1 ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_3_PIN, fanPowerLevel == 2 ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_4_PIN, fanPowerLevel == 3 ? RELAY_ON : RELAY_OFF);
}

/////////////////////////////////////////////////////////////////
