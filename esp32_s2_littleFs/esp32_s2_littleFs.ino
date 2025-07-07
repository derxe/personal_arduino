#define LED_PIN 11  // Change this to your actual LED pin
#include "Button2.h"
#define BUTTON_PIN  7

Button2 button;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("LED Toggle Starting...");

  button.begin(BUTTON_PIN);
  button.setTapHandler(tap);
  
}

void tap(Button2& btn) {
    Serial.println("click\n");
}

void loop() {
  button.loop();

}
