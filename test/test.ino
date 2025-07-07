

#include <Button2.h>

Button2 buttonA;
Button2 buttonB;
#define BUTTON_A_PIN  12
#define BUTTON_B_PIN  14
#define BUZZER_PIN    15


void setup() {
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(clickA);

    buttonB.begin(BUTTON_B_PIN);
  buttonB.setTapHandler(clickB);

  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  buttonA.loop();
  buttonB.loop();

}


void clickA(Button2& btn) {
  tone(BUZZER_PIN, 1318);
  delay(100);
  tone(BUZZER_PIN, 1760);
  delay(150);
  tone(BUZZER_PIN, 2217);
  delay(200);
  noTone(BUZZER_PIN);
}

void clickB(Button2& btn) {
  tone(BUZZER_PIN, 987);
  delay(100);
  tone(BUZZER_PIN, 689);
  delay(100);
  noTone(BUZZER_PIN);
}
