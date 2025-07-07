#include "Button2.h"

#define BUTTON_PIN  2
#define BUZZER_PIN 11   


Button2 button;

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  Serial.println("Beeper test!");

  pinMode(BUZZER_PIN, OUTPUT);

  button.begin(BUTTON_PIN);
  button.setTapHandler(tap);
}

void loop() {
  button.loop();
}

void tap(Button2& btn) {
  
  // read the input on analog pin A0:
  int analogValue = analogRead(A0);
  // Rescale to potentiometer's voltage (from 0V to 5V):
  float pitch = floatMap(analogValue, 0, 1023, 200, 5000);

  // print out the value you read:
  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(", pitch: ");
  Serial.println(pitch);

  tone(BUZZER_PIN, pitch, 1000);
}
