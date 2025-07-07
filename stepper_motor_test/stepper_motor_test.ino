#include <AFMotor.h>

// Create a stepper motor object with 48 steps per revolution (you'll update this!)
AF_Stepper motor(48, 2); // (steps, motor port 1 or 2)

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper Manual Test");

  // Set speed in RPM
  motor.setSpeed(250); // Slower is better for counting steps
}

void loop() {
  Serial.println("Rotating one step forward...");
  motor.step(100, FORWARD, SINGLE); // One step forward
  delay(00); // Delay for easy counting

  // To test reverse direction, uncomment below:
  // Serial.println("Rotating one step backward...");
  // motor.step(1, BACKWARD, SINGLE);
  // delay(1000);
}
