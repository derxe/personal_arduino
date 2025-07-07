volatile int pulseCount = 0;
unsigned long lastCalcTime = 0;
float rpmSmoothed = 0.0;
const float alpha = 0.7;  // smoothing factor
const int markersPerRev = 10;
int printDelay = 200;

unsigned long sessionStart = 0;
bool newSession = true;

void IRAM_ATTR onPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(35, INPUT);
  attachInterrupt(digitalPinToInterrupt(35), onPulse, RISING);
  sessionStart = millis();  // initialize session timer
}

void loop() {
  // Check for serial input
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      Serial.println("start");
      sessionStart = millis();  // reset session time
      newSession = true;
    }
  }

  unsigned long now = millis();
  unsigned long elapsed = now - sessionStart;

  if (now - lastCalcTime >= printDelay) {
    noInterrupts();
    int pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = pulses * (60.0 / markersPerRev) * (1000.0 / printDelay);
    rpm /= 60;

    rpmSmoothed = rpm * alpha + rpmSmoothed * (1 - alpha);

    Serial.print("ms: ");
    Serial.print(elapsed);
    Serial.print(" | rpm: ");
    Serial.println(rpmSmoothed, 1);

    lastCalcTime = now;
  }
}
