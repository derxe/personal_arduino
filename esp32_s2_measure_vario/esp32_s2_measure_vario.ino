#define LED_PIN 15

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("ESP32 S2 start");
  
  pinMode(LED_PIN, OUTPUT);
}

long lastLedOn = 0;
int count = 0;
long lastHigh = 0;
long beepStart = 0;
bool hasPrinted = true;
bool isHigh = false;

void loop() {
  long now = millis();
  
  int vario_beep_value = analogRead(1);
  if(vario_beep_value > 400) {
    lastHigh = now;
    if(hasPrinted) {
      beepStart = now;  
    }
    hasPrinted = false;

    if(!isHigh){
      count += 1;
      if(count > 20) {
        Serial.print("at;");
        Serial.print(beepStart / 1000.0, 3);
        Serial.print(";s;");
        Serial.print(lastHigh - beepStart);
        Serial.print(";ms;");
        Serial.print(count*1000.0/ (lastHigh - beepStart));
        Serial.println(";Hz;");

        count = 0;
        beepStart = now;
      }
    }
    isHigh = true;
  } else {
    isHigh = false;  
  }

  if(!hasPrinted && now - lastHigh > 100) {
    hasPrinted = true;
    if(count > 15) {
      Serial.print("at;");
      Serial.print(beepStart / 1000.0, 3);
      Serial.print(";s;");
      Serial.print(lastHigh - beepStart);
      Serial.print(";ms;");
      Serial.print(count*1000.0/ (lastHigh - beepStart));
      Serial.println(";Hz;");
    }
    count = 0;  
  }
}

/*
const int sensorPin = 1;     // Analog input pin for the beeper
const int threshold = 400;    // Threshold value to consider the signal as HIGH
const unsigned long quietThreshold = 100; // Quiet time in ms to consider beep ended

// Variables for beep detection and measurement
bool lastState = false;          // Last state of the beeper (HIGH/LOW)
unsigned long beepStartTime = 0; // Time (ms) when beep started
unsigned long lastHighTime = 0;  // Last time (ms) we detected a high reading
unsigned long lastRisingMicros = 0; // Time (µs) of last rising edge
unsigned long sumPeriods = 0;    // Sum of periods between rising edges (µs)
unsigned int periodCount = 0;    // Number of periods measured



void loop() {
  int sensorVal = analogRead(sensorPin);
  // Determine if the signal is "high"
  bool currentState = (sensorVal > threshold);
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  // Detect a rising edge: current HIGH and previous LOW.
  if (currentState && !lastState) {
    // If this is the very first rising edge of a beep, mark the start time.
    if (beepStartTime == 0) {
      beepStartTime = currentMillis;
    } else {
      // For subsequent rising edges, measure the period.
      if (lastRisingMicros != 0) {
        unsigned long period = currentMicros - lastRisingMicros;
        sumPeriods += period;
        periodCount++;
      }
    }
    // Update the last rising edge time.
    lastRisingMicros = currentMicros;
    // Update the last time we saw a high.
    lastHighTime = currentMillis;
  }

  // Keep updating the lastHighTime if the signal is high.
  if (currentState) {
    lastHighTime = currentMillis;
  }

  // If we are in the middle of a beep but haven't seen a high for a while,
  // consider the beep ended.
  if (beepStartTime != 0 && (currentMillis - lastHighTime > quietThreshold)) {
    unsigned long beepDuration = currentMillis - beepStartTime; // Duration in ms

    // Calculate the average period and then frequency in Hz
    float frequency = 0;
    if (periodCount > 0) {
      float avgPeriod = sumPeriods / (float) periodCount; // in microseconds
      frequency = 1000000.0 / avgPeriod; // convert microseconds to seconds
    }
    
    // Print the absolute start time (in seconds, 3 decimal places),
    // beep duration in ms, and frequency in Hz.
    Serial.print("Beep started at: ");
    Serial.print(beepStartTime / 1000.0, 3);
    Serial.print(" sec, ");
    Serial.print("Beep Duration: ");
    Serial.print(beepDuration);
    Serial.print(" ms, Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
    
    // Reset variables for the next beep
    beepStartTime = 0;
    lastRisingMicros = 0;
    sumPeriods = 0;
    periodCount = 0;
  }
  
  // Save the current state for the next loop iteration
  lastState = currentState;
}
*/
