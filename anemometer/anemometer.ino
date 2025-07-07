#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define LCD_I2C_ADRESS  0x27
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
LiquidCrystal_I2C lcd(LCD_I2C_ADRESS, LCD_WIDTH, LCD_HEIGHT); 

// ----------------------------
// CONFIGURATION
// ----------------------------
#define SWITCH_PIN 12            // Pin for the switch
#define MAX_PRESS_TIMES 60       // Circular buffer length

// ----------------------------
// GLOBAL VARIABLES
// ----------------------------
volatile int pressCount = 0;                    // Simple total counter
volatile unsigned long pressTimes[MAX_PRESS_TIMES];  // Circular buffer for press timestamps
volatile uint8_t pressIndex = 0;                // Current buffer position

unsigned long lastTime = 0;   // For the once-per-interval display
unsigned long interval = 500; // e.g., 500 ms update interval

long last_time_count = 0;

// ----------------------------
// FORWARD DECLARATIONS
// ----------------------------
ICACHE_RAM_ATTR void countPresses();
float getPressesInLastMs(unsigned long period);

// ----------------------------
// INITIALIZATION
// ----------------------------
void init_screen() {
  lcd.init();
  lcd.backlight();
  lcd.noCursor();
  lcd.clear();
  lcd.setCursor(0, 0);
}

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(115200);  // Start serial communication
  Serial.println("Program start!");
  
  init_screen();
  Serial.println("Screen inited");
  
  pinMode(SWITCH_PIN, INPUT);  // Set the switch pin to input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), countPresses, RISING);
  Serial.println("Interupt set");
}

// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {
  unsigned long now = millis();

  // Update display every 'interval' ms
  if (now - lastTime >= interval) {
    lastTime = now;
    
    // We read the total press count safely
    noInterrupts();
    uint32_t totalCount = pressCount;
    interrupts();
    
    // Calculate how many presses in last 600 ms
    float pressesLast600ms = getPressesInLastMs(1000);
    
    // Print to Serial
    Serial.print("Total presses: ");
    Serial.print(totalCount);
    Serial.print("  Presses in last 600 ms: ");
    Serial.println(pressesLast600ms);
    
    // Update LCD
    lcd.setCursor(0, 0);
    lcd.print("Total: ");
    lcd.print(totalCount);
    
    lcd.setCursor(0, 1);
    lcd.print("Freq: ");
    lcd.print(pressesLast600ms);
    lcd.print(" Hz   ");
  }
}

// ----------------------------
// INTERRUPT SERVICE ROUTINE
// ----------------------------
ICACHE_RAM_ATTR void countPresses() {
  // Store a timestamp in the circular buffer
  unsigned long timeNow = millis();
  pressTimes[pressIndex] = timeNow;
  
  // Move the index forward in a circular manner
  pressIndex = (pressIndex + 1) % MAX_PRESS_TIMES;
  
  // Optional total count (you can remove if not needed)
  pressCount++;
}

// ----------------------------
// HELPER FUNCTIONS
// ----------------------------
float getPressesInLastMs(unsigned long period) {
  // Returns how many timestamps are within the last 'period' ms
  // We'll snapshot 'now' so the loop sees a consistent value
  unsigned long now = millis();
  
  // Reading a shared array in a critical section is safest
  // but for short reads it's often okay if we read the array quickly.
  // We'll do the safer approach with noInterrupts() / interrupts().
  noInterrupts();
  unsigned long timeStamps[MAX_PRESS_TIMES];
  uint8_t indexCopy = pressIndex;
  for (uint8_t i = 0; i < MAX_PRESS_TIMES; i++) {
    timeStamps[i] = pressTimes[i];
  }
  interrupts();
  
  unsigned long diff_sum = 0;
  uint8_t diff_count = 0;
  uint8_t count = 0;
  unsigned long min_value = -1;
  // We simply check all stored times to see if they are within [now - period, now]
  for (uint8_t i = 0; i < MAX_PRESS_TIMES; i++) {
    if (now - timeStamps[i] <= period) {
      if(i > 0) {
        diff_sum += timeStamps[i] - timeStamps[i-1];
        diff_count += 1;
      }
      count += 1;
    }
  }
  Serial.print("Count:");
  Serial.println(count);

  if (diff_count == 0) {
    return 0.0;   // or return -1, or whatever makes sense for "no presses"
  }
  
  return (1000.0 * diff_count) / diff_sum;
}
