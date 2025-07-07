/****************************************************************************************************************************
  SwitchCounterWithTimerInterrupt.ino

  This example uses the ESP8266TimerInterrupt library to trigger an interrupt every 1 millisecond.
  In the timer ISR, we toggle a pin (pin 16) and check the state of a switch connected to pin 12.
  If a LOW-to-HIGH transition is detected on the switch, we increment a counter.

  Make sure to install the ESP8266TimerInterrupt library:
    https://github.com/khoih-prog/ESP8266TimerInterrupt
*****************************************************************************************************************************/

// These defines must be placed at the beginning before including "ESP8266TimerInterrupt.h"
// Set the debug level if desired (set to 0 for no debug prints in ISR)
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "ESP8266TimerInterrupt.h"

// Define the pins
#define SWITCH_PIN   12   // The pin connected to the switch
#define TOGGLE_PIN   14   // The pin that will be toggled every millisecond

// Timer interval in microseconds (1ms = 1000 microseconds)
#define TIMER_INTERVAL_US  1000

// Global volatile variables used in ISR and main code
volatile uint32_t risingEdgeCount = 0;     // Count for LOW-to-HIGH transitions
volatile int lastSwitchState = LOW;        // To store the last state of the switch

// Initialize ESP8266 timer 1 using the ESP8266TimerInterrupt library
ESP8266Timer ITimer;

#include <LiquidCrystal_I2C.h>

#define LCD_I2C_ADRESS  0x27
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
LiquidCrystal_I2C lcd(LCD_I2C_ADRESS, LCD_WIDTH, LCD_HEIGHT); 

void init_screen() {
  lcd.init();
  lcd.backlight();
  lcd.noCursor();
  lcd.clear();
  lcd.setCursor(0, 0);
}

#define TRIGGER_BUGGER_LEN 50       // Circular buffer length
volatile unsigned long triggerTimes[TRIGGER_BUGGER_LEN];  
volatile uint8_t triggerTimesIndex = 0;


//=======================================================================
//                      Timer ISR Handler
//=======================================================================
void IRAM_ATTR TimerHandler()
{
  // Toggle the state of TOGGLE_PIN (pin 16)
  digitalWrite(TOGGLE_PIN, !digitalRead(TOGGLE_PIN));

  // Read the current state of the switch on SWITCH_PIN (pin 12)
  int currentState = digitalRead(SWITCH_PIN);

  // Check for a rising edge: previous state LOW and current state HIGH
  if (lastSwitchState == LOW && currentState == HIGH)
  {
    risingEdgeCount++;

    triggerTimes[triggerTimesIndex] = millis();
    triggerTimesIndex = (triggerTimesIndex + 1) % TRIGGER_BUGGER_LEN;
  }

  // Save the current state for the next interrupt
  lastSwitchState = currentState;

}

//=======================================================================
//                              Setup
//=======================================================================
void setup()
{
  // Initialize Serial for debug output (optional)
  Serial.begin(115200);
  while (!Serial) { /* wait for serial port to connect */ }

  init_screen();

   
  Serial.println();
  Serial.println(F("Starting SwitchCounterWithTimerInterrupt"));

  // Initialize the pins
  pinMode(SWITCH_PIN, INPUT);
  pinMode(TOGGLE_PIN, OUTPUT);
  
  // Optionally, set an initial state for TOGGLE_PIN
  digitalWrite(TOGGLE_PIN, LOW);

  // Attach the timer interrupt with a 1ms interval
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_US, TimerHandler))
  {
    Serial.println(F("Timer interrupt successfully started."));
  }
  else
  {
    Serial.println(F("Error: Unable to start timer interrupt."));
  }
}

void print_trigger_times() {
 unsigned long now = millis();
 int count = 0;
 unsigned long prevTime = 0;
 int prevIndex = 0;
 unsigned long deltaTime = 0;

 int deltaCount = 0;
 int sumDeltaTimes = 0;
 for(int i=0; i<TRIGGER_BUGGER_LEN; i++) {
    int index = (triggerTimesIndex-1-i + TRIGGER_BUGGER_LEN) % TRIGGER_BUGGER_LEN;

    
    if (now - triggerTimes[index] > 1000) {
      break;  
    }

    count ++;

    Serial.print("i:");
    Serial.print(i);
    Serial.print(" index:");
    Serial.print(index);
    Serial.print(", ");
    Serial.print(now - triggerTimes[index]);

    if (i!=0) {
        deltaTime = triggerTimes[prevIndex] - triggerTimes[index];
        Serial.print(" delta:");
        Serial.print(deltaTime);

        sumDeltaTimes += deltaTime;
        deltaCount ++;
    }
    prevIndex = index;
    prevTime = triggerTimes[index];

    Serial.println();
  }  

  Serial.print("Avg delta:");
  Serial.print(sumDeltaTimes / (deltaCount * 1.0f));

  Serial.print(" Hz:");
  Serial.print(1000 / (sumDeltaTimes / (deltaCount * 1.0f)));

  Serial.print(" Count:");
  Serial.println(count);
  
}


float get_freq_in_interval(int interval) {
 unsigned long now = millis();
 int count = 0;
 unsigned long prevTime = 0;
 int prevIndex = 0;
 unsigned long deltaTime = 0;

 int deltaCount = 0;
 int sumDeltaTimes = 0;
 for(int i=0; i<TRIGGER_BUGGER_LEN; i++) {
    int index = (triggerTimesIndex-1-i + TRIGGER_BUGGER_LEN) % TRIGGER_BUGGER_LEN;
    if (now - triggerTimes[index] > interval) {
      break;  
    }
    count ++;

    if (i!=0) {
        deltaTime = triggerTimes[prevIndex] - triggerTimes[index];
        sumDeltaTimes += deltaTime;
        deltaCount ++;
    }
    prevIndex = index;
    prevTime = triggerTimes[index];
  }  

  if(deltaCount == 0) {
    return 0;  
  }
  
  return 500 / (sumDeltaTimes / (deltaCount * 1.0f));
}

//=======================================================================
//                              Main Loop
//=======================================================================
void loop()
{
  // For demonstration purposes, print the current count every second.
  // Note: The count is updated in the ISR.
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 200)
  {
    print_trigger_times();
    int count = risingEdgeCount; 
    lastPrintTime = millis();

    lcd.setCursor(0, 0);
    lcd.print("Count: ");
    lcd.print(count);

    lcd.setCursor(0, 1);
    lcd.print("Freq: ");
    lcd.print(get_freq_in_interval(2000));
    lcd.print(" Hz   ");
    
    // It's safe to read the volatile variable here (reads might not be atomic but acceptable for demonstration)
    //Serial.print(F("Count = "));
    //Serial.println(count);
  }
}
