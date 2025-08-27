// now.pde
// Prints a snapshot of the current date and time along with the UNIX time
// Modified by Andy Wickert from the JeeLabs / Ladyada RTC library examples
// 5/15/11

#include <DFRobot_RGBLCD1602.h>
#include <Button2.h>

#define LCD_HUE_PURPLE 220
#define LDC_HUE_CYAN   100

#define SDA_PIN  6
#define SCL_PIN  5
#define BUTTON_A_PIN  7
#define BUZZER_PIN    A3
Button2 buttonA;


DFRobot_RGBLCD1602 lcd( // 16 characters, 2 lines
    0x2D,  // RGBAddr
    16,    // lcdCols
    2      // lcdRows
);  

void init_buttons() {
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(clickA);
}

void init_screen() {
  lcd.init();
  lcd.noCursor();
  lcd.display();
  lcd.setHue(LDC_HUE_CYAN);
  Serial.print("Setting hue:"); Serial.print(LDC_HUE_CYAN);
}

void setColorLastTime(int32_t timeLastMins) {
  #define HOURS_RED 9 // how long until from the lest time until the screen is blood red
  #define HUE_GREEN 86 // starting hue green value 
  int hue = max(1, HUE_GREEN - timeLastMins*HUE_GREEN / (HOURS_RED*60));
  Serial.print("Setting hue:"); Serial.print(hue);
  Serial.print(" for mins:"); Serial.println(timeLastMins/60);
  lcd.setHue(hue);  
}


void setup () {
    Serial.begin(115200);
    Serial.println("Program start!");
    pinMode(BUZZER_PIN, OUTPUT);
    init_screen();
    init_buttons();
}

int lastMinute = 0;
void loop () {
    buttonA.loop();
  
    static uint32_t lastUpdateMillis = millis();
    if(millis() - lastUpdateMillis > 100) {
      lastUpdateMillis = millis();
      lastMinute += 10;
      setColorLastTime(lastMinute);
    }

}

void clickA(Button2& btn) {
  Serial1.println("Btn A");
  lastMinute = 0;
}




