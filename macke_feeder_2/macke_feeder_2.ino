// now.pde
// Prints a snapshot of the current date and time along with the UNIX time
// Modified by Andy Wickert from the JeeLabs / Ladyada RTC library examples
// 5/15/11

#include "SoftwareWiremy.h"
#include "DS3231my.h"
#include "AT24C32_EEPROM.h"
#include <TimeLib.h>
#include <DFRobot_RGBLCD1602.h>
#include <Button2.h>

#define LCD_HUE_PURPLE 220
#define LDC_HUE_CYAN   100

#define SDA_PIN  6
#define SCL_PIN  5
#define BUTTON_A_PIN  7
#define BUZZER_PIN    A3
Button2 buttonA;

SoftwareWire mySw(SDA_PIN, SCL_PIN); 
AT24C32_EEPROM eep(mySw, 0x57);
DS3231 ds3231(mySw);

DFRobot_RGBLCD1602 lcd( // 16 characters, 2 lines
    0x2D,  // RGBAddr
    16,    // lcdCols
    2      // lcdRows
);  

uint32_t lastFeedTime = 0;
uint8_t lastFeedHour = 0;
uint8_t lastFeedMin  = 0;
uint32_t nTimesFeeded = 0;

void printSavedData() {
    Serial.print("Last feed time unix:"); Serial.println(lastFeedTime);
    Serial.print("Last feed time hour/min:"); 
    Serial.print(lastFeedHour); Serial.print(":"); Serial.println(lastFeedMin);
    Serial.print("nTimesFeeded:"); Serial.println(nTimesFeeded);
}

void readSavedData() {
    eep.readUint32(0x0050, lastFeedTime);
    eep.readByte(0x0060, lastFeedHour);
    eep.readByte(0x0070, lastFeedMin);
    eep.readUint32(0x0080, nTimesFeeded);

    Serial.println("Loded saved data:");
    printSavedData();
}

void updateSavedData() {
    Serial.println("Saving data:");
    printSavedData();

    eep.writeUint32(0x0050, lastFeedTime);
    eep.writeByte(0x0060, lastFeedHour);
    eep.writeByte(0x0070, lastFeedMin);
    eep.writeUint32(0x0080, nTimesFeeded);
}

void init_buttons() {
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(clickA);
}

void init_screen() {
  lcd.init();
  lcd.noCursor();
  lcd.display();
  lcd.setHue(LDC_HUE_CYAN);
  Serial.print("Setting hue:"); Serial.println(LDC_HUE_CYAN);
}

void setColorLastTime(int32_t timeLastMins) {
  #define HOURS_RED 9 // how long until from the lest time until the screen is blood red
  #define HUE_GREEN 85 // starting hue green value 
  int hue = max(1, HUE_GREEN - timeLastMins*HUE_GREEN / (HOURS_RED*60));
  Serial.print("Setting hue:"); Serial.print(hue);
  Serial.print(" for mins:"); Serial.println(timeLastMins/60);
  lcd.setHue(hue);  
}

String getTimeString() {
  char buf[9]; // "HH:MM:SS" + null
  snprintf(buf, sizeof(buf), "%02u:%02u", hour(), minute());
  return String(buf);
}

String getTimeString2(int hour, int minute, char sperator = ':') {
  char buf[9];
  snprintf(buf, sizeof(buf), "%02u%c%02u", hour, sperator, minute);
  return String(buf);
}


void setup () {
    Serial.begin(115200);
    Serial.println("Program start!");
    pinMode(BUZZER_PIN, OUTPUT);
    init_screen();
    init_buttons();

    if(!ds3231.checkConnection()) {
        Serial.println("Unable to connect to DS3231");
        Serial.println("Reboot to try again.");

        lcd.setHue(LCD_HUE_PURPLE); // purple

        while(1) delay(1);
    } else {
        Serial.println("Connection to DS3231 OK!");
    }

    bool validTime = readTime();
    if(!validTime) {
        Serial.print("Time:"); 
        Serial.println("Invalid time set. Please set it with UART");
    }

    readSavedData();
}

uint32_t lastUpdateBottomRow = 0;
void loop () {
    buttonA.loop();
    readSerial();  
    
    uint32_t nowSec = now();

    static uint32_t lastUpdateTopRow = 0;
    if(nowSec - lastUpdateTopRow >= 1) {
        lastUpdateTopRow = nowSec;
        // update screen each 10 seconds
        //Serial.println(ds3231.getDateStr());

        lcd.setCursor(0, 0);
        lcd.print("Cas: ");
        char sperator = nowSec % 2 == 0? ':' : ' '; 
        lcd.print(getTimeString2(hour(), minute(), sperator));
        lcd.print(" ");
    }

    if(nowSec - lastUpdateBottomRow >= 5) {
        lastUpdateBottomRow = nowSec;       
        uint32_t lastDurrHours = (nowSec - lastFeedTime)/3600;
        uint32_t lastDurrMins = (nowSec - lastFeedTime)/60;
        setColorLastTime(lastDurrMins);
        lcd.setCursor(0, 1);
        lcd.print("Pred: ");
        if (lastDurrHours < 100) lcd.print(lastDurrHours);
        else lcd.print("? ");
        lcd.print("ur ");
        lcd.print(getTimeString2(lastFeedHour, lastFeedMin));
        lcd.print("  ");
    }

    static uint32_t lastUpdateMillis = millis();
    if(millis() - lastUpdateMillis > 1000) {
        lastUpdateMillis = millis();
        Serial.print(getTimeString());
        Serial.println(" ");
    }

}

void beep_success() {
  tone(BUZZER_PIN, 1318);
  delay(100);
  tone(BUZZER_PIN, 1760);
  delay(150);
  tone(BUZZER_PIN, 2217);
  delay(200);
  noTone(BUZZER_PIN);
}

void beep_error() {
  tone(BUZZER_PIN, 987);
  delay(100);
  tone(BUZZER_PIN, 689);
  delay(100);
  noTone(BUZZER_PIN);
}

void clickA(Button2& btn) {
  Serial1.println("Btn A");
  lcd.setCursor(0, 1);
  lcd.print("A clicked");

  if(handleCatFet()) {
    lcd.setCursor(0, 1);
    lcd.print("Nafutran :)     ");  
    setColorLastTime(0);
    beep_success();
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Sej bo se pocu!!");
    lcd.setHue(LCD_HUE_PURPLE);
    beep_error();
  }
  lastUpdateBottomRow = now();
}

bool handleCatFet() {
    uint32_t nowSec = now();

    if(nowSec - lastFeedTime > 60*5) {
        lastFeedTime = nowSec;
        nTimesFeeded ++;
        lastFeedHour = hour(); 
        lastFeedMin  = minute();
        updateSavedData();
        return true;
    } else {
        return false;
    }
}




bool readTime() {
    DateTime dt = RTClib().now(mySw);
    setTime(dt.unixtime());
    Serial.print("Time from DS3231:"); Serial.println(ds3231.getDateStr());
    time_t nowUnix = now();  // Unix timestamp in seconds
    Serial.print("Unix from DS3231:"); Serial.println(nowUnix);

    if(year() < 2025 || year() > 2030) {
        return false;
    }
    return true;
}


static char lineBuf[256];
static uint8_t lineLen = 0;
void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;               // ignore CR
    if (c == '\n') {                       // got a line
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) {
        handleLine(lineBuf);
      }
      lineLen = 0;
    } else if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = c;
    } else {
      // overflow -> reset
      lineLen = 0;
    }
  }
}

static bool isDigitStr(const char* s) {
  if (!s || !*s) return false;
  while (*s) { if (*s < '0' || *s > '9') return false; ++s; }
  return true;
}

// ---- Process incoming line ----
static void handleLine(const char* s) {
  // Trim spaces
  while (*s == ' ' || *s == '\t') ++s;

  Serial.print("Command:");
  Serial.println(s[0]);

  // Case 1: pure digits -> interpret as Unix seconds (UTC)
  if (isDigitStr(s)) {
    unsigned long sec32 = strtoul(s, NULL, 10);
    time_t epoch = (time_t)sec32;
    setTime(epoch);
    // flag_localtime=false -> gmtime_r() inside setEpoch
    ds3231.setEpoch(epoch, false);
    Serial.print(F("Set DS3231 from Unix(UTC): "));
    Serial.println((unsigned long)epoch);

    Serial.print("Ds3231 new date:");
    Serial.println(ds3231.getDateStr());
  } else if (s[0] == 'c') {
    Serial.println("Got clear memory command.");
    lastFeedTime = 0;
    lastFeedHour = 0;
    lastFeedMin  = 0;
    nTimesFeeded = 0;
    updateSavedData();
  } else {
    Serial.println(F("Unrecognized format. Use Unix seconds."));
  }
}

