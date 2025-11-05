#include "esp_log.h" 
#include <SoftwareSerial.h>
#include "Button2.h"
#include "esp_timer.h"
#include <SerialMod0.h>
#include "AS5600.h"
#include <Wire.h>
#include <TimeLib.h>
#include <Preferences.h>
#include "ErrorLogger.h"
#include "ResetDiagnostics.h"
#include "MyPrefs.h"
#include <string.h>  // for memcpy

typedef struct {
    uint16_t avg;
    //uint16_t max; // we dont need to log max speed
    int16_t dir;      // from 0 - 360, negative values for errors 
    //uint16_t ts;   // we only log last and first log instead of logging every timestamp
} WindSample;

struct AppPrefs {
  uint16_t pref_version;              // just an intiger to difirentiata between different versions 
  char     version[8];                // program/sw version
  char     url_to_send[70];           // url that is used to send the data to 

  uint8_t  light_sleep_enabled;       // light sleep between reads, 0 if disabled, and 1 if enabled
  uint8_t  sleep_enabled;             // 0 if disabled, and 1 if enabled
  int8_t   sleep_hour_start;          // which hour the device gets to sleep in 24 hour format 
  int8_t   sleep_hour_end;            // which hour the device is expected to wake up again

  uint8_t  store_wind_data_interval;  // how ofthen the values (speed and dir) are saved 
  uint8_t  send_data_interval;        // in minutes, how often do we send the data 

  uint8_t  error_led_on_time;    // in ms, how log the error led stays on when blinking 
  uint8_t  dir_led_on_time;      // in ms, how log the direc led stays on when blinking 
  uint8_t  spin_led_on_time;     // in ms, how log the spin  led stays on when blinking 
  uint8_t  blink_led_on_time;    // in ms, how log the blink led stays on when blinking 
  uint8_t  blink_led_interval;   // deca seconds 10 for 1 seconds, hof often the blink led blinks

  uint8_t  as5600_pwr_on_time;   // ms of how long to wait after power on before reading it, It is also an interupt cycle
  uint8_t  as5600_read_interval; // deca seconds 10 for 1 seconds, // how ofter we want to read direction
};

#define SCHEMA_VERSION  11

// define default preferences:
AppPrefs prefs = {
  /*pref_version*/              2,
  /*version*/                   "v2",
  /*url_to_send*/               "http://46.224.24.144/veter/save/",

  /*light_sleep_enabled*/       0,
  /*sleep_enabled*/             0,
  /*sleep_hour_start*/          20,
  /*sleep_hour_end*/            6,

  /*store_wind_data_interval*/  5,
  /*send_data_interval*/        10,

  /*error_led_on_time*/         10,
  /*dir_led_on_time*/           10,
  /*spin_led_on_time*/          20,
  /*blink_led_on_time*/         20,
  /*blink_led_interval*/        20,  

  /*as5600_pwr_on_time*/        30,
  /*as5600_read_interval*/      30
};

enum class SendResult : int {
  OK                   =  1,   // success
  AT_FAIL              = -1,   // "AT" didn't respond
  NO_SIM               = -2,   // CSMINS? => no SIM
  CSQ_FAIL             = -3,   // CSQ failed
  REG_FAIL             = -4,   // CREG? or CGREG? failed (both map here)
  CIMI_FAIL            = -5,   // CIMI failed
  GPRS_SETUP_FAIL      = -6,   // any SAPBR step failed
  HTTP_FAIL            = -7,   // sendPOST / HTTPREAD failed
};

//#define PRINT_SIM_COMM 
//#define PRINT_MAGNET_READ_DEBUG

#define UART_DEBUG_TX  21
#define UART_DEBUG_RX  17

SerialMod0 SerialUart0(UART_NUM_0, UART_DEBUG_TX, UART_DEBUG_RX); 
#define SerialDBG SerialUart0

//#define SerialDBG Serial

#define Serial_print(x)    do { SerialDBG.print(x); /* Serial1.print(x);*/ } while (0)
#define Serial_println(x)  do { SerialDBG.println(x); /* Serial1.println(x);*/ } while (0)
#define Serial_write(x)  do { SerialDBG.write(x); /*Serial1.write(x);*/ } while (0)

#define RX_PIN 37
#define TX_PIN 39
//SoftwareSerial SerialAT(TX_PIN, RX_PIN);  // SIM800L <-> Arduino
#define SerialAT Serial1

#define BUTTON_PIN       9
#define BUTTON_PIN_2     11
#define GPRS_ON_PIN      18   // MOS FET turn on pin
#define GPRS_POWER_PIN   35   // PWX pin on the SIM800C board
#define V_BATT_PIN       13
#define V_SOALR_PIN      8

#define BLINK_LED_PIN      15
//#define BLINK_LED_ON_TIME  20    
//#define BLINK_LED_INTERVAL 2000  

#define SPIN_LED_PIN       3
//#define SPIN_LED_ON_TIME   20   

#define DIR_LED_PIN        2
//#define DIR_LED_ON_TIME    10

#define ERROR_LED_PIN      1
//#define ERROR_LED_ON_TIME  10

//#define VANE_POWER_PIN   1
#define HAL_SENSOR_PIN     12

//#define STORE_WIND_DATA_INTERVAL 5 // save data each X seconds 



//#define TIMER_PIN 14

/*
TODO imporvments:

Error codes logging and sending
#DONE Change Sending IP 
Remove sending of max speed just send speed 





configurability
Presistant sessions



*/

/*

Single battery powered 
3.2 volts, no extra capacitors
Runs ok, reset on GPRS power ON or on attempting to connect 

Single battery powered 
3.2 volts, 4.7uF on gprs, No on ESP
Reset on power GPRS on

Single battery powered 
3.15 volts, 4.7uF on gprs and ESP: Send succesful!!
3.14: Send succesful:
3.10: Success! 

Single battery powered 
3.10 volts, 4.7uF and ESP: Success!

Single battery powered 
3.08 volts, 1uF and ESP: Fail on send, Fail on send 
3.3 V; Success 
3,24: ok 

3,84 v: orange battery: Fail on connect
3,8 V orange bat2: Fail on connect

3.22V fast discharging vape batt: Fail on connect 
3,48: ok maybe a bit wierd behaviour
3,45 ok
3,43 ok
3,33 ok
3,23 ok, fail on conn, fail   


*/
// Define a custom TwoWire instance
#define SDA_GPIO             5
#define SCL_GPIO             7      
#define AS600_POWER_PIN      4
AS5600 as5600; 

#define ANALOG_PIN 123

Button2 button;
Button2 button2;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastNow = 0;

void IRAM_ATTR onReadDirection(void* arg);
void IRAM_ATTR onReadHal(void* arg);
void IRAM_ATTR onReadSpeed(void* arg);
void IRAM_ATTR onBlinkLed(void* arg);
void IRAM_ATTR onSpinLed(void* arg);
void IRAM_ATTR onDirLed(void* arg);
void IRAM_ATTR onErrorLed(void* arg);
void IRAM_ATTR onStoreWindData(void* arg);

void tap(Button2& btn);
void tap2(Button2& btn);

#define DEEP_SLEEP_DURATION  3600ULL * 1000000 // value in microseconds so: one hour
//#define DEEP_SLEEP_DURATION  10 * 1000 * 1000  // 10 seconds
RTC_DATA_ATTR time_t timeBeforeSleep = 0;      // stores last time before deep sleep

void printVersionAndCompileDate() {
  Serial_print("Compiled on ");
  Serial_print(__DATE__);
  Serial_print(" at ");
  Serial_println(__TIME__);
}


void printAS5600Config() {

  //portENTER_CRITICAL(&timerMux);

  digitalWrite(AS600_POWER_PIN, HIGH);  
  delay(10);

  AS5600 as = as5600;
  Serial_println(F("===== AS5600 CONFIGURATION ====="));

  Serial_print(F("I2C Address: ")); Serial_println(as.getAddress());
  Serial_print(F("Direction: ")); Serial_println(as.getDirection() == AS5600_CLOCK_WISE ? "Clockwise" : "Counter-Clockwise");
  Serial_print(F("Power Mode: ")); Serial_println(as.getPowerMode());
  Serial_print(F("Hysteresis: ")); Serial_println(as.getHysteresis());

  Serial_print(F("Output Mode: "));
  switch (as.getOutputMode())
  {
    case AS5600_OUTMODE_ANALOG_100: Serial_println(F("Analog 0–100%")); break;
    case AS5600_OUTMODE_ANALOG_90:  Serial_println(F("Analog 10–90%")); break;
    case AS5600_OUTMODE_PWM:        Serial_println(F("PWM")); break;
    default: Serial_println(F("Unknown")); break;
  }

  Serial_print(F("PWM Frequency: "));
  switch (as.getPWMFrequency())
  {
    case AS5600_PWM_115: Serial_println(F("115 Hz")); break;
    case AS5600_PWM_230: Serial_println(F("230 Hz")); break;
    case AS5600_PWM_460: Serial_println(F("460 Hz")); break;
    case AS5600_PWM_920: Serial_println(F("920 Hz")); break;
    default: Serial_println(F("Unknown")); break;
  }

  Serial_print(F("Slow Filter: ")); Serial_println(as.getSlowFilter());
  Serial_print(F("Fast Filter: ")); Serial_println(as.getFastFilter());
  Serial_print(F("Watchdog: ")); Serial_println(as.getWatchDog() ? "ON" : "OFF");
  Serial_print(F("Z Position: ")); Serial_println(as.getZPosition());
  Serial_print(F("M Position: ")); Serial_println(as.getMPosition());
  Serial_print(F("Max Angle: ")); Serial_println(as.getMaxAngle());
  Serial_print(F("Configure Register: ")); Serial_println(as.getConfigure());
  Serial_print(F("Offset (degrees): ")); Serial_println(as.getOffset());
  Serial_print(F("Last Error: ")); Serial_println(as.lastError());

  Serial_println(F("================================"));

  Serial_print(F("Angle: ")); Serial_println(as.readAngle());
  Serial_println();

  digitalWrite(AS600_POWER_PIN, LOW);  
  //portEXIT_CRITICAL(&timerMux);
}

void welcomTurnOnBlink() {
  int dur = 10;
  digitalWrite(SPIN_LED_PIN, HIGH);
  delay(5*dur);
  digitalWrite(DIR_LED_PIN, HIGH);
  delay(5*dur);
  digitalWrite(ERROR_LED_PIN, HIGH);       
  delay(10*dur);  
  digitalWrite(SPIN_LED_PIN, LOW);
  digitalWrite(DIR_LED_PIN, LOW);     
  digitalWrite(ERROR_LED_PIN, LOW);       
  delay(10*dur);  
  digitalWrite(SPIN_LED_PIN, HIGH);
  digitalWrite(DIR_LED_PIN, HIGH);     
  digitalWrite(ERROR_LED_PIN, HIGH); 
  delay(20*dur);   
  digitalWrite(ERROR_LED_PIN, LOW);
  digitalWrite(DIR_LED_PIN, LOW);   
  digitalWrite(SPIN_LED_PIN, LOW); 
}


// Pretty print (for Serial debug)
static void printResetInfo(const ResetInfo& i) {
  Serial_print("[Reset] reason=");
  Serial_print(resetReasonToStr(i.reason));
  Serial_print(" (");
  Serial_print((int)i.reason);
  Serial_println(")");

  if (i.from_deep_sleep) {
    Serial_print("[Reset] wakeup=");
    Serial_print(wakeupCauseToStr(i.wakeup));
    Serial_print(" (");
    Serial_print((int)i.wakeup);
    Serial_println(")");
  }

  Serial_print("[Reset] unexpected=");
  Serial_println(i.unexpected ? "YES" : "no");
}

ErrorLogger elog;

PrefBlob<AppPrefs> store(SCHEMA_VERSION);

void loadPreferences() {
  store.begin();

  AppPrefs prefsLoaded;
  int loaded = store.load(prefsLoaded);
  Serial_print("Prefs loaded status: "); Serial_println(loaded);

  if (loaded < 0) {
    Serial_println("Error loading ... saving defulat preferences");

    int save = store.save(prefs);
    Serial_print("Save status: "); Serial_println(save);
    if (save < 0) {
      Serial_println("Save failed");
    }
  } else {
    Serial_println("Prefs loaded ok! Using that"); 
    prefs = prefsLoaded;
  }


  printPreferences();

  store.end();
}

void savePreferences() {
  store.begin();
  Serial_println("Saving preferences");

  int save = store.save(prefs);
  Serial_print("Save status: "); Serial_println(save);
  if (save < 0) {
    Serial_println("Save failed");
  }

  printPreferences();
  store.end();
}

void printPreferences() {
  Serial_println("---- Preferences ----");
  Serial_print("  pref_version: "); Serial_println(prefs.pref_version);
  Serial_print("  version: ");      Serial_println(prefs.version);
  Serial_print("  url_to_send: ");  Serial_println(prefs.url_to_send);
  Serial_println();

  Serial_print("  light_sleep_enabled: ");  Serial_println(prefs.light_sleep_enabled);
  Serial_print("  sleep_enabled: ");  Serial_println(prefs.sleep_enabled);
  Serial_print("  sleep_hour_start: ");  Serial_println(prefs.sleep_hour_start);
  Serial_print("  sleep_hour_end: ");    Serial_println(prefs.sleep_hour_end);
  Serial_println();

  Serial_print("  store_wind_data_interval: "); Serial_println(prefs.store_wind_data_interval);
  Serial_print("  send_data_interval: ");       Serial_println(prefs.send_data_interval);
  Serial_println();
  
  Serial_print("  error_led_on_time: ");   Serial_println(prefs.error_led_on_time);
  Serial_print("  dir_led_on_time: ");     Serial_println(prefs.dir_led_on_time);
  Serial_print("  spin_led_on_time: ");    Serial_println(prefs.spin_led_on_time);
  Serial_print("  blink_led_on_time: ");   Serial_println(prefs.blink_led_on_time);
  Serial_print("  blink_led_interval: ");  Serial_println(prefs.blink_led_interval);
  Serial_println();

  Serial_print("  as5600_pwr_on_time: ");   Serial_println(prefs.as5600_pwr_on_time);
  Serial_print("  as5600_read_interval: "); Serial_println(prefs.as5600_read_interval);
  Serial_println("---------------------");
}


String version = "2.0";

void setup() {
  //Serial.begin(115200);
  setTime(15, 0, 0, 29, 9, 2025);

  SerialDBG.begin(115200);

  Serial_println();
  Serial_println("### PROGRAM START! ###");
  Serial_print("Compiled on "); Serial_println(__DATE__ " " __TIME__);
  Serial_print("Version:"); Serial_println(prefs.version);

  loadPreferences();

  esp_log_level_set("i2c.master", ESP_LOG_NONE); 

  ResetInfo ri = readResetInfo();
  printResetInfo(ri);

  switch (ri.reason) {
    case ESP_RST_BROWNOUT:       elog.log(ErrorLogger::ERR_BROWNOUT_RESET); break;
    case ESP_RST_PANIC:          elog.log(ErrorLogger::ERR_PANIC_RESET); break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:            elog.log(ErrorLogger::ERR_WDT_RESET); break;
    case ESP_RST_DEEPSLEEP:
    case ESP_RST_POWERON:
    case ESP_RST_SW:             /* considered expected here */ break;
    default:                     elog.log(ErrorLogger::ERR_UNEXPECTED_RESET); break;
  }


  if(timeBeforeSleep == 0) {
    Serial_println("Fresh start. Normal boot");
  } else {
    // timeWas set before sleep so we can check if is time to wake up already!
    setTime(timeBeforeSleep + DEEP_SLEEP_DURATION / 1000000);
    Serial_print("Woke up from deep sleep. Current time:"); Serial_println(getFormattedTimeString());
    evaluateIfDeepSleep();
  }

  SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // 33, 34);

  pinMode(GPRS_ON_PIN, INPUT); // off
  pinMode(GPRS_POWER_PIN, OUTPUT); digitalWrite(GPRS_POWER_PIN, HIGH); // off
  pinMode(BLINK_LED_PIN, OUTPUT);   digitalWrite(BLINK_LED_PIN, HIGH); // on
  pinMode(SPIN_LED_PIN, OUTPUT);   digitalWrite(SPIN_LED_PIN, LOW);    // off
  pinMode(DIR_LED_PIN, OUTPUT);    digitalWrite(DIR_LED_PIN, LOW);          // off
  pinMode(ERROR_LED_PIN, OUTPUT);        digitalWrite(ERROR_LED_PIN, LOW);         // off
  //pinMode(TIMER_PIN, OUTPUT); digitalWrite(TIMER_PIN, LOW);
  //pinMode(VANE_POWER_PIN, OUTPUT); digitalWrite(VANE_POWER_PIN, HIGH);  
  //gpio_set_drive_capability((gpio_num_t) VANE_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(AS600_POWER_PIN, OUTPUT); digitalWrite(AS600_POWER_PIN, LOW);  
  gpio_set_drive_capability((gpio_num_t) AS600_POWER_PIN, GPIO_DRIVE_CAP_3);


  pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);

  button.begin(BUTTON_PIN);
  button.setPressedHandler(tap);

  button2.begin(BUTTON_PIN_2);
  button2.setPressedHandler(tap2);

  // init the as5600 chip so we can read wind direction 
  Wire.begin(SDA_GPIO, SCL_GPIO); // 1 Mhz
  Wire.setClock(1000000UL);

  setCpuFrequencyMhz(80);

  welcomTurnOnBlink();

  const esp_timer_create_args_t hal_timer_args = {
    .callback = &onReadHal,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "hal_timer"
  };
  esp_timer_handle_t hal_timer;
  esp_timer_create(&hal_timer_args, &hal_timer);
  esp_timer_start_periodic(hal_timer, 4*1000);  // every 4 ms

  esp_timer_handle_t blinkLed_timer;
  const esp_timer_create_args_t blinkLed_args = {
    .callback = &onBlinkLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "blinkLed_timer"
  };
  esp_timer_create(&blinkLed_args, &blinkLed_timer);
  esp_timer_start_periodic(blinkLed_timer, prefs.blink_led_on_time * 1000); 

  esp_timer_handle_t spinLed_timer;
  const esp_timer_create_args_t spinLed_args = {
    .callback = &onSpinLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "spinLed_timer"
  };
  esp_timer_create(&spinLed_args, &spinLed_timer);
  esp_timer_start_periodic(spinLed_timer, prefs.spin_led_on_time * 1000); 

  esp_timer_handle_t dirLed_timer;
  const esp_timer_create_args_t dirLed_args = {
    .callback = &onDirLed,                
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "dirLed_timer"
  };
  esp_timer_create(&dirLed_args, &dirLed_timer);
  esp_timer_start_periodic(dirLed_timer, prefs.dir_led_on_time * 1000);  

  esp_timer_handle_t errorLed_timer;
  const esp_timer_create_args_t errorLed_args = {
    .callback = &onErrorLed,               
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "errorLed_timer"
  };
  esp_timer_create(&errorLed_args, &errorLed_timer);
  esp_timer_start_periodic(errorLed_timer, prefs.error_led_on_time * 1000); 

  esp_timer_handle_t readSpeed_timer;
  const esp_timer_create_args_t readSpeed_args = {
    .callback = &onReadSpeed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readSpeed_timer"
  };
  esp_timer_create(&readSpeed_args, &readSpeed_timer);
  esp_timer_start_periodic(readSpeed_timer, 1000 * 1000); // 1s

  esp_timer_handle_t readDirection_timer;
  const esp_timer_create_args_t readDirection_args = {
    .callback = &onReadDirection,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readDirection_timer"
  };
  esp_timer_create(&readDirection_args, &readDirection_timer);
  esp_timer_start_periodic(readDirection_timer, prefs.as5600_pwr_on_time*1000); 

  esp_timer_handle_t storeWindData_timer;
  const esp_timer_create_args_t storeWindData_args = {
      .callback = &onStoreWindData,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "storeWindData_timer"
  };
  esp_timer_create(&storeWindData_args, &storeWindData_timer);
  esp_timer_start_periodic(storeWindData_timer, prefs.store_wind_data_interval*1000*1000); // X seconds (in microseconds)

  Serial_println("Done init!");   
}

uint32_t get_log_timestamp(int hour, int minute, int second) {
  //return hour*60*30 + minute*30 + second/2;  // round on every 2 seconds so that we can store it inside 16 bit int. Max:43200 < 2**16
  return hour*60*60 + minute*60 + second; // timestamp in seconds
}

uint32_t get_log_timestamp() {
  return get_log_timestamp(hour(), minute(), second());
}


#ifdef PRINT_MAGNET_READ_DEBUG
  #define DBG_MNG(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_MNG(...) do {} while (0)
#endif


void IRAM_ATTR onBlinkLed(void* arg) {
  static int nOnBlinkLedcalls = 0;
  nOnBlinkLedcalls ++;

  uint16_t toggleCount = prefs.blink_led_interval*100 / prefs.blink_led_on_time;
  if(nOnBlinkLedcalls % toggleCount == 0) {
    digitalWrite(BLINK_LED_PIN, HIGH);    
    //Serial_print("on");
  } else {
    digitalWrite(BLINK_LED_PIN, LOW); 
  }
}

volatile int rotation_detected_blink = 0;
void IRAM_ATTR onSpinLed(void* arg) {
  if(rotation_detected_blink == 1) {
    rotation_detected_blink = 0;
    digitalWrite(SPIN_LED_PIN, HIGH);    
    //Serial_print("on");
  } else {
    digitalWrite(SPIN_LED_PIN, LOW); 
  }
}

volatile int direction_detected_north = 0;
void IRAM_ATTR onDirLed(void* arg) {
  if(direction_detected_north == 1) {
    direction_detected_north = 0;
    digitalWrite(DIR_LED_PIN, HIGH);    
  } else {
    digitalWrite(DIR_LED_PIN, LOW); 
  }
}

volatile int error_notify_led = 0;
void IRAM_ATTR onErrorLed(void* arg) {
  if(error_notify_led == 1) {
    error_notify_led = 0;
    digitalWrite(ERROR_LED_PIN, HIGH);    
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); 
  }
}


#define DIRECTIONS_LOG_LEN 10  // buffer to store diractions before the are averaged and saved in an array to be send 
int last_direction_read = -1;
int directions_log[DIRECTIONS_LOG_LEN];
volatile int directions_log_i = 0;


  #define AS5600_PWR_ON_TIME   30 
#define AS5600_READ_INTERVAL 3000 // ms how ofter we want to read direction
#define AS5600_IR_CYCLES     AS5600_READ_INTERVAL / AS5600_PWR_ON_TIME  // Interupt cycles before powering on and reading the sensor again 



void IRAM_ATTR onReadDirection(void* arg) {
  static int directionReadCount = 0; 
  int angle = 0;
  int8_t as5600_error = 0;
  #define MAGNET_READ_REPEATS 4
  int8_t readRepeats;

  uint16_t ir_cycles = (prefs.as5600_read_interval*100 / prefs.as5600_pwr_on_time);

  //Serial.flush();
  if(directionReadCount == 1) {
    digitalWrite(AS600_POWER_PIN, HIGH);
  }

  else if(directionReadCount == 2) {
    //#ifdef PRINT_MAGNET_READ_DEBUG
    //Serial_print("Adress: "); Serial_println(as5600.getAddress());
    //Serial_print("Is connectet: "); Serial_println(as5600.isConnected());
    //Serial_print("Magnet magnitude: "); Serial_println(as5600.readMagnitude());
    //Serial_print("Detect magnet: "); Serial_println(as5600.detectMagnet());
    //Serial_print("magnetTooStrong: "); Serial_println(as5600.magnetTooStrong());
    //Serial_print("magnetTooWeak: "); Serial_println(as5600.magnetTooWeak());
    //#endif
    
    readRepeats = MAGNET_READ_REPEATS;
    do {
      angle = as5600.readAngle()*360 / 4096;
      as5600_error = as5600.lastError();

      //if(as5600_error != 0) elog.log(ErrorLogger::ERR_DIR_READ_ONCE);
      //DBG_MNG( Serial_print("Read angle: "); Serial_print(angle); Serial_print(" "); Serial_println(as5600_error); );
    } while(as5600_error != 0 && --readRepeats > 0);

    
    if(as5600_error == 0) {
      // successful angle read
      last_direction_read = angle;
      DBG_MNG( Serial_print("Read angle: "); Serial_println(angle); );

      // what should be considered "facing north", how much can the angle diviate from north
      #define BLINK_MARGIN  20
      if(BLINK_MARGIN > angle || angle > 360-BLINK_MARGIN) direction_detected_north = 1; // blink north direction led
      
      portENTER_CRITICAL_ISR(&timerMux);
      if(directions_log_i < DIRECTIONS_LOG_LEN) {
        // save the measurement inside the log
        directions_log[directions_log_i++] = angle;
      } else {
        elog.log(ErrorLogger::ERR_DIR_SHORT_BUF_FULL);
      }
      portEXIT_CRITICAL_ISR(&timerMux);

    } else {
      last_direction_read = as5600_error; // no angle was succesfully read due to an error
      Serial_print("Read angle error: "); {
        Serial_println(as5600_error);
        error_notify_led = 1;
      }
      elog.log(ErrorLogger::ERR_DIR_READ);
    }
    
    digitalWrite(AS600_POWER_PIN, LOW);
  }

  // reset the directionReadCount when we hit the ir_cycles 
  else if(directionReadCount == ir_cycles) {
    directionReadCount = 0;
  }

  directionReadCount++;
}


//#define PRINT_SPEED
//#define PRINT_ON_STORE_WIND

volatile float rps = 0;
volatile uint32_t rotationCount = 0;
volatile int lastHalSensorRead = -1;
volatile uint32_t lastDetection = 0; // we need to store the time of last detection to calculate rotations per second
void IRAM_ATTR onReadHal(void* arg) {
  // Example logic: latch HAL sensor if triggered
  int halSensorRead = digitalRead(HAL_SENSOR_PIN);
  if (halSensorRead != lastHalSensorRead && halSensorRead == LOW) {
    uint32_t now = micros() / 1000;

    portENTER_CRITICAL_ISR(&timerMux);
    rotationCount ++;                    // number of rotations counted, used to average rps every second in a diferent interupt
    rotation_detected_blink = 1;

    #ifdef PRINT_SPEED 
      Serial_print("|");
    #endif
    rps += 1000.0f / (now - lastDetection);
    portEXIT_CRITICAL_ISR(&timerMux);
    lastDetection = now; 
  }
  lastHalSensorRead = halSensorRead;
}


#define SPEEDS_LOG_LEN 60 
uint16_t speeds_log[SPEEDS_LOG_LEN]; // speed logged each second for a short interval
volatile int speeds_log_i = 0;

// read speed every second
void IRAM_ATTR onReadSpeed(void* arg) {
  portENTER_CRITICAL_ISR(&timerMux);

  float speed =  rotationCount == 0? 0 : rps / rotationCount;
  #ifdef PRINT_SPEED 
    Serial_print("Speed: "); Serial_print(speed * 10); 
    Serial_print(", cnt:"); Serial_println(rotationCount);
  #endif
  rotationCount = 0; 
  rps = 0;

  if (speeds_log_i < SPEEDS_LOG_LEN){
    speeds_log[speeds_log_i++] = int(speed * 10);
  } else {
    elog.log(ErrorLogger::ERR_WIND_SHORT_BUF_FULL);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}



//#define STORE_WIND_N_HOURS 2
// calculate how much store lenght do we need to save N hours of data  if we store every interval second
//#define WIND_LOG_STORE_LEN ((STORE_WIND_N_HOURS* 60*60) / STORE_WIND_DATA_INTERVAL) 
#define WIND_LOG_STORE_LEN 1000 // no reason to store more then 1000 data as it cant properly send all of it 

// ---- Data record ------------------------------------------------------------
uint32_t wind_data_start_time = 0;

// ---- Storage (ring buffer) --------------------------------------------------
static WindSample wind_log[WIND_LOG_STORE_LEN];

// Head points to next write position; count is number of valid items (<= LEN)
static volatile uint16_t w_head = 0;
static volatile uint16_t w_count = 0;
static volatile uint32_t first_timestamp = 0;
static volatile uint32_t last_timestamp = 0;

// ---- Helpers ----------------------------------------------------------------
uint16_t windlog_len(void) {
    return (uint16_t) w_count;
}

// Oldest item index in the circular buffer (physical index within wind_log[])
uint16_t windlog_oldest_index(void) {
    if (w_count == 0) return 0;
    // (head - count) modulo LEN
    uint16_t oldest = (uint16_t)((w_head + WIND_LOG_STORE_LEN - w_count) % WIND_LOG_STORE_LEN);
    return oldest;
}

// ---- Public API ------------------------------------------------------------

// Store one record (overwrites the oldest when full)
void windlog_push(uint16_t avg, int16_t dir, uint32_t ts) {
    wind_log[w_head].avg  = avg;
    //wind_log[w_head].max  = max;
    wind_log[w_head].dir  = dir;
    //wind_log[w_head].ts = ts;

    if(w_count == 0) first_timestamp = ts;
    last_timestamp = ts;
    
    w_head = (uint16_t)((w_head + 1) % WIND_LOG_STORE_LEN);
    if (w_count < WIND_LOG_STORE_LEN) {
        w_count++;
    } else{
      first_timestamp += prefs.store_wind_data_interval; // just increase the fist timestamp by the expected interval that timestamps increase (by interval )
      // buffer full -> oldest is implicitly dropped
      elog.log(ErrorLogger::ERR_WIND_BUF_OVERWRITE);
    }
}

int windlog_copy(WindSample* out, uint16_t max_out) {
  if (!out || max_out == 0) return 0;

  uint16_t copied = 0;
  uint16_t size   = w_count;
  uint16_t to_copy = (size < max_out) ? size : max_out;

  if (to_copy > 0) {
      uint16_t base = windlog_oldest_index();

      // The logical span may wrap; split into two memcpy if needed
      uint16_t first_run = (uint16_t)((base + to_copy <= WIND_LOG_STORE_LEN) ? to_copy
                                                                              : (WIND_LOG_STORE_LEN - base));
      memcpy(out, &wind_log[base], first_run * sizeof(WindSample));

      uint16_t remaining = (uint16_t)(to_copy - first_run);
      if (remaining) {
          memcpy(out + first_run, &wind_log[0], remaining * sizeof(WindSample));
      }
      copied = to_copy;
  }

  return copied;
}

// Optional: clear buffer
inline void windlog_clear(void) {
    w_head = 0;
    w_count = 0;
}

float average_direction(const int* directions_log, size_t len) {
  float sum_sin = 0.0;
  float sum_cos = 0.0;

  for (size_t i = 0; i < len; i++) {
    float radians = directions_log[i] * DEG_TO_RAD;
    sum_cos += cos(radians);
    sum_sin += sin(radians);
  }

  float avg_angle = atan2(sum_sin, sum_cos) * RAD_TO_DEG;
  if (avg_angle < 0) avg_angle += 360.0;  // Normalize to [0, 360)
  return avg_angle;
}



void IRAM_ATTR onStoreWindData(void* arg) {
  portENTER_CRITICAL(&timerMux);
  // store the miliseconds of when the first data got stored 
  if(wind_data_start_time == 0) wind_data_start_time = lastNow;

  // the log is full so we calculate average and save the measurement into the avg log
  //uint16_t maxSpeed = 0;
  int avgSpeedSum = 0;
  for(int i=0; i<speeds_log_i; i++) {
    //maxSpeed = max(maxSpeed, speeds_log[i]);
    avgSpeedSum += speeds_log[i]; 
  }

  int avgSpeed = avgSpeedSum / speeds_log_i;
  speeds_log_i = 0;


  int16_t avgDir = average_direction(directions_log, directions_log_i);
  directions_log_i = 0;
  
  uint32_t timestamp = get_log_timestamp();;

  //windlog_push(avgSpeed, maxSpeed, avgDir, timestamp);
  windlog_push(avgSpeed, avgDir, timestamp);
  portEXIT_CRITICAL_ISR(&timerMux);

  #ifdef PRINT_ON_STORE_WIND 
    Serial_print("Updated wind data, avg:"); Serial_print(avgSpeed);
    Serial_print(", max:"); Serial_print(maxSpeed);
    Serial_print(", dir:"); Serial_print(avgDir);
    Serial_println();
  #endif
}

WindSample wind_log_copy[WIND_LOG_STORE_LEN];
String getWindData() {
  portENTER_CRITICAL_ISR(&timerMux);
  uint16_t wind_log_copy_len = windlog_copy(wind_log_copy, WIND_LOG_STORE_LEN);
  portEXIT_CRITICAL_ISR(&timerMux);

  //int speed_avg_i_on_send = 0;
  //speed_avg_i_on_send = speed_avg_i; // we save the index on when we send the data so we can see if there is any new data when we restart the index after successful send 

  if(wind_log_copy_len == 0) {
    return "len=0;avg=;dir=;logFirst=;logLast=;"; 
  }

  String windData = "len=" + String(wind_log_copy_len);

  windData += ";avg=" + String(wind_log_copy[0].avg);
  for(int i=1; i<wind_log_copy_len; i++) {
    windData += ",";
    windData += String(wind_log_copy[i].avg);
  }

  /*
  windData += ";max=" + String(wind_log_copy[0].max);
  for(int i=1; i<wind_log_copy_len; i++) {
    windData += ",";
    windData += String(wind_log_copy[i].max);
  }
  */

  windData += ";dir=" + String(wind_log_copy[0].dir);
  for(int i=1; i<wind_log_copy_len; i++) {
    windData += ",";
    windData += String(wind_log_copy[i].dir);
  }

  windData += ";logFirst=" + String(first_timestamp);
  windData += ";logLast=" + String(last_timestamp);
  return windData + ";"; 
}


bool isDeepSleepTime() {
  if(!prefs.sleep_enabled) return false; // dont do anything if it is disabled 

  if(timeStatus() == timeNotSet) return false; // how can we sleep if we dont know what the time is!

  // we sleep at night ofcorse! from 8 PM to 6 AM
  if(prefs.sleep_hour_start <= hour() && hour() <= 24) return true;
  if(0 <= hour() && hour() < prefs.sleep_hour_end) return true;

  return false;
}

void evaluateIfDeepSleep() {
  if(isDeepSleepTime()) {
    Serial_print("Current time is:"); Serial_println(getFormattedTimeString());
    Serial_print("It is time to go deep sleep for: "); Serial_print(DEEP_SLEEP_DURATION/(1000000*60)); 
    Serial_print(" minutes!");
    delay(500); // delay for all the Serial_prints to finish 

    timeBeforeSleep = now();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    esp_deep_sleep_start();  // after this, it won't return here — will restart from setup()
  }
}

uint32_t lastSend = 0;
void loop() {
  button.loop();
  button2.loop();
  evaluateIfDeepSleep();

  static uint32_t lastPrint = 0;
  static uint32_t lastVBattIdeLog = 0;
  
  /*
  if(millis() - lastPrint > 60*1000) {
    lastPrint = millis();
    printDiagnosticInfo();
  }*/

  if(millis() - lastSend > prefs.send_data_interval*60*1000) {
    lastSend = millis();
    Serial_print(String(prefs.send_data_interval) + " min passed doing send");
    fullCycleSend();
  }

  if(millis() - lastVBattIdeLog > 5*1000) {
    lastVBattIdeLog = millis();
    logVbattIde();
  }

  lastNow = millis();

  //updateSerial();

  if(prefs.light_sleep_enabled) {
    esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start(); // 5 ms sleep 
  } else {
    delay(1);
  }
}

double read_batt_v() {
  return analogRead(V_BATT_PIN) * 0.0006598; 
}

double read_solar_v() {
  return analogRead(V_SOALR_PIN) * 0.003532;
}

void turnOnModule() {
  Serial1.println("turning on");

  for(int i=0; i<40;i++){
    pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW);
    delayMicroseconds(50*i);
    pinMode(GPRS_ON_PIN, INPUT);
    delay(3);
  }
  pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW);

  delay(1000);

  Serial_println("Turning on GPRS module!");
  digitalWrite(GPRS_POWER_PIN, LOW);
  Serial_println("waiting 1s ...");
  delay(1000);
  digitalWrite(GPRS_POWER_PIN, HIGH);
  Serial_println("Done");
}

void turnOffModule() {
  pinMode(GPRS_ON_PIN, INPUT);
  //digitalWrite(GPRS_ON_PIN, HIGH);
  Serial_println("gprs high -> turning off");
}

#define V_BATT_IDE_LEN 50
float vbattIde[V_BATT_IDE_LEN];
int   vbattHead  = 0;       // next write position
int   vbattCount = 0;       // how many samples are valid 
float vbattSum   = 0.0f;    // running sum of valid samples

float getAvgVBattIde(void) {
    if (vbattCount == 0) return logVbattIde();
    return vbattSum / (float)vbattCount;
}

float logVbattIde(void) {
    float val = read_batt_v();

    if (vbattCount < V_BATT_IDE_LEN) {
        // still filling the buffer
        vbattIde[vbattHead] = val;
        vbattSum += val;
        vbattHead = (vbattHead + 1) % V_BATT_IDE_LEN;
        vbattCount++;
    } else {
        // buffer full: overwrite oldest (at vbattHead)
        float old = vbattIde[vbattHead];
        vbattSum -= old;                // remove oldest from sum
        vbattIde[vbattHead] = val;      // store new
        vbattSum += val;                // add newest to sum
        vbattHead = (vbattHead + 1) % V_BATT_IDE_LEN;
    }
    return val;
}

void resetVBattIde(void) {
  // reset the circular buffer 
  vbattHead = 0;
  vbattCount = 0;
  vbattSum = 0.0f;
}

void printVBattIde(void) {
    if (vbattCount == 0) {
        Serial_println("  [empty]");
        return;
    }

    // Oldest element index
    int idx = (vbattHead - vbattCount + V_BATT_IDE_LEN) % V_BATT_IDE_LEN;

    for (int i = 0; i < vbattCount; i++) {
        Serial_print(String(vbattIde[idx], 3));
        Serial_print(",");
        idx = (idx + 1) % V_BATT_IDE_LEN;
    }
}

unsigned long httpGetStart = 0;
int signalStrength = -1;
int regDuration = -1;
int gprsRegDuration = -1;
void tap2(Button2& btn) {
  fullCycleSend();
}

void fullCycleSend() {
  const int nSendRetrys = 3;
  bool sendOk = false;

  for(int i=0; i<nSendRetrys && !sendOk; i++) {
    Serial_print("Sending try n:"); Serial_println(i);
    
    signalStrength = -1;
    regDuration = -1;
    gprsRegDuration = -1;
    turnOnModule();
      
    httpGetStart = millis();
    SendResult r = runHttpGetHot();
    if (r == SendResult::OK) {
      Serial_println("Send successful!");
      sendOk = true;
    } else {
      logSendErrorForResult(r);
      elog.log(ErrorLogger::ERR_SEND_REPEAT);
      Serial_print("Send failed: ");
      Serial_println(sendResultToStr(r)); // human-readable reason
      error_notify_led = 1;
    }


    Serial_println("Finished sending!");
    Serial_print("Duration:"); Serial_print((millis()-httpGetStart)/1000); Serial_println("s");

    turnOffModule();
    delay(1000);
  } 
}

void printDiagnosticInfo() {
  Serial_print("\n");
  Serial_print("WIND_LOG_STORE_LEN: "); Serial_println(WIND_LOG_STORE_LEN);
  Serial_print("w_count: "); Serial_println(w_count);
  Serial_print("Time: "); Serial_println(getFormattedTimeString());
  Serial_print("Last send: "); Serial_print((millis() - lastSend)/(1000*60)); Serial_println(" min ago"); 
  Serial_print("v batt ide:"); Serial_println(String(getAvgVBattIde(), 3));
  Serial_print("v batt:"); Serial_println(String(read_batt_v(), 3));
  Serial_print("v solar:"); Serial_println(String(read_solar_v(), 3));
  //Serial_print("ide array:"); printVBattIde(); Serial_println();

  // Serial_println("wind data: " + getWindData());
  // TODO implement idk Serial_println("last_dir_read: " + String(last_direction_read));

  // read AS5600 angle:
  digitalWrite(AS600_POWER_PIN, HIGH); delay(20); // turn it on and wait 
  int angle = as5600.readAngle() * 360 / 4096;
  int error = as5600.lastError();
  if(error == 0) {
    Serial_print(F("AS600 Angle: ")); Serial_println(angle);
  } else {
    Serial_print(F("AS600 Angle: Read error: ")); Serial_println(error);
  }

  String postBody = getPostBody();
  Serial_print("Post body (len=");
  Serial_print(postBody.length());
  Serial_print("): ");
  Serial_println(postBody);

  Serial_println("All errors:"); Serial_println(elog.getAll());

  Serial_println();
}

void tap(Button2& btn) {
  printPreferences();
  printDiagnosticInfo();
}

String inputBuffer = "";
bool updateSerial() {
  delay(30);
  
  while (SerialDBG.available()) {
    char c = SerialDBG.read();
    SerialDBG.write(c);
    //Serial_print("read:'");
    //Serial_print(c);
    //Serial_println("'");

    if (c == '\n' || c == '\r') {
      Serial_print("Command:'");
      Serial_print(inputBuffer);
      Serial_print("'\r\n");

      if (inputBuffer[0] == 'o' || inputBuffer[0] == 'O') {
        turnOnModule();
      } else if (inputBuffer[0] == 'x' || inputBuffer[0] == 'X') {
        turnOffModule();
      } else if (inputBuffer[0] == 's' || inputBuffer[0] == 'S') {
        fullCycleSend();
      } else if (inputBuffer[0] == 'i' || inputBuffer[0] == 'I') {
        printDiagnosticInfo();
      } else if (inputBuffer[0] == 'n') {
        setPhoneNumber(inputBuffer);
      } else {
        SerialAT.println(inputBuffer.c_str());
        //SerialAT.write(inputBuffer.c_str());       //Forward what Serial received to Software Serial Port
      }
      
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  String atResponse = "";
  while (SerialAT.available()) {
    //Forward what Software Serial received to Serial Port
    char atRead = SerialAT.read();
    atResponse += atRead;
    delay(1); // we need to wait since SerialAt seems to send quite slowly 
  }
  if(atResponse.length() > 0) {
    SerialDBG.print("AT:");
    SerialDBG.print(atResponse);
    SerialDBG.println();
  }

  return true;
}


#ifdef PRINT_SIM_COMM
  #define DBG_CMD(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_CMD(...) do {} while (0)
#endif

String sendCommand(const String& command, int timeoutMs = 1000, String expectedResponse = "OK") {
  SerialAT.println(command);

  DBG_CMD( Serial_print("#Command: "); Serial_println(command); );
  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      response += c;

      DBG_CMD( Serial_write(c); ); 

      // Early exit if we detect "OK"
      if (response.indexOf(expectedResponse) != -1) {
        DBG_CMD( Serial_println(";"); );
        return response;
      }
    }
    delay(1); // Yield to avoid tight spinning
  }
  #ifdef PRINT_SIM_COMM
  Serial_println("norsps?");
  #endif 
  return response;
}

void setPhoneNumber(String& inputBuffer) {
  String digits = "";
  for(int i=1; i<inputBuffer.length(); i++) {
    if (isDigit(inputBuffer[i])) digits += inputBuffer[i];
  }

  Serial_print("Setting phone number:");
  Serial_println(digits);

  Serial_println(sendCommand("AT+CPBS=\"ON\""));
  Serial_println(sendCommand("AT+CPBW=1,\"" + digits + "\",129,\"mojast\""));

  Serial_println(sendCommand("AT+CNUM"));

  Serial_println("Done!");
}

const char* regStatusToStr(int status) {
  switch (status) {
    case 0: return "Not registered, not searching";
    case 1: return "Registered, home network";
    case 2: return "Not registered, searching";
    case 3: return "Registration denied";
    case 4: return "Unknown";
    case 5: return "Registered, roaming";
    default: return "Invalid status";
  }
}

bool parseCSQResponse(const String& response) {
  int idx = response.indexOf("+CSQ:");
  if (idx == -1) return false;

  // Extract substring starting after "+CSQ:"
  String sub = response.substring(idx + 5);
  sub.trim(); // remove leading/trailing whitespace

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int rssi = sub.substring(0, commaIdx).toInt();
  Serial_print("s:");
  Serial_print(rssi);

  if(rssi > 0) {
    Serial_print("; signal strenght: ");
    Serial_print((rssi * 827 + 127) / 256);
    Serial_println("%");
  }
  signalStrength = rssi;
  return rssi > 0 && rssi < 99; // 99 = unknown or no signal
}

bool parseCGREGResponse(const String& response) {
  int idx = response.indexOf("REG:");
  if (idx == -1) return false;

  String sub = response.substring(idx + 4);
  sub.trim();

  int commaIdx = sub.indexOf(',');
  if (commaIdx == -1) return false;

  int status = sub.substring(commaIdx + 1).toInt();

  //Serial_print("GPRS registration status: ");
  //Serial_println(regStatusToStr(status));
  Serial_print(status);
  return status == 1 || status == 5; // Registered (home or roaming)
}

String getFormattedTimeString() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  return String(buf);
}

String imsiNum; 

bool parseCIMIResponse(const String& response) {
  int idx;
  idx = response.indexOf("AT+CIMI"); if (idx == -1) return false;
  idx = response.indexOf("OK"); if (idx == -1) return false;

  String digits = "";
  for (int i = 0; i < response.length(); i++) {
    if (isDigit(response[i])) digits += response[i];
  }  

  if (digits.length() < 10) return false;  // sanity check
  
  imsiNum = digits;
  return true;
}

String phoneNum; 

void readPhoneNum() {
  String response = sendCommand("AT+CNUM");
  int idx;
  idx = response.indexOf("+CNUM"); if (idx == -1) return;
  idx = response.indexOf("OK"); if (idx == -1) return;

  int firstQuote = response.indexOf(",\""); if (firstQuote == -1) return;
  int secondQuote = response.indexOf('"', firstQuote + 2); if (secondQuote == -1) return;

  phoneNum = response.substring(firstQuote + 2, secondQuote);
}

void windlog_shift_timestamps(int32_t delta)
{
    if (delta == 0 || w_count == 0) return;

    portENTER_CRITICAL(&timerMux);

    first_timestamp += delta;
    last_timestamp += delta;

    /*
    // compute index of oldest record
    uint16_t base = (w_head + WIND_LOG_STORE_LEN - w_count) % WIND_LOG_STORE_LEN;

    // iterate through all valid entries in logical order
    for (uint16_t i = 0; i < w_count; i++) {
        uint16_t idx = (base + i) % WIND_LOG_STORE_LEN;
        wind_log[idx].ts +=  delta;
    }
    */

    portEXIT_CRITICAL(&timerMux);
}

void shiftTimestampsOnNewTime(int newHour, int newMinute, int newSecond) {
  uint32_t oldTimestamp = get_log_timestamp();                              // Old timestamp (before correction)
  uint32_t newTimestamp = get_log_timestamp(newHour, newMinute, newSecond); // New correct time

  int32_t delta = newTimestamp - oldTimestamp;                      // How much the clock moved
  Serial_print("Shifting timestamps by: "); Serial_println(delta);
  windlog_shift_timestamps(delta);
}


bool parseCCLKResponse(const String& response) {
  // expected response example: +CCLK: "25/08/01,00:19:52+08"
  int idx = response.indexOf("CCLK:");
  if (idx == -1) return false;

  int quoteStart = response.indexOf('"', idx);
  int quoteEnd = response.indexOf('"', quoteStart + 1);
  if (quoteStart == -1 || quoteEnd == -1) return false;

  String datetime = response.substring(quoteStart + 1, quoteEnd);  // "25/08/01,00:19:52+08"

  // Split by delimiters: / , :
  int year   = datetime.substring(0, 2).toInt() + 2000;
  int month  = datetime.substring(3, 5).toInt();
  int day    = datetime.substring(6, 8).toInt();
  int hour   = datetime.substring(9, 11).toInt();
  int minute = datetime.substring(12, 14).toInt();
  int second = datetime.substring(15, 17).toInt();

  // Check for default time (00/01/01 etc.)
  if (year < 2023) return false;  // Reject obviously invalid time

  Serial_println("Got new date!");
  // save the time inside the TimeLib library
  shiftTimestampsOnNewTime(hour, minute, second); 
  setTime(hour, minute, second, day, month, year);

  // Print the parsed time
  Serial_print("New date:"); Serial_println(getFormattedTimeString());
  
  return true;
}

/*
Parse the response data:
The expected payload is: 
saved: <num written>\n
params:\n
<param name>:<param_value>\n
<param name>:<param_value>\n
<param name>:<param_value>\n
<param name>:<param_value>\n
*/
void saveNewPrefValue(String key, String value) {
  Serial_print("Saving the new pref: '");
  Serial_print(key); Serial_print("':'"); Serial_print(value); Serial_println("'");

  if(key == "pref_version") {
    prefs.pref_version = value.toInt();
  } 
  else if(key == "version") {
    value.toCharArray(prefs.version, sizeof(prefs.version));
  } 
  else if(key == "url_to_send") {
    value.toCharArray(prefs.url_to_send, sizeof(prefs.url_to_send));
  }
  else if(key == "store_wind_data_interval") {
    prefs.store_wind_data_interval = value.toInt();
  }
  else if(key == "error_led_on_time") {
    prefs.error_led_on_time = value.toInt();
  }
  else if(key == "dir_led_on_time") {
    prefs.dir_led_on_time = value.toInt();
  }
  else if(key == "spin_led_on_time") {
    prefs.spin_led_on_time = value.toInt();
  }
  else if(key == "as5600_pwr_on_time") {
    prefs.as5600_pwr_on_time = value.toInt();
  }
  else if(key == "as5600_read_interval") {
    prefs.as5600_read_interval = value.toInt();
  }
  else if(key == "light_sleep_enabled") {
    prefs.light_sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_enabled") {
    prefs.sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_hour_start") {
    prefs.sleep_hour_start = value.toInt();
  } 
  else if(key == "sleep_hour_end") {
    prefs.sleep_hour_end = value.toInt();
  }
  else if(key == "blink_led_on_time") {
    prefs.blink_led_on_time = value.toInt();
  } 
  else if(key == "blink_led_interval") {
    prefs.blink_led_interval = value.toInt();
  }
  else if(key == "send_data_interval") {
    prefs.send_data_interval = value.toInt();
  }
  else {
    Serial_print("Unable to find the prefs key: '"); Serial_print(key); Serial_println("'");
  }
}


void parseReturnData(String data) {
  int prefsPos = data.indexOf("prefs:");
  if (prefsPos < 0) {
    Serial_println("No 'params' section found.");
    return;
  }

  // Start after "prefs"
  int pos = data.indexOf('\n', prefsPos);
  if (pos < 0) {
    Serial_println("No newline after 'params'.");
    return;
  }
  pos++; // move past newline

  while (pos < data.length()) {
    // Find the next colon — separates key from value
    int colonPos = data.indexOf(':', pos);
    if (colonPos < 0) break;

    // Find the next newline — end of this line
    int lineEnd = data.indexOf('\n', colonPos);
    if (lineEnd < 0) lineEnd = data.length();

    // Extract key and value
    String key = data.substring(pos, colonPos);
    String value = data.substring(colonPos + 1, lineEnd);

    // Trim simple whitespace and trailing commas
    key.trim();
    value.trim();

    saveNewPrefValue(key, value);
    // Advance to next line
    pos = lineEnd + 1;
  }

  savePreferences();
  Serial_println("Done params section.\n");
  Serial_println("Reseting the module to apply the settings soon.\n\n");
  delay(2000); // wait so that everything can print before reseting 
  esp_restart();
}

bool parseHTTPREADResponse(const String& response) {
  int startIdx = response.indexOf("+HTTPREAD:");
  if (startIdx == -1) {
    Serial_println("No +HTTPREAD header found");
    return false;
  }

  // Find the first line break after the header
  int dataStart = response.indexOf('\n', startIdx);
  if (dataStart == -1) return false;

  // Trim until actual data
  String data = response.substring(dataStart + 1);
  data.trim();

  // Remove any trailing "OK" or extra content
  int okIdx = data.indexOf("OK");
  if (okIdx != -1) {
    data = data.substring(0, okIdx);
    data.trim();
  }

  Serial_print("Parsing return data: \r\n'");
  data.replace("\n", "\r\n");
  Serial_print(data);
  data.replace("\r\n", "\n");
  Serial_println("'");

  parseReturnData(data);
  return true;
}

bool parseCSMINSResponse(const String& response) {
  int idx = response.indexOf("+CSMINS:");
  if (idx == -1) return false;

  return response.indexOf("0,1") != -1;
}

bool waitForResponse(const String& command,
                     unsigned long timeoutS,
                     bool (*parseFn)(const String&),
                     unsigned long retryDelay = 1000) {
  String response = "";
  unsigned long start = millis();
  bool responseOk = false;

  do {
    response = sendCommand(command);
    responseOk = parseFn ? parseFn(response) : (response != "");

    if (!responseOk) {
      if (millis() - start > timeoutS*1000) {
        Serial_print(" Command: '"); Serial_print(command); 
        Serial_println("' timeouted, aborting.");
        return false;
      }
      delay(retryDelay); // wait before retrying
    }
  } while (!responseOk);

  Serial_print(command); Serial_print(" done. Duration: "); 
  Serial_print(String((millis() - start)/1000.0, 2)); Serial_println("s");
  return true;
}


String zeros(int length) {
  String result;
  result.reserve(length);
  for (int i = 0; i < length; ++i) {
    result += '0';
  }
  return result;
}


String getPostBody() {
  String body = "";
  body += "pref=" + String(prefs.pref_version) + ";";
  body += "ver=" + String(prefs.version) + ";";
  body += "imsi=" + imsiNum + ";";
  body += "phoneNum=" + phoneNum + ";";
  body += "vbatIde=" + String(getAvgVBattIde(), 3) + ";";
  body += "vbatGprs=" + String(read_batt_v(), 3) + ";";
  body += "vsol=" + String(read_solar_v(), 1) + ";";
  body += "dur=" + String((millis() - httpGetStart) / 1000.0, 1) + ";";
  body += "signal=" + String(signalStrength) + ";";
  body += "regDur=" + String(regDuration / 1000.0, 1) + ";";
  body += "gprsRegDur=" + String(gprsRegDuration / 1000.0, 1) + ";";
  body += "errors=" + elog.getAllForSend() + ";";
  body += getWindData();

  return body;
}

String waitForHttpActionResponse(unsigned long timeoutMs) {
  unsigned long start = millis();
  String line = "";

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      Serial_write(c);
      if (c == '\n') {
        if (line.startsWith("+HTTPACTION:")) {
          return line;
        }
        line = "";  // reset for next line
      } else if (c != '\r') {
        line += c;
      }
    }
  }

  Serial_println("Timeout!");
  return "";  // timeout
}

bool sendPOST() {
  if (sendCommand("AT+HTTPINIT", 1000) == "") return false;
  if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
  if (sendCommand(String("AT+HTTPPARA=\"URL\",\"") + prefs.url_to_send + imsiNum + "\"", 500) == "") return false;

  String postData = getPostBody();
  if (sendCommand("AT+HTTPDATA=" + String(postData.length()) + ",10000", 500, "DOWNLOAD") == "") return false;
  if (sendCommand(postData, 1000) == "") return false;
  Serial_println("Done sending postData!");

  if (sendCommand("AT+HTTPACTION=1", 5000) == "") return false; // 0 = GET, 1 = POST, 2 = HEAD
  String actionResult = waitForHttpActionResponse(10000);  // wait up to 10s

  if (actionResult.indexOf(",2") > 0) {
    return true;
  } else {
    return false;
  }

  return true;
}


const char* sendResultToStr(SendResult r) {
  switch (r) {
    case SendResult::OK:              return "OK";
    case SendResult::AT_FAIL:         return "AT failed";
    case SendResult::NO_SIM:          return "No SIM";
    case SendResult::CSQ_FAIL:        return "CSQ failed";
    case SendResult::REG_FAIL:        return "Network registration failed";
    case SendResult::CIMI_FAIL:       return "CIMI failed";
    case SendResult::GPRS_SETUP_FAIL: return "GPRS setup failed";
    case SendResult::HTTP_FAIL:       return "HTTP failed";
    default:                          return "Unknown";
  }
}

void logSendErrorForResult(SendResult r) {
  switch (r) {
    case SendResult::AT_FAIL:         elog.log(ErrorLogger::ERR_SEND_AT_FAIL);   break;
    case SendResult::NO_SIM:          elog.log(ErrorLogger::ERR_SEND_NO_SIM);    break;
    case SendResult::CSQ_FAIL:        elog.log(ErrorLogger::ERR_SEND_CSQ_FAIL);  break;
    case SendResult::REG_FAIL:        elog.log(ErrorLogger::ERR_SEND_REG_FAIL);  break;
    case SendResult::CIMI_FAIL:       elog.log(ErrorLogger::ERR_SEND_CIMI_FAIL); break;
    case SendResult::GPRS_SETUP_FAIL: elog.log(ErrorLogger::ERR_SEND_GPRS_FAIL); break;
    case SendResult::HTTP_FAIL:       elog.log(ErrorLogger::ERR_SEND_HTTP_FAIL); break;
    default: break; // OK or unknown: no log here
  }
}

SendResult runHttpGetHot() {
  Serial_println("\n\nExecuting HTTP GET HOT...");

  if (!waitForResponse("AT", 10, nullptr)) return SendResult::AT_FAIL;

  if (!waitForResponse("AT+CSMINS?", 20, parseCSMINSResponse, 200)) {
    Serial_println("No Sim detected!.");
    elog.log(ErrorLogger::ERR_SEND_NO_SIM);
    return SendResult::NO_SIM;
  }

  if (!waitForResponse("AT+CSQ", 20, parseCSQResponse, 2000)) return SendResult::CSQ_FAIL;

  unsigned long start = millis();
  if (!waitForResponse("AT+CREG?", 120, parseCGREGResponse, 2000)) return SendResult::REG_FAIL;
  regDuration = millis() - start;

  start = millis();
  if (!waitForResponse("AT+CGREG?", 120, parseCGREGResponse, 2000)) return SendResult::REG_FAIL;
  gprsRegDuration = millis() - start;

  if (!waitForResponse("AT+CCLK?", 10, parseCCLKResponse, 2000)) {
    // probably missing automatic time update NITZ
    sendCommand("AT+CLTS=1");
    sendCommand("AT&W");
  }

  if (!waitForResponse("AT+CIMI", 10, parseCIMIResponse, 2000)) return SendResult::CIMI_FAIL;
  Serial_print("Got IMSI:"); Serial_println(imsiNum);

  readPhoneNum();
  Serial_print("Got phone Num:"); Serial_println(phoneNum);

  // Step 1: Configure GPRS connection
  if (sendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 1000) == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=3,1,\"APN\",\"internet.simobil.si\"") == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=3,1,\"USER\",\"simobil\"") == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=3,1,\"PWD\",\"internet\"", 2000) == "") return SendResult::GPRS_SETUP_FAIL;

  // Step 2: Open GPRS bearer
  if (sendCommand("AT+SAPBR=1,1", 15000) == "") return SendResult::GPRS_SETUP_FAIL;
  if (sendCommand("AT+SAPBR=2,1", 1000) == "") return SendResult::GPRS_SETUP_FAIL;

  if (!sendPOST()) return SendResult::HTTP_FAIL;
  
  waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);
  

  // Step 5: Cleanup
  sendCommand("AT+HTTPTERM");
  sendCommand("AT+SAPBR=0,1");

  Serial_print("Success!");

  elog.clearAll(); // clear all the errors so they are not send again
  windlog_clear();
  /*
  portENTER_CRITICAL(&timerMux);
  Serial_print("We need to move: ");
  Serial_print(speed_avg_i - speed_avg_i_on_send);
  Serial_print(" data ...");
  // move tail [speed_avg_i_on_send .. speed_avg_i-1] to front starting at 0
  int dst = 0;
  for (int src = speed_avg_i_on_send; src < speed_avg_i; ++src, ++dst) {
    speeds_avg[dst]  = speeds_avg[src];
    speeds_max[dst]  = speeds_max[src];
    speeds_time[dst] = speeds_time[src];
  }
  speed_avg_i = dst;
  speed_avg_i_on_send = 0;
  speeds_avg_time_start = millis();
  directions_avg_i = 0;
  portEXIT_CRITICAL(&timerMux);
    */

  return SendResult::OK;
}








































