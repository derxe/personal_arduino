#define UART_DEBUG_TX_PIN  16
#define UART_DEBUG_RX_PIN  14 // unused pin 14

#include <SerialMod0.h>
SerialMod0 SerialUart0(UART_NUM_0, UART_DEBUG_TX_PIN, UART_DEBUG_RX_PIN); 
#define SerialDBG SerialUart0

#define Serial_print(x)    do { SerialDBG.print(x); /* Serial1.print(x);*/ } while (0)
#define Serial_println(x)  do { SerialDBG.println(x); /* Serial1.println(x);*/ } while (0)
#define Serial_write(x)  do { SerialDBG.write(x); /*Serial1.write(x);*/ } while (0)

#include <Preferences.h>
#include "esp_log.h" 
#include <SoftwareSerial.h>
#include "Button2.h"
#include "esp_timer.h"
#include "AS5600.h"
#include <Wire.h>
#include <TimeLib.h>
#include <Preferences.h>
#include "ErrorLogger.h"
#include "ResetDiagnostics.h"
#include "MyPrefs.h"
#include "FloatRunningAverage.h"
#include <string.h>  // for memcpy
#include "AHT20_SoftI2C.h"
#include "unix_compile_time.h"
#include "esp_task_wdt.h"

typedef struct {
    uint16_t avg;
    //uint16_t max; // we dont need to log max speed
    int16_t dir;      // from 0 - 360, negative values for errors 
    //uint16_t ts;   // we only log last and first log instead of logging every timestamp
} WindSample;

struct AppPrefs { 
  uint16_t pref_version;               // just an intiger to difirentiata between different versions 
  uint32_t pref_set_date;              // the date time when the preferences were set
  char     version[8];                 // program/sw version
  char     url_data[128];              // url that is used to send the data to 
  char     url_prefs[128];             // url that is used to send the preferences to if requeste
  char     url_errors[128];            // url that is used to send all the error names 

  uint8_t  light_sleep_enabled;        // light sleep between reads, 0 if disabled, and 1 if enabled and 2 if enabled only after 1 min after boot 
  uint8_t  sleep_enabled;              // 0 if disabled, and 1 if enabled, 2 sends data once after each sleep cycle 
  uint16_t sleep_2_send_interval_s;    // seconds to collect data before sending in sleep mode 2 
  int8_t   sleep_hour_start;           // which hour the device gets to sleep in 24 hour format 
  int8_t   sleep_hour_end;             // which hour the device is expected to wake up again

  uint16_t store_wind_data_interval_s; // seconds between storing wind data (0 disables; also disables hal/read speed events)
  uint8_t  send_data_interval_min;     // minutes between data sends 
  uint8_t  n_send_retries;             // how many times do we retry sending 

  uint8_t  at_timeout_s;               // timeouts for crutical AT commands
  uint8_t  sim_timeout_s;
  uint8_t  csq_timeout_s;             
  uint8_t  creg_timeout_s;  
  uint8_t  cgreg_timeout_s;  

  uint8_t  error_led_on_time_ms;    // ms error LED stays on when blinking (<=0 disables)
  uint8_t  dir_led_on_time_ms;      // ms direction LED stays on when blinking (<=0 disables)
  uint8_t  spin_led_on_time_ms;     // ms spin LED stays on when blinking (<=0 disables)
  uint8_t  blink_led_on_time_ms;    // ms blink LED stays on when blinking (<=0 disables)
  uint8_t  blink_led_interval_ds;   // deciseconds (0.1 s) between blink cycles

  uint8_t  as5600_pwr_on_time_ms;   // ms to wait after power on before reading (also an interrupt cycle)
  uint16_t as5600_read_interval_s;  // seconds between direction reads (0 disables)

  uint16_t wind_log_store_len;      // how many wind data we can store, to be send on the next interaction 

  float  vbat_calib;                // calibration for converting the measured voltage on vbat to the actual voltage
  float  vsolar_calib;              // calibration for converting the measured voltage on vsolar to the actual voltage
};

// define default preferences:
AppPrefs prefs = {
  /*pref_version*/              0,
  /*pref_set_date*/             0, 
  /*version*/                   "v8.1",
  /*url_data*/                  "http://46.224.24.144/veter/save/",
  /*url_prefs*/                 "http://46.224.24.144/veter/save_prefs/",
  /*url_errors*/                "http://46.224.24.144/veter/save_error/",

  /*light_sleep_enabled*/       1, 
  /*sleep_enabled*/             2,
  /*sleep_2_send_interval_s*/   20, 
  /*sleep_hour_start*/          18,     // time hours
  /*sleep_hour_end*/            6,      // time hours

  /*store_wind_data_interval_s*/  5,     
  /*send_data_interval_min*/      2,    
  /*n_send_retries*/              5,

  /*at_timeout_s*/              10,     
  /*sim_timeout_s*/             20,     
  /*csq_timeout_s*/             120,    
  /*creg_timeout_s*/            120,    
  /*cgreg_timeout_s*/           120,    

  /*error_led_on_time_ms*/      10,  
  /*dir_led_on_time_ms*/        10,  
  /*spin_led_on_time_ms*/       20,  
  /*blink_led_on_time_ms*/      20,  
  /*blink_led_interval_ds*/     20,  // deciseconds (0.1 s)

  /*as5600_pwr_on_time_ms*/       100, 
  /*as5600_read_interval_s*/      0,    

  /*wind_log_store_len*/        600,    

  /*vbat_calib*/                0.0006598, 
  /*vsolar_calib*/              0.003532
};


enum class SendResult : int {
  OK                   =  1,   // success
  AT_FAIL              = -1,   // "AT" didn't respond
  NO_SIM               = -2,   // CSMINS? => no SIM
  CSQ_FAIL             = -3,   // CSQ failed
  REG_FAIL             = -4,   // CREG? or CGREG? failed (both map here)
  CCLK_FAIL            = -5,   // failed getting time
  CIMI_FAIL            = -6,   // CIMI failed
  GPRS_SETUP_FAIL      = -7,   // any SAPBR step failed
  HTTP_FAIL            = -8,   // sendPOSTData / HTTPREAD failed
};

//#define PRINT_SIM_COMM 
//#define PRINT_MAGNET_READ_DEBUG


//#define SerialDBG Serial

#define RX_PIN 37
#define TX_PIN 39
//SoftwareSerial SerialAT(TX_PIN, RX_PIN);  // SIM800L <-> Arduino
#define SerialAT Serial1

#define BUTTON_PIN       9
#define BUTTON_2_PIN     11
#define GPRS_ON_PIN      18   // MOS FET turn on pin
#define GPRS_POWER_PIN   35   // PWX pin on the SIM800C board
#define V_BATT_PIN       13
#define V_SOALR_PIN      8

#define POWER_GPRS_BOARD_ON()  do { pinMode(GPRS_ON_PIN, OUTPUT); digitalWrite(GPRS_ON_PIN, LOW); } while (0)
#define POWER_GPRS_BOARD_OFF() do { digitalWrite(GPRS_ON_PIN, HIGH); pinMode(GPRS_ON_PIN, INPUT); } while (0)


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
#define AS5600_SDA_PIN             5
#define AS5600_SCL_PIN             7      
#define AS600_POWER_PIN            4
AS5600 as5600; 

// this temerature sensor is inside
#define AHT20_1_PWR_PIN   40
#define AHT20_1_SDA_PIN   36
#define AHT20_1_SCL_PIN   38
AHT20SoftI2C aht1(AHT20_1_SDA_PIN, AHT20_1_SCL_PIN);

// for the outside sensor 
#define AHT20_2_PWR_PIN   17
#define AHT20_2_SDA_PIN   21
#define AHT20_2_SCL_PIN   34
AHT20SoftI2C aht2(AHT20_2_SDA_PIN, AHT20_2_SCL_PIN);


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

#define DEEP_SLEEP_DURATION  (3600ULL * 1000*1000) // value in microseconds so: one hour
//#define DEEP_SLEEP_DURATION  5*60 * 1000 * 1000  // 20 seconds
RTC_DATA_ATTR time_t timeBeforeSleep = 0;      // stores last time before deep sleep

void printVersionAndCompileDate() {
  Serial_print("Compiled on ");
  Serial_print(__DATE__);
  Serial_print(" at ");
  Serial_print(__TIME__);
  Serial_print(" unix:");
  Serial_println(BUILD_UNIX_TIME);

  Serial_print("Version:"); Serial_println(prefs.version);
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

// pass in BUILD_UNIX_TIME so that we update the preferences on each new compile 
PrefBlob<AppPrefs> store(BUILD_UNIX_TIME);

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
  Serial_print("  pref_set_date: "); Serial_println(getFormattedUnixTime(prefs.pref_set_date));
  Serial_print("  version:    ");      Serial_println(prefs.version);
  Serial_print("  url_data:   ");  Serial_println(prefs.url_data);
  Serial_print("  url_prefs:  ");  Serial_println(prefs.url_prefs);
  Serial_print("  url_errors: ");  Serial_println(prefs.url_errors);
  Serial_println();

  Serial_print("  light_sleep_enabled:   ");  Serial_println(prefs.light_sleep_enabled);
  Serial_print("  sleep_enabled:         ");  Serial_println(prefs.sleep_enabled);
  Serial_print("  sleep_2_send_interval_s: ");  Serial_println(prefs.sleep_2_send_interval_s);
  Serial_print("  sleep_hour_start:      ");  Serial_println(prefs.sleep_hour_start);
  Serial_print("  sleep_hour_end:        ");  Serial_println(prefs.sleep_hour_end);
  Serial_println();

  Serial_print("  store_wind_data_interval_s: "); Serial_println(prefs.store_wind_data_interval_s);
  Serial_print("  send_data_interval_min:       ");       Serial_println(prefs.send_data_interval_min);
  Serial_print("  n_send_retries:           ");       Serial_println(prefs.n_send_retries);
  Serial_println();

  Serial_print("  at_timeout_s:    ");       Serial_println(prefs.at_timeout_s);
  Serial_print("  sim_timeout_s:   ");       Serial_println(prefs.sim_timeout_s);
  Serial_print("  csq_timeout_s:   ");       Serial_println(prefs.csq_timeout_s);
  Serial_print("  creg_timeout_s:  ");       Serial_println(prefs.creg_timeout_s);
  Serial_print("  cgreg_timeout_s: ");       Serial_println(prefs.cgreg_timeout_s);
  Serial_println();  

  Serial_print("  error_led_on_time_ms:  ");   Serial_println(prefs.error_led_on_time_ms);
  Serial_print("  dir_led_on_time_ms:    ");     Serial_println(prefs.dir_led_on_time_ms);
  Serial_print("  spin_led_on_time_ms:   ");    Serial_println(prefs.spin_led_on_time_ms);
  Serial_print("  blink_led_on_time_ms:  ");   Serial_println(prefs.blink_led_on_time_ms);
  Serial_print("  blink_led_interval_ds: ");  Serial_println(prefs.blink_led_interval_ds);
  Serial_println();

  Serial_print("  as5600_pwr_on_time_ms:   "); Serial_println(prefs.as5600_pwr_on_time_ms);
  Serial_print("  as5600_read_interval_s: "); Serial_println(prefs.as5600_read_interval_s);
  Serial_println();

  Serial_print("  wind_log_store_len:   "); Serial_println(prefs.wind_log_store_len);
  Serial_println();

  Serial_print("  vbat_calib:   ");   Serial_println(String(prefs.vbat_calib, 9));
  Serial_print("  vsolar_calib: "); Serial_println(String(prefs.vsolar_calib, 9));
  Serial_println("---------------------");
}

bool checkHalSpinSensorConnected() {
  // We cant test that since there is no way to control the power of the hal sensor
  // TODO check if the PIN is connected by giving a power to the sensor and then checking how the hal pin responds 
  return true;
}

void checkSensorsConnected() {
  if(!checkAS5600Connected()) {
    Serial_println("AS5600 not connected!");
    digitalWrite(DIR_LED_PIN, LOW);
    delay(400);
    digitalWrite(DIR_LED_PIN, HIGH);
    delay(400);
    digitalWrite(DIR_LED_PIN, LOW);
    delay(400);
    digitalWrite(DIR_LED_PIN, HIGH);
    delay(400);
    digitalWrite(DIR_LED_PIN, LOW);
    delay(400);
    digitalWrite(DIR_LED_PIN, HIGH);
    delay(400);
    digitalWrite(DIR_LED_PIN, LOW);
    delay(400);
    digitalWrite(DIR_LED_PIN, HIGH);
  }

  if(!checkHalSpinSensorConnected()) {
    Serial_print("Hal Spin sensor not connected!");
    digitalWrite(SPIN_LED_PIN, LOW);
    delay(400);
    digitalWrite(SPIN_LED_PIN, HIGH);
    delay(400);
    digitalWrite(SPIN_LED_PIN, LOW);
    delay(400);
    digitalWrite(SPIN_LED_PIN, HIGH);
  }
}

String version = "2.1";
bool accurateTimeSet = false;
bool hasSendAfterTurnOn = false; 

void setup() {
  pinMode(BLINK_LED_PIN, OUTPUT);  digitalWrite(BLINK_LED_PIN, HIGH); // on
  delay(500);

  hasSendAfterTurnOn = false;

  elog.init();
  ResetInfo ri = readResetInfo();
  printResetInfo(ri);

  switch (ri.reason) {
    case ESP_RST_BROWNOUT:       
      elog.log(ErrorLogger::ERR_RESET_BROWNOUT); break;

    case ESP_RST_PANIC:          
      elog.log(ErrorLogger::ERR_RESET_PANIC); break;

    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:            
      elog.log(ErrorLogger::ERR_RESET_WDT); break;

    default:                     
      elog.log(ErrorLogger::ERR_RESET_UNEXPECTED); break;

    /* expected bootups  */
    case ESP_RST_POWERON:        
       elog.log(ErrorLogger::LOG_RESET_POWERON); break;

    case ESP_RST_SW:        
       elog.log(ErrorLogger::LOG_RESET_SW); break;

    case ESP_RST_DEEPSLEEP:                
      break;
  }

  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 10*1000, // 10 seconds watchdog timout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    // Bitmask of all cores
    .trigger_panic = true,
	};
  ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
  esp_task_wdt_add(NULL);

  //Serial.begin(115200);
  setTime(15, 0, 0, 29, 9, 2025);

  SerialDBG.begin(115200);

  Serial_println();
  Serial_println("### PROGRAM START! ###");
  printVersionAndCompileDate();

  restoreTimeIfScheduledReset();

  loadPreferences();

  esp_log_level_set("i2c.master", ESP_LOG_NONE); 


  // TODO if the reason is unexpected_reset get more information about it 
  // We should also check ESP_RST_EXT and treat it as error reason 
  // the RST_EXT is when EN pin is pulled down 

  // TODO also log which part of the program was in when the reset happened to debug where it crashed


  if(timeBeforeSleep == 0) {
    Serial_println("Fresh start. Normal boot");
  } else {
    // timeWas set before sleep so we can check if is time to wake up already!
    setTime(timeBeforeSleep + DEEP_SLEEP_DURATION / 1000000);
    accurateTimeSet = true;
    Serial_print("Woke up from deep sleep. Current time:"); Serial_println(getFormattedTimeLibString());
    evaluateIfDeepSleep();
  }

  SerialAT.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // 33, 34);

  POWER_GPRS_BOARD_OFF();
  pinMode(GPRS_POWER_PIN, OUTPUT); digitalWrite(GPRS_POWER_PIN, HIGH); // off
  //pinMode(BLINK_LED_PIN, OUTPUT);  digitalWrite(BLINK_LED_PIN, HIGH); // on
  pinMode(SPIN_LED_PIN, OUTPUT);   digitalWrite(SPIN_LED_PIN, LOW);    // off
  pinMode(DIR_LED_PIN, OUTPUT);    digitalWrite(DIR_LED_PIN, LOW);          // off
  pinMode(ERROR_LED_PIN, OUTPUT);  digitalWrite(ERROR_LED_PIN, LOW);         // off
  pinMode(AHT20_1_PWR_PIN, OUTPUT);  digitalWrite(AHT20_1_PWR_PIN, LOW);
  pinMode(AHT20_2_PWR_PIN, OUTPUT);  digitalWrite(AHT20_2_PWR_PIN, LOW);
  // Power ON AHT20
  //pinMode(TIMER_PIN, OUTPUT); digitalWrite(TIMER_PIN, LOW);
  //pinMode(VANE_POWER_PIN, OUTPUT); digitalWrite(VANE_POWER_PIN, HIGH);  
  //gpio_set_drive_capability((gpio_num_t) VANE_POWER_PIN, GPIO_DRIVE_CAP_3);

  pinMode(AS600_POWER_PIN, OUTPUT); digitalWrite(AS600_POWER_PIN, LOW);  
  gpio_set_drive_capability((gpio_num_t) AS600_POWER_PIN, GPIO_DRIVE_CAP_3);


  pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);

  button.begin(BUTTON_PIN);
  button.setPressedHandler(tap);

  button2.begin(BUTTON_2_PIN);
  button2.setPressedHandler(tap2);

  checkSensorsConnected();

  // init the as5600 chip so we can read wind direction 
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN); 
  Wire.setClock(1000000UL); // 1 Mhz

  aht1.SDA_LOW();  // Put everything to low
  aht1.SCL_LOW();
  aht2.SDA_LOW(); 
  aht2.SCL_LOW();

  setCpuFrequencyMhz(80);

  welcomTurnOnBlink();

  const esp_timer_create_args_t hal_timer_args = {
    .callback = &onReadHal,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "hal_timer"
  };
  esp_timer_handle_t hal_timer;
  if(prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&hal_timer_args, &hal_timer);
    esp_timer_start_periodic(hal_timer, 4*1000);  // every 4 ms
  }

  esp_timer_handle_t blinkLed_timer;
  const esp_timer_create_args_t blinkLed_args = {
    .callback = &onBlinkLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "blinkLed_timer"
  };
  if(prefs.blink_led_on_time_ms > 0) {
    esp_timer_create(&blinkLed_args, &blinkLed_timer);
    esp_timer_start_periodic(blinkLed_timer, prefs.blink_led_on_time_ms * 1000); 
  }

  esp_timer_handle_t spinLed_timer;
  const esp_timer_create_args_t spinLed_args = {
    .callback = &onSpinLed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "spinLed_timer"
  };
  if(prefs.spin_led_on_time_ms > 0) {
    esp_timer_create(&spinLed_args, &spinLed_timer);
    esp_timer_start_periodic(spinLed_timer, prefs.spin_led_on_time_ms * 1000); 
  }

  esp_timer_handle_t dirLed_timer;
  const esp_timer_create_args_t dirLed_args = {
    .callback = &onDirLed,                
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "dirLed_timer"
  };
  if(prefs.dir_led_on_time_ms > 0) {
    esp_timer_create(&dirLed_args, &dirLed_timer);
    esp_timer_start_periodic(dirLed_timer, prefs.dir_led_on_time_ms * 1000);  
  }

  esp_timer_handle_t errorLed_timer;
  const esp_timer_create_args_t errorLed_args = {
    .callback = &onErrorLed,               
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "errorLed_timer"
  };
  if(prefs.error_led_on_time_ms > 0) {
    esp_timer_create(&errorLed_args, &errorLed_timer);
    esp_timer_start_periodic(errorLed_timer, prefs.error_led_on_time_ms * 1000); 
  }
  

  esp_timer_handle_t readSpeed_timer;
  const esp_timer_create_args_t readSpeed_args = {
    .callback = &onReadSpeed,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readSpeed_timer"
  };
  if(prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&readSpeed_args, &readSpeed_timer);
    esp_timer_start_periodic(readSpeed_timer, 1000 * 1000); // 1s
  }

  esp_timer_handle_t readDirection_timer;
  const esp_timer_create_args_t readDirection_args = {
    .callback = &onReadDirection,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "readDirection_timer"
  };
  if(prefs.as5600_read_interval_s > 0) { 
    esp_timer_create(&readDirection_args, &readDirection_timer);
    esp_timer_start_periodic(readDirection_timer, prefs.as5600_pwr_on_time_ms*1000); 
  }
  

  esp_timer_handle_t storeWindData_timer;
  const esp_timer_create_args_t storeWindData_args = {
      .callback = &onStoreWindData,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "storeWindData_timer"
  };
  if(prefs.store_wind_data_interval_s > 0) {
    esp_timer_create(&storeWindData_args, &storeWindData_timer);
    esp_timer_start_periodic(storeWindData_timer, prefs.store_wind_data_interval_s*1000*1000ULL); // X seconds (in microseconds)
  }

  Serial_println("Done init!");
}

bool readSensorTempHum_inside(float &tempC, float &humRH) {
  digitalWrite(AHT20_1_PWR_PIN, HIGH);

  // Put bus in idle state
  aht1.SDA_HIGH();
  aht1.SCL_HIGH();
  delay(200);

  if (!aht1.aht20_init()) {
    digitalWrite(AHT20_1_PWR_PIN, LOW);
    aht1.SDA_LOW();   // put everything to low
    aht1.SCL_LOW();
    return false;
  }

  bool success = aht1.aht20_read(tempC, humRH);
  success = aht1.aht20_read(tempC, humRH); // read twice to be safe

  digitalWrite(AHT20_1_PWR_PIN, LOW);

  aht1.SDA_LOW();
  aht1.SCL_LOW();
  return success;
}

bool readSensorTempHum_outside(float &tempC, float &humRH) {
  digitalWrite(AHT20_2_PWR_PIN, HIGH);

  // Put bus in idle state
  aht2.SDA_HIGH();
  aht2.SCL_HIGH();
  delay(200);

  if (!aht2.aht20_init()) {
    digitalWrite(AHT20_2_PWR_PIN, LOW);
    aht2.SDA_LOW();   // put everything to low
    aht2.SCL_LOW();
    return false;
  }

  bool success = aht2.aht20_read(tempC, humRH);
  success = aht2.aht20_read(tempC, humRH); // read twice to be safe

  digitalWrite(AHT20_2_PWR_PIN, LOW);

  aht2.SDA_LOW();
  aht2.SCL_LOW();
  return success;
}

uint32_t get_log_timestamp(int hour, int minute, int second) {
  //return hour*60*30 + minute*30 + second/2;  // round on every 2 seconds so that we can store it inside 16 bit int. Max:43200 < 2**16
  return hour*60*60 + minute*60 + second; // timestamp in seconds
}

uint32_t get_log_timestamp() {
  return get_log_timestamp(hour(), minute(), second());
}

#define PRINT_MAGNET_READ_DEBUG

#ifdef PRINT_MAGNET_READ_DEBUG
  #define DBG_MNG(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_MNG(...) do {} while (0)
#endif


void IRAM_ATTR onBlinkLed(void* arg) {
  static int nOnBlinkLedcalls = 0;
  nOnBlinkLedcalls ++;

  uint16_t toggleCount = prefs.blink_led_interval_ds*100 / prefs.blink_led_on_time_ms;
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


//#define AS5600_PWR_ON_TIME   60 
//#define AS5600_READ_INTERVAL 3000 // ms how ofter we want to read direction
//#define AS5600_IR_CYCLES     AS5600_READ_INTERVAL / AS5600_PWR_ON_TIME  // Interupt cycles before powering on and reading the sensor again 

bool checkAS5600Connected() {
  Wire.end();

  // when we give power to the sensor the we 
  // can measure the pulldownd on the sensor board to see if it is connected at all 

  digitalWrite(AS600_POWER_PIN, HIGH); 
  pinMode(AS5600_SCL_PIN, INPUT_PULLDOWN);
  pinMode(AS5600_SDA_PIN, INPUT_PULLDOWN);
  delayMicroseconds(20); // wait to settle

  int sda_read = analogRead(AS5600_SDA_PIN);
  int scl_read = analogRead(AS5600_SCL_PIN);
  bool sda_high = sda_read > 1000;
  bool scl_high = scl_read > 1000;

  bool something_connected = sda_high || scl_high;

  if(!sda_high) elog.logTmp(ErrorLogger::ERR_DIR_SDA_NOT_CONN);
  if(!scl_high) elog.logTmp(ErrorLogger::ERR_DIR_SCL_NOT_CONN);
  //Serial_print("sda:"); Serial_print(sda_high);
  //Serial_print(" scl:"); Serial_println(scl_high);

  // restore I2C
  pinMode(AS5600_SDA_PIN, INPUT);
  pinMode(AS5600_SCL_PIN, INPUT);
  Wire.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  Wire.setClock(1000000UL);

  return something_connected;
}

void IRAM_ATTR onReadDirection(void* arg) {
  static int directionReadCount = 0; 
  int angle = 0;
  int8_t as5600_error = 0;
  #define MAGNET_READ_REPEATS 4
  int8_t readRepeats;

  uint16_t ir_cycles = (prefs.as5600_read_interval_s*1000 / prefs.as5600_pwr_on_time_ms);

  //Serial.flush();
  if(directionReadCount == 1) {
    digitalWrite(AS600_POWER_PIN, HIGH);

    if(!checkAS5600Connected()) {
      Serial_println("Dir sensor not conn");
      elog.logTmp(ErrorLogger::ERR_DIR_NOT_CONNECTED);
      directionReadCount = 3; // skip the next step where the magnet is actually read
      digitalWrite(AS600_POWER_PIN, LOW); // the sensor is not connected so turn it off again
      error_notify_led = 1;
    }
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
      if(millis() < 1000*20) {// only log for first 20 s
        DBG_MNG( Serial_print("Read angle: "); Serial_println(angle); );
      }

      // what should be considered "facing north", how much can the angle diviate from north
      #define BLINK_MARGIN  20
      if(BLINK_MARGIN > angle || angle > 360-BLINK_MARGIN) direction_detected_north = 1; // blink north direction led
      
      portENTER_CRITICAL_ISR(&timerMux);
      if(directions_log_i < DIRECTIONS_LOG_LEN) {
        // save the measurement inside the log
        directions_log[directions_log_i++] = angle;
      } else {
        elog.logTmp(ErrorLogger::ERR_DIR_SHORT_BUF_FULL);
      }
      portEXIT_CRITICAL_ISR(&timerMux);

    } else {
      last_direction_read = as5600_error; // no angle was succesfully read due to an error
      Serial_print("Read angle error: "); {
      Serial_println(as5600_error);
      error_notify_led = 1;
      }
      elog.logTmp(ErrorLogger::ERR_DIR_READ);
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
    elog.logTmp(ErrorLogger::ERR_WIND_SHORT_BUF_FULL);
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
    uint16_t oldest = (uint16_t)((w_head + prefs.wind_log_store_len - w_count) % prefs.wind_log_store_len);
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
    
    w_head = (uint16_t)((w_head + 1) % prefs.wind_log_store_len);
    if (w_count < prefs.wind_log_store_len) {
        w_count++;
    } else{
      first_timestamp += prefs.store_wind_data_interval_s; // just increase the fist timestamp by the expected interval that timestamps increase (by interval )
      // buffer full -> oldest is implicitly dropped
      elog.logTmp(ErrorLogger::ERR_WIND_BUF_OVERWRITE);
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
      uint16_t first_run = (uint16_t)((base + to_copy <= prefs.wind_log_store_len) ? to_copy : (prefs.wind_log_store_len - base));
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

  // if there is no directions_log set the value to -1 so that we know that no value was read
  int16_t avgDir = directions_log_i == 0 ? -1 : average_direction(directions_log, directions_log_i);
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


bool isSleepHour(int start, int end, int hour) {
  if (start > end) 
    return start <= hour || hour < end;
  else
    return end > hour && hour >= start;
}

bool isDeepSleepTime() {
  if(prefs.sleep_enabled == 0) return false; // dont do anything if it is disabled 
  if(prefs.sleep_enabled == 2 && hasSendAfterTurnOn == false) return false; // we dont go to sleep if we dont try sending first // if sleep mode 2 (sends once after sleeping) and if there is no send after turn on meaning it hasnt tried sending yet dont go to deep sleep until the send it at least once 
  return isSleepTime();
}

bool isSleepTime() {
  if(!accurateTimeSet) return false; // the time was not set from the GMS module yet

  if(timeStatus() == timeNotSet) return false; // how can we sleep if we dont know what the time is!

  //Serial_print("sleep?"); Serial_println(isSleepHour(prefs.sleep_hour_start, prefs.sleep_hour_end, hour()));
  // we sleep at night ofcorse! from 8 PM to 6 AM
  return isSleepHour(prefs.sleep_hour_start, prefs.sleep_hour_end, hour());
}

void goToDeepSleep() {
    Serial_print("Current time is:"); Serial_println(getFormattedTimeLibString());
    Serial_print("It is time to go deep sleep for: "); Serial_print(DEEP_SLEEP_DURATION/(1000000*60)); 
    Serial_print(" minutes!");
    delay(500); // delay for all the Serial_prints to finish 

    timeBeforeSleep = now();
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    esp_deep_sleep_start();  // after this, it won't return here — will restart from setup()
}

void evaluateIfDeepSleep() {
  if(isDeepSleepTime()) goToDeepSleep();
}

bool isTimeToSendData(uint32_t secSinceLastSend) {
  if(prefs.sleep_enabled == 2 && isSleepTime()) {
      // if the sleep mode 2 is on and we are currently in the time when we should be sleeping then we check the 
      // sleep2 send interval instead of the usal send_data interval :) 
      return secSinceLastSend > prefs.sleep_2_send_interval_s;
  } else {
    // normal case when the devices is on and sending data 
    return secSinceLastSend > prefs.send_data_interval_min*60;
  }
}

bool isLightSleep() {
  if(prefs.light_sleep_enabled == 1) return true;  // default state, sleep enabled
  if(prefs.light_sleep_enabled == 0) return false; // sleep disabled 
  
  if(prefs.light_sleep_enabled == 2) {
    // special debug case where the sleep is disabled in frist 2 seconds 
    bool enoughTimePassed = millis() > 1*60*1000; 
    return enoughTimePassed;
  }

  return true; // undefined state presumes light sleep is enabled
}

FloatRunningAverage<32> vBattAvg(read_batt_v);
FloatRunningAverage<8> vSolarAvg(read_solar_v);


uint32_t lastSend = 0;
uint32_t lastSuccessfulSend = 0;
void loop() {
  button.loop();
  button2.loop();
  evaluateIfDeepSleep();
  esp_task_wdt_reset();

  static uint32_t lastPrint = 0;
  static uint32_t lastVBattIdeLog = 0;
  
  /*
  if(millis() - lastPrint > 60*1000) {
    lastPrint = millis();
    printDiagnosticInfo();
  }*/

  uint32_t secSinceLastSend = (millis() - lastSend)/1000;
  if(isTimeToSendData(secSinceLastSend)) {
    lastSend = millis();
    Serial_println(String(secSinceLastSend) + " s passed doing send");
    fullCycleSend();
  }

  // read battery voltage every 5s
  if(millis() - lastVBattIdeLog > 5*1000) {
    lastVBattIdeLog = millis();
    vBattAvg.log();
    vSolarAvg.log();
  }

  // check if the station was unable to send the data for longer then 1 hour 
  #define NO_SEND_FORCE_RESET_TIME  1*60*60*1000
  if(millis() - lastSuccessfulSend > NO_SEND_FORCE_RESET_TIME) { 
    Serial1.println("Unable to send the data for longer then 1 hour, force resetting.");
    elog.log(ErrorLogger::ERR_CANT_SEND_FORCE_RST); 
    esp_restart();  // software reset
  }


  lastNow = millis();

  //updateSerial();

  if(prefs.light_sleep_enabled == 1) {
    esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start(); // 5 ms sleep 
  } else {
    // only go to light sleep if enough time passed after reset. So that we can connect to USB after reseting 
    
  }

  if(isLightSleep()){
    esp_sleep_enable_timer_wakeup(5*1000); esp_light_sleep_start();
  } else {
    delay(2);
  }
}

float read_batt_v() {
  return analogRead(V_BATT_PIN) *  prefs.vbat_calib; 
}

float read_solar_v() {
  return analogRead(V_SOALR_PIN) * prefs.vsolar_calib; 
}

void turnOnModule() {
  Serial1.println("turning on");

  for(int i=0; i<40;i++){
    POWER_GPRS_BOARD_ON();
    delayMicroseconds(50*i); 
    POWER_GPRS_BOARD_OFF();
    delay(3);
  }
  POWER_GPRS_BOARD_ON();

  delay(1000);

  Serial_println("Turning on GPRS module!");
  digitalWrite(GPRS_POWER_PIN, LOW);  // the power pin has to be pulle to low for 1 second in order to turn
  Serial_println("waiting 1s ...");
  delay(1000);
  digitalWrite(GPRS_POWER_PIN, HIGH);
  Serial_println("Done");
}

void turnOffModule() {
  POWER_GPRS_BOARD_OFF();

  Serial_println("gprs high -> turning off");
}

unsigned long httpGetStart = 0;
int signalStrength = -1;
int simDuration = -1;
int regDuration = -1;
int gprsRegDuration = -1;
float temp_in;
float hum_in; 
float temp_out;
float hum_out; 

void tap2(Button2& btn) {
  fullCycleSend();
}

void fullCycleSend() {
  esp_task_wdt_delete(NULL);

  const int nSendRetrys = prefs.n_send_retries;
  bool sendOk = false;

  // we read temereature and humidity before tu
  temp_in = NAN; hum_in = NAN;
  int tempReadTries = 5;
  while(--tempReadTries > 0 && !readSensorTempHum_inside(temp_in, hum_in));

  Serial_print("temp_in:"); Serial_println(String(temp_in, 2));
  Serial_print("humy_in:"); Serial_println(String(hum_in, 2));

  if (temp_in == NAN) {
    elog.log(ErrorLogger::ERR_TEMP_READ);
  }

  // we read temereature and humidity before tu
  temp_out = NAN; hum_out = NAN;
  tempReadTries = 5;
  while(--tempReadTries > 0 && !readSensorTempHum_outside(temp_out, hum_out));

  Serial_print("temp_out:"); Serial_println(String(temp_out, 2));
  Serial_print("humy_out:"); Serial_println(String(hum_out, 2));

  if (temp_out == NAN) {
    elog.log(ErrorLogger::ERR_TEMP_READ);
  }

  hasSendAfterTurnOn = true; // we tried sending!
  for(int nTry=0; nTry<nSendRetrys && !sendOk; nTry++) {
    Serial_print("Sending try n:"); Serial_println(nTry);
    
    signalStrength = -1;
    simDuration = -1;
    regDuration = -1;
    gprsRegDuration = -1;
    turnOnModule();
      
    httpGetStart = millis();
    SendResult r = runHttpGetHot(nTry);
    if (r == SendResult::OK) {
      Serial_println("Send successful!");
      sendOk = true;
      lastSuccessfulSend = millis(); 
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

  esp_task_wdt_add(NULL);
}

void printDiagnosticInfo() {
  Serial_print("\n");
  Serial_print("WIND_LOG_STORE_LEN: "); Serial_println(WIND_LOG_STORE_LEN);
  Serial_print("w_count: "); Serial_println(w_count);
  Serial_print("Time: "); Serial_println(getFormattedTimeLibString());
  Serial_print("Last send: "); Serial_print((millis() - lastSend)/(1000*60)); Serial_println(" min ago"); 
  Serial_print("v batt avg:"); Serial_println(String(vBattAvg.get(), 3));
  Serial_print("v solar avg:"); Serial_println(String(vSolarAvg.get(), 3));
  Serial_print("v batt:"); Serial_println(String(read_batt_v(), 3));
  Serial_print("v solar:"); Serial_println(String(read_solar_v(), 3));

  float t = 0.0f; float h = 0.0f;
  if(readSensorTempHum_inside(t, h)) { 
    Serial_print("temp_in:"); Serial_println(String(t, 2));
    Serial_print("humy_in:"); Serial_println(String(h, 2));
  } else {
    Serial_println("temp_in: failed to read");
    Serial_println("humy_in: failed to read");
  }

  if(readSensorTempHum_outside(t, h)) { 
    Serial_print("temp_out:"); Serial_println(String(t, 2));
    Serial_print("humy_out:"); Serial_println(String(h, 2));
  } else {
    Serial_println("temp_out: failed to read");
    Serial_println("humy_out: failed to read");
  }

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

  //goToDeepSleep();
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

String getFormattedTimeLibString() {
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  return String(buf);
}

String getFormattedUnixTime(uint32_t unix_time) {
    tmElements_t tm;
    breakTime(unix_time, tm);  // fill 'tm' with the date/time fields

    char buf[24];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d_%02d:%02d:%02d", // we print with _ because this function is used in post body
             tm.Year + 1970, tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);

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
  accurateTimeSet = true;

  // Print the parsed time
  Serial_print("New date:"); Serial_println(getFormattedTimeLibString());
  
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
bool saveNewPrefValue(String key, String value) {
  Serial_print("Saving the new pref: '");
  Serial_print(key); Serial_print("':'"); Serial_print(value); Serial_println("'");

  if(key == "pref_version") {
    prefs.pref_version = value.toInt();
  } 
  else if(key == "version") {
    value.toCharArray(prefs.version, sizeof(prefs.version));
  } 
  else if(key == "url_data") {
    value.toCharArray(prefs.url_data, sizeof(prefs.url_data));
  }
  else if(key == "url_prefs") {
    value.toCharArray(prefs.url_prefs, sizeof(prefs.url_prefs));
  }
  else if(key == "url_errors") {
    value.toCharArray(prefs.url_errors, sizeof(prefs.url_errors));
  }
  else if(key == "store_wind_data_interval_s") {
    prefs.store_wind_data_interval_s = value.toInt();
  }
  else if(key == "error_led_on_time_ms") {
    prefs.error_led_on_time_ms = value.toInt();
  }
  else if(key == "dir_led_on_time_ms") {
    prefs.dir_led_on_time_ms = value.toInt();
  }
  else if(key == "spin_led_on_time_ms") {
    prefs.spin_led_on_time_ms = value.toInt();
  }
  else if(key == "as5600_pwr_on_time_ms") {
    prefs.as5600_pwr_on_time_ms = value.toInt();
  }
  else if(key == "as5600_read_interval_s") {
    prefs.as5600_read_interval_s = value.toInt();
  }
  else if(key == "light_sleep_enabled") {
    prefs.light_sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_enabled") {
    prefs.sleep_enabled = value.toInt();
  } 
  else if(key == "sleep_2_send_interval_s") {
    prefs.sleep_2_send_interval_s = value.toInt();
  }
  else if(key == "sleep_hour_start") {
    prefs.sleep_hour_start = value.toInt();
  } 
  else if(key == "sleep_hour_end") {
    prefs.sleep_hour_end = value.toInt();
  }
  else if(key == "blink_led_on_time_ms") {
    prefs.blink_led_on_time_ms = value.toInt();
  } 
  else if(key == "blink_led_interval_ds") {
    prefs.blink_led_interval_ds = value.toInt();
  }
  else if(key == "send_data_interval_min") {
    prefs.send_data_interval_min = value.toInt();
  }
  else if(key == "n_send_retries") {
    prefs.n_send_retries = value.toInt();
  }

  else if(key == "at_timeout_s") {
    prefs.at_timeout_s = value.toInt();
  }
  else if(key == "sim_timeout_s") {
    prefs.sim_timeout_s = value.toInt();
  }
  else if(key == "csq_timeout_s") {
    prefs.csq_timeout_s = value.toInt();
  }
  else if(key == "creg_timeout_s") {
    prefs.creg_timeout_s = value.toInt();
  }
  else if(key == "cgreg_timeout_s") {
    prefs.cgreg_timeout_s = value.toInt();
  }
  else if(key == "vbat_calib") {
    prefs.vbat_calib = value.toFloat();
  }
  else if(key == "vsolar_calib") {
    prefs.vsolar_calib = value.toFloat();
  }
  else if(key == "wind_log_store_len") {
    prefs.wind_log_store_len = value.toInt();
  }


  else {
    Serial_print("Unable to find the prefs key: '"); Serial_print(key); Serial_println("'");
    return false; // no new key was set
  }

  return true; // a new prefs key was set
}


Preferences prefsStorage;
static const char *PREF_NAMESPACE = "time";
static const uint32_t ESTIMATED_RESET_SECONDS = 2; 

void saveTimeAndScheduleReset() {
    time_t current = now();  // TimeLib current UNIX time

    prefsStorage.begin(PREF_NAMESPACE, false);
    prefsStorage.putULong64("saved_time", (uint64_t)current);
    prefsStorage.putBool("sched_reset", true);
    prefsStorage.end();

    Serial_println("Scheduled reset, saving time and restarting...");
    delay(2000); // wait so that everything can print before reseting 

    esp_restart();  // software reset
}

void restoreTimeIfScheduledReset() {
    prefsStorage.begin(PREF_NAMESPACE, false);
    bool scheduled = prefsStorage.getBool("sched_reset", false);
    uint64_t saved = prefsStorage.getULong64("saved_time", 0);
    prefsStorage.putBool("sched_reset", false);   // clear flag so it’s one-shot
    prefsStorage.end();

    if (scheduled && saved > 0) {
        time_t restored = (time_t)saved + ESTIMATED_RESET_SECONDS;

        setTime(restored); // Restore time into TimeLib
        accurateTimeSet = true;
        Serial_print("Restored time after scheduled reset: ");  Serial_println(restored);
        Serial_print("Restored datetime: "); Serial_println(getFormattedTimeLibString());
    } else {
        Serial_println("No scheduled reset time to restore.");
    }
}

bool shouldSendPrefs = false; 
bool shouldSendErrorNames = false; 
bool shouldReset = false; 
void parseReturnData(String& data) {
  shouldReset = true; 
  shouldSendPrefs = true; // on default we always reset the esp after changing preferences
  shouldSendErrorNames = false;
  bool newPrefsSet = false;
  int prefsPos = data.indexOf("prefs:");
  if (prefsPos < 0) {
    Serial_println("No 'prefs:' section found.");
    shouldReset = false; 
    shouldSendPrefs = false;
    return;
  }

  // Start after "prefs"
  int pos = data.indexOf('\n', prefsPos);
  if (pos < 0) {
    Serial_println("No newline after 'params'.");
    shouldReset = false; 
    shouldSendPrefs = false;
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

    Serial_print(" - pref key:"); Serial_println(key); 

    if(key == "no_reset") shouldReset = false; 
    else if(key == "no_send_prefs") shouldSendPrefs = false; 
    else if(key == "set_phone_num") setPhoneNumber(value);
    else if(key == "send_error_names") shouldSendErrorNames = true;
    else {
      bool prefsSet = saveNewPrefValue(key, value); 
      if(prefsSet) newPrefsSet = true;      
    }         
    
    // Advance to next line
    pos = lineEnd + 1;
  }

  if(newPrefsSet) {
    prefs.pref_set_date = now();  // save when the preferences were set (TimeLib current UNIX time)
    savePreferences();
    Serial_println("Done params section.\n");
  } else {
    Serial_println("No new preferences set.\n");
  }
  
}

String postReturnData;
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

  // Print the data
  Serial_print("Parsing return data:'");
  for (char c : data) {
      if (c == '\n') Serial_println(); 
      else Serial_write(c);
  }
  Serial_println("'");

  // store globally
  postReturnData = data;
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
  body.reserve(512);   // avoid fragmentation, improve speed
  body += "pref=" + String(prefs.pref_version) + ";";
  //body += "prefDate=" + getFormattedUnixTime(prefs.pref_set_date) + ";";
  body += "ver=" + String(prefs.version) + ";";
  //body += "imsi=" + imsiNum + ";";
  //body += "phoneNum=" + phoneNum + ";";
  body += "temp_in=" + String(temp_in, 1) + ";";
  body += "hum_in=" + String(hum_in, 0) + ";";
  body += "temp_out=" + String(temp_out, 1) + ";";
  body += "hum_out=" + String(hum_out, 0) + ";";
  body += "vbatIde=" + String(vBattAvg.get(), 3) + ";";
  body += "vbatGprs=" + String(read_batt_v(), 3) + ";";
  body += "vsol=" + String(vSolarAvg.get(), 3) + ";"; 
  body += "dur=" + String((millis() - httpGetStart) / 1000.0, 1) + ";";
  body += "signal=" + String(signalStrength) + ";";
  if (simDuration     > 3*1000) body += "simDur=" + String(simDuration / 1000.0, 1) + ";";
  if (regDuration     > 4*1000) body += "regDur=" + String(regDuration / 1000.0, 1) + ";";
  if (gprsRegDuration > 5*1000) body += "gprsRegDur=" + String(gprsRegDuration / 1000.0, 1) + ";";
  //body += "err_ver=" + String(ErrorLogger::ERROR_CODE_VERSION) + ";";
  body += "errors=" + elog.getAllForSend() + ";";
  body += getWindData();

  return body;
}

String getPostBodyPrefs() {
  String body;
  body.reserve(1024);   // avoid fragmentation, improve speed

  body += "compiled_on=" + getFormattedUnixTime(BUILD_UNIX_TIME) + ";";
  body += "pref_version=" + String(prefs.pref_version) + ";";
  body += "pref_set_date=" + getFormattedUnixTime(prefs.pref_set_date) + ";";
  body += "version=" + String(prefs.version) + ";";
  body += "url_data=" + String(prefs.url_data) + ";";
  body += "url_prefs=" + String(prefs.url_prefs) + ";";
  body += "url_errors=" + String(prefs.url_errors) + ";";

  body += "light_sleep_enabled=" + String(prefs.light_sleep_enabled) + ";";
  body += "sleep_enabled=" + String(prefs.sleep_enabled) + ";";
  body += "sleep_2_send_interval_s=" + String(prefs.sleep_2_send_interval_s) + ";";
  body += "sleep_hour_start=" + String(prefs.sleep_hour_start) + ";";
  body += "sleep_hour_end=" + String(prefs.sleep_hour_end) + ";";

  body += "store_wind_data_interval_s=" + String(prefs.store_wind_data_interval_s) + ";";
  body += "send_data_interval_min=" + String(prefs.send_data_interval_min) + ";";
  body += "n_send_retries=" + String(prefs.n_send_retries) + ";";

  body += "at_timeout_s=" + String(prefs.at_timeout_s) + ";";
  body += "sim_timeout_s=" + String(prefs.sim_timeout_s) + ";";
  body += "csq_timeout_s=" + String(prefs.csq_timeout_s) + ";";
  body += "creg_timeout_s=" + String(prefs.creg_timeout_s) + ";";
  body += "cgreg_timeout_s=" + String(prefs.cgreg_timeout_s) + ";";

  body += "error_led_on_time_ms=" + String(prefs.error_led_on_time_ms) + ";";
  body += "dir_led_on_time_ms=" + String(prefs.dir_led_on_time_ms) + ";";
  body += "spin_led_on_time_ms=" + String(prefs.spin_led_on_time_ms) + ";";
  body += "blink_led_on_time_ms=" + String(prefs.blink_led_on_time_ms) + ";";
  body += "blink_led_interval_ds=" + String(prefs.blink_led_interval_ds) + ";";

  body += "as5600_pwr_on_time_ms=" + String(prefs.as5600_pwr_on_time_ms) + ";";
  body += "as5600_read_interval_s=" + String(prefs.as5600_read_interval_s) + ";";

  body += "wind_log_store_len=" + String(prefs.wind_log_store_len) + ";";

  body += "vbat_calib=" + String(prefs.vbat_calib) + ";";
  body += "vsolar_calib=" + String(prefs.vsolar_calib) + ";";

  body += "imsi=" + imsiNum + ";";
  body += "phoneNum=" + phoneNum + ";";
  body += "err_ver=" + String(ErrorLogger::ERROR_CODE_VERSION) + ";";

  return body;
}

String getPostErrorsList() {
  String body;
  body.reserve(768);
  body += "errors=";

  body += "ERR_NONE:0,";

  // ---- SEND / GSM / HTTP ----
  body += "ERR_SEND_AT_FAIL:1,";
  body += "ERR_SEND_NO_SIM:2,";
  body += "ERR_SEND_CSQ_FAIL:3,";
  body += "ERR_SEND_REG_FAIL:4,";
  body += "ERR_SEND_CCLK_FAIL:5,";
  body += "ERR_SEND_CIMI_FAIL:6,";
  body += "ERR_SEND_GPRS_FAIL:7,";
  body += "ERR_SEND_HTTP_FAIL_DATA:8,";
  body += "ERR_SEND_UNKWN_FAIL:10,";
  body += "ERR_SEND_REPEAT:11,";
  body += "ERR_SEND_FAIL_WRONG_RESPONSE:12,";
  body += "ERR_SEND_PREFS_HTTP_FAIL:13,";
  body += "ERR_SEND_PREFS_HTTP_FAIL_RESPONSE:14,";
  body += "ERR_SEND_ERRORS_HTTP_FAIL:15,";
  body += "ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE:16,";

  // ---- DIR / I2C ----
  body += "ERR_DIR_READ:20,";
  body += "ERR_DIR_READ_ONCE:21,";
  body += "ERR_DIR_NOT_CONNECTED:22,";
  body += "ERR_DIR_SHORT_BUF_FULL:23,";
  body += "ERR_DIR_SDA_NOT_CONN:24,";
  body += "ERR_DIR_SCL_NOT_CONN:25,";

  // ---- WIND ----
  body += "ERR_WIND_BUF_OVERWRITE:30,";
  body += "ERR_WIND_SHORT_BUF_FULL:31,";

  // ---- TEMP ----
  body += "ERR_TEMP_READ:40,";

  // ---- POWER / RESET ----
  body += "ERR_RESET_BROWNOUT:52,";
  body += "ERR_RESET_PANIC:53,";
  body += "ERR_RESET_WDT:54,";
  body += "ERR_RESET_UNEXPECTED:55;";
  body += "ERR_CANT_SEND_FORCE_RST:56;";

  body += "LOG_RESET_SW:70;";
  body += "LOG_RESET_POWERON:71;";

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

bool sendPOST(const String &url, const String &body) {
    if (sendCommand("AT+HTTPPARA=\"CID\",1", 500) == "") return false;
    if (sendCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 500) == "") return false;

    if (sendCommand("AT+HTTPDATA=" + String(body.length()) + ",10000", 500, "DOWNLOAD") == "") return false;
    if (sendCommand(body, 1000) == "") return false;

    if (sendCommand("AT+HTTPACTION=1", 5000) == "") return false;

    String actionResult = waitForHttpActionResponse(10000);
    return (actionResult.indexOf(",2") > 0);
}


const char* sendResultToStr(SendResult r) {
  switch (r) {
    case SendResult::OK:              return "OK";
    case SendResult::AT_FAIL:         return "AT failed";
    case SendResult::NO_SIM:          return "No SIM";
    case SendResult::CSQ_FAIL:        return "CSQ failed";
    case SendResult::REG_FAIL:        return "Network registration failed";
    case SendResult::CCLK_FAIL:       return "get time failed";
    case SendResult::CIMI_FAIL:       return "get imi num failed";
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
    case SendResult::CCLK_FAIL:       elog.log(ErrorLogger::ERR_SEND_CCLK_FAIL); break;
    case SendResult::CIMI_FAIL:       elog.log(ErrorLogger::ERR_SEND_CIMI_FAIL); break;
    case SendResult::GPRS_SETUP_FAIL: elog.log(ErrorLogger::ERR_SEND_GPRS_FAIL); break;
    case SendResult::HTTP_FAIL:       elog.log(ErrorLogger::ERR_SEND_HTTP_FAIL_DATA); break;
    case SendResult::OK: break; 
    default: elog.log(ErrorLogger::ERR_SEND_UNKWN_FAIL); break;
  }
}

SendResult runHttpGetHot(int nTry) {
  Serial_println("\n\nExecuting HTTP GET HOT...");

  if (!waitForResponse("AT", prefs.at_timeout_s, nullptr)) return SendResult::AT_FAIL;

  unsigned long start = millis();
  if (!waitForResponse("AT+CSMINS?", prefs.sim_timeout_s, parseCSMINSResponse, 100)) {
    Serial_println("No Sim detected!.");
    return SendResult::NO_SIM;
  }
  simDuration = millis() - start;

  if (!waitForResponse("AT+CSQ", prefs.csq_timeout_s, parseCSQResponse, 500)) return SendResult::CSQ_FAIL;

  start = millis();
  if (!waitForResponse("AT+CREG?", prefs.creg_timeout_s, parseCGREGResponse, 500)) return SendResult::REG_FAIL;
  regDuration = millis() - start;

  start = millis();
  if (!waitForResponse("AT+CGREG?", prefs.cgreg_timeout_s, parseCGREGResponse, 500)) return SendResult::REG_FAIL;
  gprsRegDuration = millis() - start;

  if (!waitForResponse("AT+CCLK?", 10, parseCCLKResponse, 500)) {
    // probably missing automatic time update NITZ
    sendCommand("AT+CLTS=1");
    sendCommand("AT&W");

    if(nTry == 0) {
      // first try so we will repeat the sending and see if we can actually get the time next time
      return SendResult::CCLK_FAIL;
    }
  }

  if (!waitForResponse("AT+CIMI", 10, parseCIMIResponse, 500)) return SendResult::CIMI_FAIL;
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

  if (sendCommand("AT+HTTPINIT", 1000) == "") return SendResult::HTTP_FAIL;

  if (!sendPOST(prefs.url_data + imsiNum, getPostBody())) return SendResult::HTTP_FAIL;
  
  waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

  if(postReturnData.indexOf("saved:") < 0) {
    // expected "saved:" in the response if it is not there the response from the server was incorrect
    elog.log(ErrorLogger::ERR_SEND_FAIL_WRONG_RESPONSE);
    return SendResult::HTTP_FAIL;
  }

  if(!postReturnData.isEmpty()) {
    parseReturnData(postReturnData);
  }
  
  elog.clearAll(); // clear all the errors so they are not send again
  windlog_clear();

  if(shouldSendPrefs) {
    Serial_println();
    Serial_println("Sending preferences");
    bool postSuccess = sendPOST(prefs.url_prefs + imsiNum, getPostBodyPrefs());
    if (!postSuccess) {
      Serial_print("Sending preferences failed!");
      elog.log(ErrorLogger::ERR_SEND_PREFS_HTTP_FAIL);
    } else {
      waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

      if (postReturnData.indexOf("saved:") < 0) {
        Serial_println("Sending preferences failed wrong response!");
        elog.log(ErrorLogger::ERR_SEND_PREFS_HTTP_FAIL_RESPONSE);
      } else {
        Serial_println("Sending prefs OK!");
      }
    }
  }

  if(shouldSendErrorNames) {
    Serial_println();
    Serial_println("Sending error names");
    bool postSuccess = sendPOST(prefs.url_errors + imsiNum, getPostErrorsList());
    if (!postSuccess) {
      Serial_println("Sending errors failed!");
      elog.log(ErrorLogger::ERR_SEND_ERRORS_HTTP_FAIL);
    } else {
      waitForResponse("AT+HTTPREAD", 5, parseHTTPREADResponse);

      if (postReturnData.indexOf("saved:") < 0) {
        Serial_println("Sending errors failed wrong response!");
        elog.log(ErrorLogger::ERR_SEND_ERRORS_HTTP_FAIL_RESPONSE);
      } else {
        Serial_println("Sending errors OK!");
      }
    }
  }

  if(shouldReset) {
    Serial_println("Reseting the module to apply the settings soon.\n\n");
    saveTimeAndScheduleReset();
  } {
    Serial_println("No reset requested");
  }

  // Step 5: Cleanup
  sendCommand("AT+HTTPTERM");
  sendCommand("AT+SAPBR=0,1");

  Serial_print("Success!");
  
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







































