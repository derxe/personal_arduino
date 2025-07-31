#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <GyverINA.h>
#include "Button2.h"


#define BUTTON_A_PIN  16
#define BUTTON_B_PIN  17

Button2 buttonA, buttonB;

/*
   Attention!!! The shunt voltage measurement limits of the INA226 are ±81.92 mV.
   When using the INA226 module with a 0.1 Ω shunt, the maximum measurable current will be about I ~ 820 mA.
   When using a different shunt, it's recommended to select it so that the voltage drop does not exceed 82 mV!

   Example:
   Max expected current = 5 A
   Voltage drop limit across the shunt = 80 mV
   Shunt resistance = 0.08 V / 5 A = 0.016 Ω
   The shunt should have a resistance of 0.016 Ω (16 mΩ)
*/

// Create object: INA226 ina(shunt resistance, max expected current, I2C address);
// INA226 ina(0x41);              // Default settings for the module, but with a custom address
// INA226 ina(0.05f);             // Default address and max current, but different shunt (0.05 Ω)
// INA226 ina(0.05f, 1.5f);       // Default address, but different shunt (0.05 Ω) and max expected current (1.5 A)
// INA226 ina(0.05f, 1.5f, 0x41); // Fully customizable version, manual parameter specification
INA226 ina;                       // Default parameter set for Arduino module (0.1, 0.8, 0x40)

// OLED display TWI address
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(-1);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void tapA(Button2& btn) {
  ina.adjCalibration(1);

  Serial.print("Calib: ");
  Serial.print(ina.getCalibration());
  Serial.println("inc 1");
}

void tapB(Button2& btn) {
  ina.adjCalibration(-1);

    Serial.print("Calib: ");
  Serial.print(ina.getCalibration());
  Serial.println("dec 1");
}

String formatFloat(float value) {
  char buffer[32]; // large enough for most use cases
  // Format into buffer
  if (abs(value) < 0.001) {
    sprintf(buffer, "%7.0f u", value*1000*1000);
  } else if (abs(value) < 1) {
    sprintf(buffer, "%7.1f m", value*1000);
  } else {
    sprintf(buffer, "%7.2f ", value);
  }
  

  return String(buffer);
}

#define SDA_PIN 2
#define SCL_PIN 4

void setup() {
  Serial.begin(115200);
  Serial.println("Start!");

  Wire.begin(SDA_PIN, SCL_PIN);
  
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(tapA);

  buttonB.begin(BUTTON_B_PIN);
  buttonB.setTapHandler(tapB);

  // initialize and clear display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.println("begin done!");
  display.clearDisplay();
  display.display();

  Serial.println("init done!");
  display.setTextWrap(false);

    // Open the serial port
  Serial.begin(115200);
  Serial.print(F("INA226..."));

  while (!ina.begin()) {
    Serial.println(F("INA226 not found, retrying..."));
    delay(1000); // Wait 1 second before retrying
  }

  Serial.println(F("INA226 connected!"));
  
  ina.setCalibration(1905);
  Serial.print(F("Calibration value: ")); Serial.println(ina.getCalibration());
  ina.setSampleTime(INA226_VBUS, INA226_CONV_2116US);   // Increase voltage sample time by 2×
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_8244US); // Increase current sample time by 8×
  ina.setAveraging(INA226_AVG_X16); // Enable built-in 4× averaging, default is no averaging

  Serial.println("");
}

String formatTime(unsigned long timeMillis) {
  char buffer[16]; // Format: HH:MM:SS.s → max 12 chars needed

  unsigned long totalSeconds = timeMillis / 1000;
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds / 60) % 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long tenths = (timeMillis % 1000) / 100;  // get first decimal digit

  sprintf(buffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);

  return String(buffer);
}

float voltage = 0;
float current = 0;
float power = 0;

unsigned long countStart = 0;
uint64_t nAh = 0;
uint64_t nWh = 0;

void read() {
  static unsigned long lastRead = 0;
  unsigned long now = millis();
  unsigned long diff = now - lastRead;
  lastRead = now;

  if (diff == 0) return;

  current = ina.getCurrent(); // in A
  voltage = ina.getVoltage(); // in V
  power = ina.getPower();     // in W

  // µAh = current [A] * ms / 3600 (convert to µAh directly)
  nAh += current * diff * 277;  // or 1000.0 / 3600000.0
  nWh += power   * diff * 277;
}



void loop() {
  buttonA.loop();
  buttonB.loop();

  static unsigned long lastPrint = 0;

  if(millis() - lastPrint > 100) {
    lastPrint = millis();
    read();
    // Read voltage
    Serial.print(F("Voltage: "));
    Serial.print(voltage, 4);
    Serial.print(F(" V "));

    // Read current
    Serial.print(F("Current: "));
    Serial.print(current, 5);
    Serial.print(F(" A "));

    Serial.print(F("Power: "));
    Serial.print(power, 5);
    Serial.print(F(" W "));

    Serial.print(F("Energy: "));
    Serial.print(nWh/1000);
    Serial.print(F(" uWh "));

    Serial.print(F("uAh: "));
    Serial.print(nAh/1000);
    Serial.println(F(" uAh"));

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.print(formatFloat(current) + "A");

    display.setCursor(0,19);
    display.print(formatFloat(voltage) + "V");

    display.setCursor(0,38);
    display.print(formatFloat(power) + "W");

    display.setTextSize(1);
    display.setCursor(0,57);
    display.print(formatFloat(nWh/1000000000.0) + "Wh " + formatTime(millis()-countStart));

    display.display();
  }
}
