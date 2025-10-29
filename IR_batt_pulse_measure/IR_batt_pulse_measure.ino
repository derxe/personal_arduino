#include <Wire.h>
#include <GyverINA.h>
#include "Button2.h"


struct AvgReading {
  float current;  // in A
  float voltage;  // in V
};

#define BUTTON_A_PIN  22
#define BUTTON_B_PIN  23

#define MOS_PIN  18

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


#define SHUNT_RESISTOR 0.00719f // at 3 amps we measured roughly 0.0215mV on shunt so the resistor must be around 0.00719 ohms
#define MAX_SHUNT_V    0.082f // V 
#define MAX_CURRENT    MAX_SHUNT_V / SHUNT_RESISTOR
INA226 ina(SHUNT_RESISTOR, MAX_CURRENT, 0x40);            // Default address and max current, but different shunt (0.05 Ω)
// INA226 ina(0.05f, 1.5f);       // Default address, but different shunt (0.05 Ω) and max expected current (1.5 A)
// INA226 ina(0.05f, 1.5f, 0x41); // Fully customizable version, manual parameter specification
//INA226 ina;                       // Default parameter set for Arduino module (0.1, 0.8, 0x40)

void tapAa(Button2& btn) {
  ina.adjCalibration(1);

  Serial.print("Calib: ");
  Serial.print(ina.getCalibration());
  Serial.println(" inc 1");
}
 

 


bool printEnabled = true;



void tapA(Button2& btn) {
  printEnabled = !printEnabled;
}



void tapBa(Button2& btn) {
  ina.adjCalibration(-1);

    Serial.print("Calib: ");
  Serial.print(ina.getCalibration());
  Serial.println("dec 1");
}



float voltage = 0;
float current = 0;
float power = 0;


AvgReading readAverage(int samples) {
  float sumCurrent = 0.0f;
  float sumVoltage = 0.0f;

  for (int i = 0; i < samples; i++) {
    float current = ina.getCurrent();   // in A
    float voltage = ina.getVoltage();   // in V
    sumCurrent += current;
    sumVoltage += voltage;
  }

  AvgReading result;
  result.current = sumCurrent / samples;
  result.voltage = sumVoltage / samples;
  return result;
}


float oneIntervalMeasure() {
  AvgReading avgPre = readAverage(5);

  pinMode(MOS_PIN, OUTPUT);

  uint32_t onStart = micros();
  digitalWrite(MOS_PIN, LOW); // on
  delay(1); // wait a bit for current and voltage to stabilise

  AvgReading avgDur = readAverage(10);

  pinMode(MOS_PIN, INPUT); // off
  Serial.print((micros()-onStart)/1000.0, 2); // on duration
  Serial.print("; "); Serial.print(avgPre.voltage, 3); Serial.print(" - "); Serial.print(avgDur.voltage, 3); 
  Serial.print(" = "); Serial.print(abs(avgPre.voltage - avgDur.voltage), 3); Serial.print(" V");
  Serial.print("  ");
  Serial.print(""); Serial.print(avgPre.current, 3); Serial.print(" - "); Serial.print(avgDur.current, 3); 
  Serial.print(" = "); Serial.print(abs(avgPre.current - avgDur.current), 3); Serial.print(" A");

  float ir = abs(avgPre.voltage - avgDur.voltage) / abs(avgPre.current - avgDur.current);
  Serial.print(" => "); Serial.print(ir * 1000, 1); Serial.print(" mOhm");
  Serial.println();

  return ir;
}
  


void tapB(Button2& btn) {
  float sumResistance = 0;
  int nSamples = 5;
  for(int i=0; i<nSamples; i++) {
    sumResistance += oneIntervalMeasure();
    delay(100);
  }

  Serial.print("Avg Resistance: "); Serial.print(sumResistance * 1000 / nSamples); Serial.print(" mOhm");
  Serial.println();
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

#define SDA_PIN 17
#define SCL_PIN 16

void setup() {
  pinMode(MOS_PIN, INPUT); // off
  Serial.begin(115200);
  Serial.println("Start!");
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(tapA);

  buttonB.begin(BUTTON_B_PIN);
  buttonB.setTapHandler(tapB);

  // initialize and clear display
  Serial.println("begin done!");

  Serial.println("init done!");

    // Open the serial port
  Serial.begin(115200);
  Serial.print(F("INA226..."));

  while (!ina.begin()) {
    Serial.println(F("INA226 not found, retrying..."));
    delay(1000); // Wait 1 second before retrying
  }

  Serial.println(F("INA226 connected!"));
  
  //ina.setCalibration(2050);
  Serial.print(F("Calibration value: ")); Serial.println(ina.getCalibration());
  ina.setSampleTime(INA226_VBUS, INA226_CONV_140US);   // Increase voltage sample time by 2×
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_140US); // Increase current sample time by 8×
  ina.setAveraging(INA226_AVG_X1); // Enable built-in 4× averaging, default is no averaging

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


void loop() {
  buttonA.loop();
  buttonB.loop();

  static unsigned long lastPrint = 0;


  if(printEnabled && millis() - lastPrint > 500) {
    lastPrint = millis();
    //read();
    // Read voltage
    Serial.print(F("Voltage: "));
    Serial.print(ina.getVoltage(), 4);
    Serial.print(F(" V "));

    // Read current
    Serial.print(F("Current: "));
    Serial.print(ina.getCurrent()*1000, 1);
    Serial.print(F(" mA "));

    Serial.print(F("Power: "));
    Serial.print(ina.getPower(), 5);
    Serial.print(F(" W "));

    Serial.print(F("Shunt: "));
    Serial.print(ina.getShuntVoltage()*1000);
    Serial.print(F(" mV "));

    Serial.println();

  }
}































































