//
//    FILE: AS5600_position.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include "TLx493D_inc.hpp"

using namespace ifx::tlx493d;

TLx493D_A1B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);

#define SDA_PIN             4
#define SCL_PIN             3      
#define POWER_PIN           5

#define UART_TX_PIN  39
#define UART_RX_PIN  37 

static bool readLowPowerSample(double *x, double *y, double *z, double *t) {
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    dut.getMagneticFieldAndTemperature(x, y, z, t);
    if (dut.hasValidData()) return true;
    delayMicroseconds(500);
  }

  return false;
}

void setup()   
{
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.begin(115200);
  Serial.print("Hello world");
  Serial1.println(__FILE__);
  Serial1.println();

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);


  //Wire.begin();
  Wire1.begin(SDA_PIN, SCL_PIN, 4000000);

  dut.begin();
  dut.setPowerMode(TLx493D_LOW_POWER_MODE_e);

  delay(1000);
}

float calcAngle(float x, float y, float z) {
    (void)y; // ignore y

    float angle = atan2f(z, x);
    angle /= 2.0f * (float)M_PI;

    if (angle < 0.0f)
        angle += 1.0f;

    return angle;
}

float calcMagnetPower(float x, float y, float z) {
    (void)y; // ignore y
    return sqrtf(x * x + z * z);
}

float dir;
float power;

void loop()
{
  static uint32_t lastTime = 0;
  static float prevDir = -1;
  static float total_rotation = 0;

  //  set initial position
  //as5600.getCumulativePosition();W

  static int nSamples = 0;
  static int nSamplesPrint = 0;
  static int prevrotations = 0;
  static float speed = 0;
  static float prevtotal_rotation = 0;
  static uint32_t lastTimeSpeed = 0;
  static uint32_t lastTimePrint = 0;
  if (millis() - lastTimeSpeed >= 1000)
  {
    speed = total_rotation - prevtotal_rotation;
    prevtotal_rotation = total_rotation;
    lastTimeSpeed = millis();

       
    Serial1.printf("dir:%3d total_rotation:%5.2f power:%0.4f speed:%4.1f n_samples:%5d\r\n", (int)(dir*360), total_rotation, power, speed,nSamplesPrint);
    Serial.printf( "dir:%3d total_rotation:%5.2f power:%0.4f speed:%4.1f n_samples:%d\r\n", (int)(dir*360), total_rotation, power, speed, nSamplesPrint);
  }


  //  update every 100 ms
  //  should be enough up to ~200 RPM
  if (millis() - lastTime >= 20)
  {
    lastTime = millis();
    double t, x, y, z;

    uint32_t startTime = micros();
    if (!readLowPowerSample(&x, &y, &z, &t)) return;
    uint32_t endTime = micros();
      

    // print the communication time with the sensor every 100 samples
    if(millis() - lastTimePrint >= 500) {
       Serial.printf("Communication time: %d us\r\n", endTime - startTime);
       Serial1.printf("Communication time: %d us\r\n", endTime - startTime);
       lastTimePrint = millis();
    }
    

    dir = calcAngle(x, y, z);
    power = calcMagnetPower(x, y, z);
    if(power < 5) return; // not enough power to calc angle
    nSamples ++;

    if(prevDir == -1) prevDir = dir;

    float delta = dir - prevDir;
    prevDir = dir;

    // handle wrap-around
    if (delta < -0.5) delta += 1.0;
    if (delta > 0.5)  delta -= 1.0;

    total_rotation += delta;

    int rotations = (int) total_rotation;
    if(prevrotations != rotations) {
      nSamplesPrint = nSamples;
      nSamples = 0;
      prevrotations = rotations;
    }    

    //Serial1.printf("dir:%3d total_rotation:%5.2f power:%0.4f\r\n", (int)(dir*360), total_rotation, power);
    //Serial.printf( "dir:%3d total_rotation:%5.2f power:%0.4f\r\n", (int)(dir*360), total_rotation, power);
   }
}


//  -- END OF FILE --
