// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/
/*
#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;

void sensor_read() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  Serial.print("x:");
  Serial.print(AccX);
  Serial.print(", y:");
  Serial.print(AccY);
  Serial.print(", z:");
  Serial.println(AccZ);
  
  //AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  //AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);  
  
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  
}


void loop() {
  
  sensor_read();
  delay(100);
}

*/
/*
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(10000);
  
}

void loop() {
  // Get new sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print out the values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}
*/

// This sketch shows how to configure the output

#include <basicMPU6050.h> 

//-- Input parameters:

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH

//Your offsets:  -4194 -1696 520 60  -20 54
//Data is printed as: acelX acelY acelZ giroX giroY giroZ

// Accelerometer offset:
//constexpr int   AX_OFFSET = -4194;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
//constexpr int   AY_OFFSET = -1696;       // These values are unlikely to be zero.
//constexpr int   AZ_OFFSET = 520;

constexpr int   AX_OFFSET =  520;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -1696;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -4194;

// Output scale: 
constexpr float AX_SCALE = 1.0000;     // Multiplier for accelerometer outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float AY_SCALE = 1.0000;
constexpr float AZ_SCALE = 1.0000;

constexpr float GX_SCALE = 1.0;     // Multiplier to gyro outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float GY_SCALE = 1.0;
constexpr float GZ_SCALE = 1.0;

// Bias estimate:
#define         GYRO_BAND   35          // Standard deviation of the gyro signal. Gyro signals within this band (relative to the mean) are suppresed.   
#define         BIAS_COUNT  5000        // Samples of the mean of the gyro signal. Larger values provide better calibration but delay suppression response. 

//-- Set the template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu;

void setup() {
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  // Start console
  Serial.begin(38400);
}

void loop() { 
  // Update gyro calibration 
  imu.updateBias();
 
  //-- Scaled and calibrated output:
  // Accel
  Serial.print("ax:");
  Serial.print(imu.rawAx()/4096.0 *2);
  Serial.print(", ay:" );
  Serial.print(imu.rawAy()/4096.0 *2);
  Serial.print(", az:" );
  Serial.print(imu.rawAz()/4096.0 * 2 );
  Serial.println("" );
  delay(10);
  return;
 
  // Gyro
  Serial.print( imu.gx() );
  Serial.print( " " );
  Serial.print( imu.gy() );
  Serial.print( " " );
  Serial.print( imu.gz() ); 
  Serial.println();
} 
