#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define RAD_TO_DEG (180 / 3.1315)

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#include <BMP280_DEV.h> 
BMP280_DEV bmp280;    

#define LED_PIN 13
bool blinkState = false;

void SetBestOffsets() {
    accelgyro.setXAccelOffset(  -4133 );
    accelgyro.setYAccelOffset( -1706 );
    accelgyro.setZAccelOffset(  527 );
    accelgyro.setXGyroOffset(   58 );
    accelgyro.setYGyroOffset(   -20 );
    accelgyro.setZGyroOffset(    49 );
}

void printOffsets() {
      Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0  
}

void printDLPF() {
  uint8_t dlpf = accelgyro.getDLPFMode();

  Serial.print("DLPF Mode: ");
  switch (dlpf) {
    case MPU6050_DLPF_BW_256:
      Serial.println("256 Hz bandwidth (delay ~0.98ms)");
      break;
    case MPU6050_DLPF_BW_188:
      Serial.println("188 Hz bandwidth (delay ~1.9ms)");
      break;
    case MPU6050_DLPF_BW_98:
      Serial.println("98 Hz bandwidth (delay ~2.8ms)");
      break;
    case MPU6050_DLPF_BW_42:
      Serial.println("42 Hz bandwidth (delay ~4.8ms)");
      break;
    case MPU6050_DLPF_BW_20:
      Serial.println("20 Hz bandwidth (delay ~8.3ms)");
      break;
    case MPU6050_DLPF_BW_10:
      Serial.println("10 Hz bandwidth (delay ~13.4ms)");
      break;
    case MPU6050_DLPF_BW_5:
      Serial.println("5 Hz bandwidth (delay ~18.6ms)");
      break;
    default:
      Serial.println("Unknown DLPF mode");
      break;
  }  
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
    while(!Serial);

    accelgyro.initialize();
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    SetBestOffsets();
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    pinMode(LED_PIN, OUTPUT);
}

float getAcc() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float devider = 16384 >> accelgyro.getFullScaleAccelRange();
    float accX = ax / devider;
    float accY = ay / devider;
    float accZ = az / devider;

    float roll = atan2(accY, accZ);
    float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
    Serial.print(""); Serial.print(roll * 100); Serial.print("/");
    Serial.print(""); Serial.print(pitch * 100); Serial.print("/");

    //Serial.print(""); Serial.print(accX); Serial.print("/");
    //Serial.print(""); Serial.print(accY); Serial.print("/");
    //Serial.print(""); Serial.print(accZ); Serial.println("");

    // Normalize the acc vector
    float magnitude = sqrt(accX * accX + accY * accY + accZ * accZ);
    float normX = accX / magnitude;
    float normY = accY / magnitude;
    float normZ = accZ / magnitude;

    // Print normalized values
    Serial.print(""); Serial.print(normX); Serial.print("/");
    Serial.print(""); Serial.print(normY); Serial.print("/");
    Serial.print(""); Serial.print(normZ); Serial.println("");

    float inertialZ = accX * (-sin(pitch))
                    + accY * (sin(roll) * cos(pitch))
                    + accZ * (cos(roll) * cos(pitch));
    return (inertialZ - 1) * 9.80;
}

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

unsigned long loopTimer = 0;
const unsigned long loopDuration = 3000;  // Loop period in microseconds (e.g., 30ms)



void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.04*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

//
// Main loop: read sensors, update the Kalman filter, and print the results
//
void loop() {
  while (micros() - loopTimer < loopDuration);
  loopTimer=micros();

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float devider = 16384 >> accelgyro.getFullScaleAccelRange();
  float accX = ax / devider;
  float accY = ay / devider;
  float accZ = az / devider;
  AngleRoll = atan2(accY, accZ);
  AnglePitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));

  AngleRoll = AngleRoll * RAD_TO_DEG;
  AnglePitch = AnglePitch * RAD_TO_DEG;

  float AccX = accX;
  float AccY = accY;
  float AccZ = accZ;
  
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

  float deviderGyro = 131 >> accelgyro.getFullScaleGyroRange();
  RateRoll = gx/deviderGyro;
  RatePitch = gy/deviderGyro;
  RateYaw = gz/deviderGyro;

  //Serial.print("roll:"); Serial.print(AngleRoll); Serial.print(" ");
  //Serial.print("pitch:"); Serial.print(AnglePitch); Serial.print(" ");

  //Serial.print("roll:"); Serial.print(RateRoll); Serial.print(" ");
  //Serial.print("pitch:"); Serial.print(RatePitch); Serial.print(" ");
  //Serial.print("yaw:"); Serial.print(RateYaw); Serial.print(" ");
  //Serial.println();

  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print("roll:");
  Serial.print(KalmanAngleRoll);
  Serial.print(" ");
  Serial.print("pitch:");
  Serial.println(KalmanAnglePitch);
  

}
