#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


MPU6050 accelgyro;


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

void intiBmp280() {
  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X8);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_4);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startForcedConversion();

  float pressure = readPressure();
  Serial.print(pressure);
  Serial.println(";");
  
  updateSeaLevelPressure(530, pressure);
  Serial.println("bmp280 init finish");  
}

// Function to update sea level pressure based on current altitude (in meters)
void updateSeaLevelPressure(float currentAltitude, float currentPressure) {
  float seaLevelPressure = currentPressure / pow(1 - (currentAltitude / 44330.0), 5.255);
  bmp280.setSeaLevelPressure(seaLevelPressure);
}

float readPressure() {
  bmp280.startForcedConversion();             
  float temperature, pressure, altitude;
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  
  return pressure;
}

float readAltitude() {
  bmp280.startForcedConversion();            
  float temperature, pressure, altitude;
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  
  return altitude;
}

#include "altitude.h"

// Altitude estimator
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, // sigma Accel
        0.0005, // sigma Gyro
        0.018,   // sigma Baro
        0.5, // ca
        0.1);// accelThreshold
        
float startAlt; 

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

    intiBmp280();

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    SetBestOffsets();
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

    float altSum = 0;
    for(int i=0;i<10;i++){
        altSum += readAltitude();
    }
    startAlt = altSum / 10;
    
    pinMode(LED_PIN, OUTPUT);
}


float acc[3];
float gyro[3];
void readMPU650() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float deviderAcc = 16384 >> accelgyro.getFullScaleAccelRange();
    
    acc[0] = ax/deviderAcc;
    acc[1] = ay/deviderAcc;
    acc[2] = az/deviderAcc;

    float deviderGyro = 131 >> accelgyro.getFullScaleGyroRange();
    gyro[0] = gx/deviderGyro / 10;
    gyro[1] = gy/deviderGyro / 10;
    gyro[2] = gz/deviderGyro / 10;
}

// Global variables for measurements
float altitude_measurement = 0.0;
float acceleration_measurement = 0.0;
float dt = 0.03;            // Time step in seconds (adjust as needed)

// Loop timing variables
unsigned long loopTimer = 0;
const unsigned long loopDuration = 30000;  // Loop period in microseconds (e.g., 30ms)


//
// Main loop: read sensors, update the Kalman filter, and print the results
//
void loop() {
  long measureStart = micros();
  float alt = readAltitude() - startAlt;
  readMPU650();
  long measureEnd = micros();

  altitude.estimate(acc, gyro, alt, micros());

  Serial.print("alt:"); Serial.print(altitude.getAltitude()); Serial.print(" ");
  Serial.print("var:"); Serial.print(altitude.getVerticalVelocity()); Serial.print(" ");
  Serial.print("acc:"); Serial.print(altitude.getVerticalAcceleration()); Serial.print(" ");

  Serial.print("ax:"); Serial.print(acc[0]); Serial.print(", ");
  Serial.print("ay:"); Serial.print(acc[1]); Serial.print(", ");
  Serial.print("az:"); Serial.print(acc[2]); Serial.print(", ");

  Serial.print("gx:"); Serial.print(gyro[0]); Serial.print(", ");
  Serial.print("gy:"); Serial.print(gyro[1]); Serial.print(", ");
  Serial.print("gz:"); Serial.print(gyro[2]); Serial.print(" ");
 
  Serial.print("balt:"); Serial.print(alt); Serial.print(" ");


  
  //Serial.print("dt: "); Serial.print(measureEnd - measureStart); Serial.print(" us");

  Serial.println();
  
  // Wait until the next loop cycle to maintain consistent timing
  while (micros() - loopTimer < loopDuration);
  loopTimer = micros();
}
