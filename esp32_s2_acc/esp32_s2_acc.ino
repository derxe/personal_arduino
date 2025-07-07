#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


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
    SetBestOffsets();
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);

    setupKalman(readAltitude());
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

#include <BasicLinearAlgebra.h>

// Kalman filter matrices and variables
BLA::Matrix<2,2> F;         // State transition matrix
BLA::Matrix<2,1> G;         // Control input matrix (for acceleration)
BLA::Matrix<2,2> P;         // Error covariance matrix
BLA::Matrix<2,2> Q;         // Process noise covariance
BLA::Matrix<2,1> S;         // State vector [altitude; velocity]
BLA::Matrix<1,2> H;         // Measurement matrix (altitude measurement)
BLA::Matrix<2,2> I;         // Identity matrix
BLA::Matrix<2,1> K;         // Kalman gain

BLA::Matrix<1,1> R;         // Measurement noise covariance (altimeter)
BLA::Matrix<1,1> L;         // Innovation (measurement residual)
BLA::Matrix<1,1> M;         // Innovation covariance

// Global variables for measurements
float altitude_measurement = 0.0;
float acceleration_measurement = 0.0;
float dt = 0.03;            // Time step in seconds (adjust as needed)

// Loop timing variables
unsigned long loopTimer = 0;
const unsigned long loopDuration = 30000;  // Loop period in microseconds (e.g., 30ms)


//
// Initialize all Kalman filter matrices and variables
//
void setupKalman(float startAlt) {
  // State transition matrix F
  F = {1, dt,
       0, 1};

  // Control input matrix G (for acceleration)
  G = {0.5f * dt * dt,
       dt};

  // Initial state estimate (altitude and velocity)
  S = {startAlt,
       0};

  // Initial error covariance matrix P
  P = {1, 0,
       0, 1};

  // Process noise covariance Q (tune these values)
  Q = {0.01, 0,
       0,    0.03};

  // Measurement matrix H (measuring altitude only)
  H = {1, 0};

  // Identity matrix
  I = {1, 0,
       0, 1};

  // Measurement noise covariance R (altimeter noise)
  R = {1};
}


//
// Perform one Kalman filter iteration (prediction + update)
//
void kalman_3d() {
  // -------- Prediction Step --------
  // Predict the next state using the acceleration as control input
  // S_pred = F * S + G * acceleration_measurement
  BLA::Matrix<2,1> S_pred = F * S + G * acceleration_measurement;
  
  // Predict the error covariance
  // P_pred = F * P * F^T + Q
  BLA::Matrix<2,2> P_pred = F * P * ~F + Q;
  
  // -------- Measurement Update --------
  // Create measurement vector z (here, only altitude is measured)
  BLA::Matrix<1,1> z;
  z(0,0) = altitude_measurement;
  
  // Compute the innovation (measurement residual): L = z - H * S_pred
  L = z - (H * S_pred);
  
  // Compute the innovation covariance: M = H * P_pred * H^T + R
  M = H * P_pred * ~H + R;
  
  // For a 1x1 matrix, the inverse is simply 1 divided by its element
  BLA::Matrix<1,1> invM;
  invM(0,0) = 1.0 / M(0,0);
  
  // Compute the Kalman gain: K = P_pred * H^T * inv(M)
  K = P_pred * ~H * invM;
  
  // Update the state estimate: S = S_pred + K * L
  S = S_pred + K * L;
  
  // Update the error covariance: P = (I - K * H) * P_pred
  P = (I - K * H) * P_pred;
}


//
// Main loop: read sensors, update the Kalman filter, and print the results
//
void loop() {
  getAcc(); 
  return; 
  
  long measureStart = micros();
  
  // Read sensors (replace these with your actual sensor functions)
  altitude_measurement = readAltitude();
  acceleration_measurement = getAcc();
  
  long measureEnd = micros();
  
  // Run the Kalman filter update
  kalman_3d();
  
  // Print raw sensor data and filtered estimates
  Serial.print("Raw Altitude: ");
  Serial.print(altitude_measurement);
  Serial.print(" m, ");
  
  Serial.print("Raw_Acceleration: ");
  Serial.print(acceleration_measurement);
  Serial.print(" m/s^2, ");
  
  Serial.print("Filtered_Altitude: ");
  Serial.print(S(0,0));
  Serial.print(" m, ");
  
  Serial.print("Filtered_Velocity: ");
  Serial.print(S(1,0));
  Serial.print(" m/s, ");
  
  Serial.print("Computation_Time: ");
  Serial.print(measureEnd - measureStart);
  Serial.print(" us");

  Serial.println();
  
  // Wait until the next loop cycle to maintain consistent timing
  while (micros() - loopTimer < loopDuration);
  loopTimer = micros();
}
