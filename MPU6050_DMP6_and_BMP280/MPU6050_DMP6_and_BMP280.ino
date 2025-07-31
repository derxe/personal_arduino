// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <BMP280_DEV.h> 
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#include "Wire.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "Button2.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu; // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
#define BUTTON_PIN 6
Button2 button;


#define INTERRUPT_PIN 1  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LED_PIN_RED 5
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void buttonClicked(Button2 &b) {
  toggleDataPrint();
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void SetBestOffsets() {
    mpu.setXAccelOffset( -4070 );
    mpu.setYAccelOffset( -1753 );
    mpu.setZAccelOffset(  543 );
    mpu.setXGyroOffset(   41 );
    mpu.setYGyroOffset(  -20 );
    mpu.setZGyroOffset(   35 );
}

void PrintFullScaleRanges() {
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getFullScaleAccelRange()) {
    case MPU6050_ACCEL_FS_2:
      Serial.println("±2G");
      break;
    case MPU6050_ACCEL_FS_4:
      Serial.println("±4G");
      break;
    case MPU6050_ACCEL_FS_8:
      Serial.println("±8G");
      break;
    case MPU6050_ACCEL_FS_16:
      Serial.println("±16G");
      break;
    default:
      Serial.println("Unknown");
      break;
  }

  Serial.print("Gyroscope range set to: ");
  switch (mpu.getFullScaleGyroRange()) {
    case MPU6050_GYRO_FS_250:
      Serial.println("±250 deg/s");
      break;
    case MPU6050_GYRO_FS_500:
      Serial.println("±500 deg/s");
      break;
    case MPU6050_GYRO_FS_1000:
      Serial.println("±1000 deg/s");
      break;
    case MPU6050_GYRO_FS_2000:
      Serial.println("±2000 deg/s");
      break;
    default:
      Serial.println("Unknown");
      break;
  }
}

void printDLPF() {
  uint8_t dlpf = mpu.getDLPFMode();

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

float temperature, pressure, altitude;            // Create the temperature, pressure and altitude variables
BMP280_DEV bmp280;      

void initBmp280() {
  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X16);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_8);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startNormalConversion(); 
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Wire.begin(11, 10);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  button.begin(BUTTON_PIN);
  button.setTapHandler(buttonClicked);

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  WiFi.mode(WIFI_STA);  // Optional, forces station mode
  WiFi.begin("kuhna-wifi", "");
  //WiFi.begin("tony-lenovo", "8uPj1P0d");

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  initBmp280();

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  SetBestOffsets();

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  PrintFullScaleRanges();
  printDLPF();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      Serial.print("PacketSize:"); Serial.println(packetSize);
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_RED, OUTPUT);
  toggleDataPrint();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

bool dataPrintEnabled = false;
unsigned long dataPrintStart = 0;
unsigned long printNumber = 0;

void toggleDataPrint() {
  dataPrintEnabled = !dataPrintEnabled;
  digitalWrite(LED_PIN_RED, dataPrintEnabled);
  digitalWrite(LED_PIN, dataPrintEnabled);
  dataPrintStart = millis();
  printNumber = 0;
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      toggleDataPrint();
    }
  }
}


WiFiUDP Udp;                             // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(192, 168, 0, 121);     // remote IP to receive OSC
//const IPAddress outIp(10, 42, 0, 1);  
const unsigned int outPort = 9999;          // remote port to receive OSC


void readUdp() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    char incoming[packetSize + 1];  // +1 for null terminator
    int len = Udp.read(incoming, packetSize);
    if (len > 0) {
      incoming[len] = '\0';  // null-terminate string
    }

    // Now process like Serial input
    for (int i = 0; i < len; ++i) {
      char c = incoming[i];
      if (c == '\n' || c == '\r') {
        toggleDataPrint();
      }
    }
  }
}

void printBmp280DataIfReady() {
  if (bmp280.getMeasurements(temperature, pressure, altitude)) {
    unsigned long now = millis();
    unsigned long timestamp = now - dataPrintStart;

    printNumber++;
    Serial.print(printNumber);
    Serial.print(":");
    Serial.print(timestamp);
    Serial.print(F(":bmp:"));
    Serial.print(pressure, 3);    
    Serial.print(F("\t"));
    Serial.print(altitude, 3);
    Serial.println(F("\t"));  

        // --- UDP output ---
    Udp.beginPacket(outIp, outPort);
    Udp.print(printNumber);
    Udp.print(":");
    Udp.print(timestamp);
    Udp.print(":bmp:");
    Udp.print(pressure, 3);
    Udp.print("\t");
    Udp.print(altitude, 3);
    Udp.print("\t\n");  // Matches Serial output
    Udp.endPacket();
  }
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("*** Disconnected from AP so rebooting ***");
    Serial.println();
    ESP.restart();
  }
  button.loop();
  readSerial();
  readUdp();

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if(dataPrintEnabled) {
    printBmp280DataIfReady();
  }
/*
  static unsigned long lastSend = 0;
  if(millis() - lastSend > 1000) {
    lastSend = millis();
    unsigned long duration = micros();
    Udp.beginPacket(outIp, outPort);
    Udp.print(1); Udp.print(","); 
    Udp.print(2); Udp.print(",");
    Udp.print(lastSend); Udp.print(",");
    Udp.print(232); Udp.print("\n");
    Udp.endPacket();
    Serial.print("Duration: "); Serial.println((micros() - duration));
  }
*/

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);

      if(dataPrintEnabled) {
          unsigned long timestamp = millis() - dataPrintStart;
           static bool blinkLedState = false;
          if(timestamp/1000 % 2 == blinkLedState) {
            blinkLedState = !blinkLedState;
            digitalWrite(LED_PIN_RED, blinkLedState);
          }

          printNumber++;
          Serial.print(printNumber);
          Serial.print(":");
          Serial.print(timestamp);
          Serial.print(":");

          Serial.print("quat:");
          Serial.print(q.w, 4);
          Serial.print("\t");
          Serial.print(q.x, 4);
          Serial.print("\t");
          Serial.print(q.y, 4);
          Serial.print("\t");
          Serial.print(q.z, 4);

          float devider = 16384 >> MPU6050_ACCEL_FS_8; // the DMP seems to have predifined scaling for acceleration set to +-8 
          Serial.print("\t:acc:");
          Serial.print(aa.x / devider, 4);
          Serial.print("\t");
          Serial.print(aa.y / devider, 4);
          Serial.print("\t");
          Serial.println(aa.z / devider, 4);

          Udp.beginPacket(outIp, outPort);
          Udp.print(printNumber);
          Udp.print(":");
          Udp.print(timestamp);
          Udp.print(":");

          Udp.print("quat:");
          Udp.print(q.w, 4);
          Udp.print("\t");
          Udp.print(q.x, 4);
          Udp.print("\t");
          Udp.print(q.y, 4);
          Udp.print("\t");
          Udp.print(q.z, 4);

          Udp.print("\t:acc:");
          Udp.print(aa.x / devider, 4);
          Udp.print("\t");
          Udp.print(aa.y / devider, 4);
          Udp.print("\t");
          Udp.print(aa.z / devider, 4);
          Udp.print("\n");
          Udp.endPacket();

                // blink LED to indicate activity

      }


  }
}

/*
#ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);

            Serial.print("\t:acc\t");
            Serial.print(aa.x);
            Serial.print("\t");
            Serial.print(aa.y);
            Serial.print("\t");
            Serial.println(aa.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

*/

