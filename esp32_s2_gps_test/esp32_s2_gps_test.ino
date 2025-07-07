#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <basicMPU6050.h> 

// Create instance
basicMPU6050<> imu;

/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
// The serial connection to the GPS module
#define TX 3 // ORANGE GPS-RX
#define RX 2 // YELLOW GPS-TX
SoftwareSerial ss(RX, TX);

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

#include <BMP280_DEV.h> 

BMP280_DEV bmp280;    

// Function to update sea level pressure based on current altitude (in meters)
void updateSeaLevelPressure(float currentAltitude, float currentPressure) {
  // Calculate sea level pressure using the barometric formula:
  // Sea Level Pressure (P0) = P / (1 - h/44330)^5.255
  float seaLevelPressure = currentPressure / pow(1 - (currentAltitude / 44330.0), 5.255);

  Serial.print("Sea level pressure: ");
  Serial.println(seaLevelPressure);
  // Pass the calculated sea level pressure to the BMP280 object
  bmp280.setSeaLevelPressure(seaLevelPressure);
}


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  ss.begin(GPSBaud);

  

  Serial.println("START?!?!!?");

  imu.setup();
  Serial.println("basicMPU6050 init finish");
  
  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X8);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_4);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startForcedConversion();

  float temperature, pressure, altitude;
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  
      Serial.print(pressure);
    Serial.println(";");
  updateSeaLevelPressure(530, pressure);
  Serial.println("bmp280 init finish");
}


float readAltitude() {
  bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if in SLEEP_MODE)

  float temperature, pressure, altitude;
  // Wait until the measurement is complete
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {
    
    }  

  return altitude;
}


long lastPing = millis();
void loop()
{

  long now = millis();
  if(now - lastPing > 20) {
    bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if in SLEEP_MODE)
    float temperature, pressure, altitude;
    // Wait until the measurement is complete
    while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  

    Serial.print("alt:");
    Serial.print(altitude);
    Serial.println("");

    lastPing = now;

    return;

      // Update gyro calibration 
      imu.updateBias();
     
      //-- Scaled and calibrated output:
      // Accel
      Serial.print( imu.ax() );
      Serial.print( " " );
      Serial.print( imu.ay() );
      Serial.print( " " );
      Serial.print( imu.az() );
      Serial.print( "    " );

      /*
      // Gyro
      Serial.print( imu.gx() );
      Serial.print( " " );
      Serial.print( imu.gy() );
      Serial.print( " " );
      Serial.print( imu.gz() );
      Serial.print( "    " );  
      
      // Temp
      Serial.print( imu.temp() );
      Serial.println(); 
      */
  }

  return;
  
  //Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  //Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
