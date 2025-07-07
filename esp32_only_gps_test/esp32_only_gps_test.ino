#include <SoftwareSerial.h>

#include <TinyGPS.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial ss(5, 4);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 7, 6);
  ss.begin(9600);
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();

  Serial1.print("Simple TinyGPS library v. "); Serial1.println(TinyGPS::library_version());
  Serial1.println("by Mikal Hart");
  Serial1.println();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      Serial1.write(c);
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial1.print("LAT=");
    Serial1.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial1.print(" LON=");
    Serial1.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial1.print(" SAT=");
    Serial1.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial1.print(" PREC=");
    Serial1.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial1.print(" CHARS=");
  Serial1.print(chars);
  Serial1.print(" SENTENCES=");
  Serial1.print(sentences);
  Serial1.print(" CSUM ERR=");
  Serial1.println(failed);
  if (chars == 0)
    Serial1.println("** No characters received from GPS: check wiring **");
}