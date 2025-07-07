/////////////////////////////////////////////////////////////////////////////////
// BMP280_DEV - I2C Communications, Forced Conversion with Timing Measurement
/////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(5, 4);

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//#include <VoltageReference.h>
//VoltageReference vRef;

unsigned long startTime, endTime, elapsedTime;    // Variables for timing measurements

const int BATTERY_PIN = 2; // Replace with your GPIO
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Because of 100k:100k

float getBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  //Serial1.print("raw:");
  //Serial1.println(raw);
  float voltage = (raw * 5.05) / 2640.0;
  //Serial1.print("voltage:");
  //Serial1.println(voltage);
  return voltage;
}

void setup() 
{
  Serial.begin(115200);                           // Initialize the serial port
  Serial1.begin(115200, SERIAL_8N1, 7, 6);
  ss.begin(9600);
  Serial1.println("Over 8, IRR 16");

  analogReadResolution(12); // Optional, default is 12-bit (0–4095)
  analogSetAttenuation(ADC_11db); // Measure up to ~3.3V input
  //Voltage Measurement
  //vRef.begin();

  // initialize and clear display
  #define SDA 8
  #define SCL 9
  Wire.begin(SDA, SCL);
  //Wire.setClock(20*1000); 
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial1.println("display init");
  Serial.println("display init");
  display.clearDisplay();
  display.display();
  Serial1.println("init done!");

  //setCpuFrequencyMhz(80);
}

void updateGPS() {
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

  float flat = -1;
  int nSetalites = -1;
  if (newData)
  {
    float flon;
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

    nSetalites = gps.satellites();
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

  refresh_display(nSetalites, flat, chars, sentences);
}

unsigned long lastDisplay = 0;
int br=1;
void loop() 
{
  updateGPS();
}


void refresh_display(float nSetalites, float alt, int nChars, int nSentences){
  display.fillScreen(0);
  display.setTextColor(1);
  display.setTextSize(2);
  int yPos = 0; 
  display.setCursor(0,yPos);
  display.print(nSetalites);

  display.setCursor(0,yPos+=15);
  display.print(alt);

  display.setCursor(0,yPos+=15);
  display.print(nChars);

  display.setCursor(0,yPos+=15);
  display.print(nSentences);

  display.setCursor(80,yPos);
  display.print(getBatteryVoltage());

  display.display();
  delay(50);
}
