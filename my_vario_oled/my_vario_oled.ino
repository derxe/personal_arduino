/////////////////////////////////////////////////////////////////////////////////
// BMP280_DEV - I2C Communications, Forced Conversion with Timing Measurement
/////////////////////////////////////////////////////////////////////////////////

#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library
#include <Adafruit_SSD1306.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#include <VoltageReference.h>
VoltageReference vRef;


float last_temp, last_altitude;
BMP280_DEV bmp280;                                // Instantiate (create) a BMP280_DEV object for I2C operation (address 0x77)
#define buzzer 11   

#define SAMPLES  15
float alt[SAMPLES];                                                       
unsigned long tim[SAMPLES];  
int samples_index = 0;
unsigned long tempo = millis();             // Beschreibung einer Tempovariablen vom Typ float und Zuweisung eines Wertes mit der Funktion millis () - Zählt die Zeit ab Programmstart in Millisekunden

float beep;                                  
float Beep_period; 

#define VARIO_TRESH_UP 0.4
#define VARIO_TRESH_DOWN -3.0

unsigned long startTime, endTime, elapsedTime;    // Variables for timing measurements

void setup() 
{
  Serial.begin(115200);                           // Initialize the serial port
  Serial.println("Over 8, IRR 16");
  pinMode(buzzer, OUTPUT); 

  //Voltage Measurement
  vRef.begin();

  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X16);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_16);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startForcedConversion();

  
  initSamples();

  // initialize and clear display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println("display init");
  display.clearDisplay();
  display.display();
  Serial.println("init done!");

         
  
}

unsigned long lastDisplay = 0;
int br=1;
void loop() 
{
  //delay(10);                                 

  float vario = VarioMSCalculation();
  Serial.print("var ");
  Serial.println(vario);
  VarioBeep(vario);

  if (millis() - lastDisplay > 500) {
    lastDisplay = millis();
    //unsigned long calculationStart = micros();

    refresh_display(vario);
    //unsigned long calculationEnd = micros();
    //Serial.print("Duration:");
    //Serial.println((calculationEnd - calculationStart)/1000.0);
  }
}


void refresh_display(float vario){
  char buffer[10];
  
  int bg_color = vario > 0.1? 1 : 0;
  int txt_color = !bg_color;
  display.fillScreen(bg_color);
  

  display.setTextColor(txt_color);
  display.setTextSize(2);
  display.setCursor(0,2);
  dtostrf(abs(last_altitude), 6, 1, buffer);
  display.print(buffer);

  display.setCursor(73,2);
  display.print("m");
  
  display.setTextSize(4);
  display.setCursor(0,21);

  display.print(vario > 0? "+" : "-");

  dtostrf(abs(vario), 3, 1, buffer);
  display.setCursor(30,21);
  display.print(buffer);

  int vcc = vRef.readVcc();
  #define BATERY_MAX 4200
  #define BATERY_MIN 3500

  int battery_proc = ((vcc - BATERY_MIN)*100) / (BATERY_MAX - BATERY_MIN);
  //battery_proc = constrain(battery_proc, 150, -1);
  Serial.print("vcc:");
  Serial.println(vcc);

  Serial.print("proc:");
  Serial.println(battery_proc);

  display.setTextSize(1);
  display.setCursor(0,56);
  display.print(vcc);

  display.setCursor(30,56);
  display.print(battery_proc);

  display.fillRect(128-40, 64-8, 40, 8, bg_color);

  display.fillRect(128-20, 0, 20, 64, 0);
  display.drawRect(128-20, 0, 20, 64, 1);

  int bar_height = abs(vario) * 20;
  int y = vario>0? 64/2 - bar_height : 64/2;
  display.fillRect(128-19, y, 18, bar_height, 1);
  
  display.display();
  delay(50);
}

void VarioBeep(float vario){


    if ((tempo - beep) > Beep_period + 100)                         
  {                                                                
    beep=tempo;                                                    
    if (vario > VARIO_TRESH_UP && vario<15)                                
    {                                                              
      
      Beep_period = 400-(vario*100);

      Serial.print(vario);
      Serial.print(" ");
      Serial.print(vario);
      Serial.print(" ");
      
      Serial.print("Rise ");
      Serial.println(Beep_period);
      
      tone( buzzer , (300 + (250 * vario)), abs(Beep_period));
    }                                             
    else if (vario < VARIO_TRESH_DOWN)                      // Sinking Tone
    {                                              
      Beep_period=200;  
            Serial.print(vario);
      Serial.print(" ");   
      Serial.println("Sink");                                      
      tone(buzzer,200,400);                                    
    }                                                          
  }           
  
}

float readAltitude() {
  bmp280.startForcedConversion();                 // Start BMP280 forced conversion (if in SLEEP_MODE)

  float temperature, pressure, altitude;
  // Wait until the measurement is complete
  while (!bmp280.getMeasurements(temperature, pressure, altitude)) {}  

  last_temp = temperature;
  last_altitude = altitude;
  return altitude;
}


void initSamples() {
  float altitude = readAltitude();
  float tempo = millis();
  for(int cc=0; cc<SAMPLES; cc++) {   
    alt[cc] = readAltitude();                                 
    tim[cc] = tempo;               
  }  
}

// Variofunktion FROM ANDREI's Project https://www.instructables.com/id/DIY-Arduino-Variometer-for-Paragliding
float VarioMSCalculation(){
  tempo = millis();         

  // record a new sample   
  TWBR = 2;    
  Serial.print(samples_index);                                                  
  alt[samples_index] = readAltitude();                                 
  tim[samples_index] = tempo;                                             
 
  unsigned long stime = tim[(samples_index+1)%SAMPLES];      // oldest sample                          
  float N1 = 0;                                                        
  float N2 = 0;                                          
  float N3 = 0;                                                   
  float D1 = 0;                                             
  float D2 = 0;                      
                                                                      
  for(int i=0; i<SAMPLES; i++)                  
  {
    int cc = (samples_index + i) % SAMPLES;
    N1+=(tim[cc]-stime)*alt[cc];                                          //
    N2+=(tim[cc]-stime);                                                  //
    N3+=(alt[cc]);                                                        //
    D1+=(tim[cc]-stime)*(tim[cc]-stime);                                  //
    D2+=(tim[cc]-stime);                                                  //
  }
  samples_index = (samples_index+1) % SAMPLES;
  
  // Durchschnittliches Körperende 
  
  /////VARIO VALUES CALCULATING /////
  float vario=1000*((SAMPLES*N1)-N2*N3)/(SAMPLES*D1-D2*D2);                     // BERECHNUNG VON VARIOMETERWERTEN

  //Serial.print(tim[samples_index]/1000);
  //Serial.print(";");
  //Serial.print(alt[samples_index]);
  //Serial.print(";");
  //Serial.println(vario);
  return vario;
}
