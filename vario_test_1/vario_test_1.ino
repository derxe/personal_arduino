/////////////////////////////////////////////////////////////////////////////////
// BMP280_DEV - I2C Communications, Forced Conversion with Timing Measurement
/////////////////////////////////////////////////////////////////////////////////

#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library

float temperature, pressure, altitude;            // Create the temperature, pressure, and altitude variables
BMP280_DEV bmp280;                                // Instantiate (create) a BMP280_DEV object for I2C operation (address 0x77)
#define buzzer 11   

#define SAMPLES  20   
float alt[SAMPLES];                                                       
unsigned long tim[SAMPLES];  
int samples_index = 0;
unsigned long tempo = millis();             // Beschreibung einer Tempovariablen vom Typ float und Zuweisung eines Wertes mit der Funktion millis () - Zählt die Zeit ab Programmstart in Millisekunden

float beep;                                  
float Beep_period; 

#define VARIO_TRESH_UP 0.2
#define VARIO_TRESH_DOWN -3.0


unsigned long startTime, endTime, elapsedTime;    // Variables for timing measurements

void setup() 
{
  Serial.begin(115200);                           // Initialize the serial port
  Serial.println("Over 8, IRR 16");
  bmp280.begin();                                 // Default initialization, places BMP280 into SLEEP_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X16);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);   
  bmp280.setIIRFilter(IIR_FILTER_16);  
  bmp280.setTimeStandby(TIME_STANDBY_05MS);
  bmp280.startForcedConversion();
  
  initSamples();
         
  pinMode(buzzer, OUTPUT);
}

void loop() 
{
  delay(10);                                 

  unsigned long calculationStart = micros();
  float vario = VarioMSCalculation();
  unsigned long calculationEnd = micros();
  //Serial.print("Duration:");
  //Serial.println((calculationEnd - calculationStart));

  //Serial.print("var ");
  //Serial.println(vario);
  VarioBeep(vario);
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
