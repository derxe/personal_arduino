#include <LittleFS.h>
#include "Button2.h"
#define BUTTON_PIN  7


#define BUZZER_PIN 11  // Change this to your actual LED pin
#define ROTATION_SENSOR_PIN 3

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;

#define SAMPLES  15   
float values[SAMPLES];                                                       
unsigned long times[SAMPLES];  
int samples_index = 0;
float smooth_value = 0;
float prev_smooth_value = 0;

bool detectQuickChange(float val){
  // record a new sample                                                         
  values[samples_index] = val;                                 
  times[samples_index] = micros();                                             
   unsigned long stime = times[(samples_index+1)%SAMPLES];      // oldest sample                          
  float N1 = 0;                                                        
  float N2 = 0;                                          
  float N3 = 0;                                                   
  float D1 = 0;                                             
  float D2 = 0;                      
                                                                      
  for(int i=0; i<SAMPLES; i++)                  
  {
    int cc = (samples_index + i) % SAMPLES;
    N1+=(times[cc]-stime)*values[cc];                                          
    N2+=(times[cc]-stime);                                                  
    N3+=(values[cc]);                                                        
    D1+=(times[cc]-stime)*(times[cc]-stime);                                  
    D2+=(times[cc]-stime);                                                  
  }
  samples_index = (samples_index+1) % SAMPLES;
  
  float valuea = ((SAMPLES*N1)-N2*N3)/(SAMPLES*D1-D2*D2)*500;
  //Serial.print("valuea:");
  //Serial.print(valuea);
  valuea = valuea*valuea*valuea*valuea*valuea*valuea*valuea ;

  
  
  float alfa = 0.9;
  smooth_value = valuea*(1-alfa) + smooth_value*alfa;
  smooth_value = constrain(smooth_value, -5000, 10000) - 2000;
  
  //Serial.print("zero:");
  //Serial.print(0);
  //Serial.print(" dvesto: ");
  //Serial.print(200);
  //Serial.print(" smoo:");
  //Serial.println(smooth_value);

  bool quickChange = (prev_smooth_value < 0 && smooth_value > 0);
  prev_smooth_value = smooth_value;

  //Serial.print(" change:");
  //Serial.print(quickChange*100);
  //Serial.println();

  return quickChange;
}

#define SPEED_LOG_SIZE 1000
float speedLog[SPEED_LOG_SIZE];
int speedLogIndex = 0;


void saveSpeeds() {
  File file = LittleFS.open("/log.txt", "a");
  if (file) {
    Serial.print("Saving data: ");
    Serial.print(speedLogIndex);
    file.print("Speeds:");
    for(int i=0; i<speedLogIndex; i++) {
        file.print(speedLog[i]);
        file.print(", ");
    }
    speedLogIndex = 0;
    Serial.println(" Done");
    file.println("end");
    file.close();
  }
}

void ledTurnOnSequence() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);  // Small pause between beeps
  }

  // Long blink (400ms)
  digitalWrite(BUZZER_PIN, HIGH);
  delay(400);
  digitalWrite(BUZZER_PIN, LOW);
  
}
void click(Button2& btn) {
    unsigned long writeStart = micros();
    
    saveSpeeds();
    Serial.print(" write:");
    Serial.println((micros() - writeStart)/1000);
    
}

void longClick(Button2& btn) {
    Serial.println(" Long click:");

    File file = LittleFS.open("/log.txt", "r");
    if (!file) {
      Serial.println("No file found. Creating one...");
    } else {
      Serial.println("Contents of /log.txt:");
      while (file.available()) {
        Serial.write(file.read());
      }
      file.close();
      Serial.println("\n--- End of file ---");
    }

    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
  
    Serial.println();
    Serial.print("Total bytes: ");
    Serial.println(total);
    Serial.print("Used bytes : ");
    Serial.println(used);
}

Button2 button;

void setup() {
  //Serial.begin(115200);
  //while(!Serial); // Just to give time for serial to initialize
  //Serial.println("Analogue demo start");

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ROTATION_SENSOR_PIN, INPUT);

  button.begin(BUTTON_PIN);
  button.setClickHandler(click);
  button.setLongClickHandler(longClick);
  
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount + Format Failed");
    return;
  }

  //LittleFS.remove("/log.txt");


  ledTurnOnSequence();
}

long microsStart = 0;
int allRotationCount = 0;
int rotationCount = 0;

int speedAllRotationCount = 0; 
int speedRotationCount = 0;
float speedRps = 0;


float rps = 0; // rotations per second
unsigned long lastDetection = 0; 
unsigned long lastPrint = 0;
unsigned long ledOnTime = 0; 
int lastRotationRead = 0;

void loop() {
  button.loop();
   
  unsigned long duration = micros() - microsStart;
  delay(2);
  
  microsStart = micros();
  unsigned long millisNow = millis();

  if(millisNow - ledOnTime > 40) {
     digitalWrite(BUZZER_PIN, HIGH);
  }

  int rotationRead = digitalRead(ROTATION_SENSOR_PIN);
  if(rotationRead == 1 && lastRotationRead == 0) {
      Serial.println("Rotation!");
      lastRotationRead = 1;
      speedAllRotationCount ++;
      speedRotationCount ++;
  }
  lastRotationRead = rotationRead; 
  
  float lightValue = analogRead(1);
  if(detectQuickChange(lightValue)) {
    allRotationCount ++;
    rotationCount ++; 
    rps += 1000.0f / (millisNow - lastDetection);
    lastDetection = millisNow; 

    digitalWrite(BUZZER_PIN, LOW);  // Turn LED ON
    ledOnTime = millisNow;
    
    //Serial.print(" rps: ");
    //Serial.print(rps);
    //Serial.print("Rotations:");
    //Serial.print(rotationCount);
    //Serial.print(" ");
    //Serial.print(allRotationCount);
    //Serial.println();
  }  

  if(millisNow - lastPrint> 1000) {  
    
    float sped =  rotationCount == 0? 0 : rps / rotationCount;
    //Serial.print("speed:"); 
    //Serial.println(sped);

    speedLog[speedLogIndex] = sped; 
    speedLogIndex = (speedLogIndex+1) % SPEED_LOG_SIZE;

    rotationCount = 0; 
    rps = 0;

    lastPrint = millisNow; 
  }
  
}
