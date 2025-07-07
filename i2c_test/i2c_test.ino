#define BLYNK_TEMPLATE_ID           "TMPL4htHPYeWK"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "xbyILJ0dOsHEsQWysZkVzUrD9pE0dj3M"
#define BLYNK_PRINT Serial

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


#define LCD_I2C_ADRESS  0x27
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
LiquidCrystal_I2C lcd(LCD_I2C_ADRESS, LCD_WIDTH, LCD_HEIGHT); 

// Define pins for SDA and SCL
#define SDA_PIN 0
#define SCL_PIN 2
#define OUT1_PIN 13

BlynkTimer timer;


// Set SDA high
void sda_high() {
    digitalWrite(SDA_PIN, HIGH);
}

// Set SDA low
void sda_low() {
    digitalWrite(SDA_PIN, LOW);
}

// Set SCL high
void scl_high() {
    digitalWrite(SCL_PIN, HIGH);
}

// Set SCL low
void scl_low() {
    digitalWrite(SCL_PIN, LOW);
}

// Initialize I2C pins
void i2c_port_initial() {
    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
    sda_high();
    scl_high();
}

// Start condition
void i2c_start_condition() {
    sda_high();
    scl_high();
    delayMicroseconds(10);
    sda_low();
    delayMicroseconds(10);
    scl_low();
}

// Stop condition
void i2c_stop_condition() {
    sda_low();
    scl_high();
    delayMicroseconds(10);
    sda_high();
    delayMicroseconds(10);
}

// Write a byte of data to the I2C bus
void i2c_write_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            sda_high();
        } else {
            sda_low();
        }
        delayMicroseconds(10);
        scl_high();
        delayMicroseconds(10);
        scl_low();
    }
}

// Read a byte of data from the I2C bus
uint8_t i2c_read_byte() {
    uint8_t data = 0;
    pinMode(SDA_PIN, INPUT);
    pinMode(SDA_PIN, INPUT_PULLUP);
    for (int i = 0; i < 8; i++) {
        scl_high();
        delayMicroseconds(10);
        if (digitalRead(SDA_PIN)) {
            data |= (1 << (7 - i));
        }
        scl_low();
        delayMicroseconds(10);
    }
    pinMode(SDA_PIN, OUTPUT);
    return data;
}

// Send ACK
void i2c_send_ack() {
    sda_low();
    delayMicroseconds(10);
    scl_high();
    delayMicroseconds(10);
    scl_low();
    sda_high();
}

// Send NACK
void i2c_send_nack() {
    sda_high();
    delayMicroseconds(10);
    scl_high();
    delayMicroseconds(10);
    scl_low();
}

// Read LM75 sensor
float Read_LM75(uint8_t address) {
    uint16_t Temp = 0;
    uint8_t read_d[2];
    uint8_t prefix = 0b10010000;

    for (int i = 0; i < 1; i++) { // Try up to 3 times
        i2c_start_condition();
        i2c_write_byte(prefix | (((address & 0x07) << 1) | 0x01)); // Send address with read bit
        i2c_send_ack();

        read_d[0] = i2c_read_byte();
        i2c_send_ack();
        read_d[1] = i2c_read_byte();
        i2c_send_nack();

        //Serial.println();

        i2c_stop_condition();

        Temp = (read_d[0] << 8) | read_d[1];
        Temp = 5 * (Temp >> 7);

        if (Temp <= 1260) {
            return Temp / 10.0;
        }
    }
    return -1;
}

float read_temp1() {
  return Read_LM75(0);
}

float read_temp2() {
  return Read_LM75(2);
}


// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
  Serial.println("On V0??");

  digitalWrite(OUT1_PIN, value ? HIGH : LOW);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  //Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  //Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  //Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
  Serial.println("Connected!");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  //Blynk.virtualWrite(V2, millis() / 1000);

  float temp1 = read_temp1();
  float temp2 = read_temp2();
  Serial.print("temp1:");
  Serial.print(temp1);
  Serial.print(" temp2:");
  Serial.println(temp2);

  lcd.setCursor(0, 0);
  lcd.print("T1:");
  lcd.print(temp1);

  lcd.setCursor(0, 1);
  lcd.print("T2:");
  lcd.print(temp2);
  
  //Blynk.virtualWrite(V3, temp1);
  Blynk.virtualWrite(V6, temp2);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(OUT1_PIN, OUTPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, "kuhna-wifi", "");
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(2000L, myTimerEvent);
  
  i2c_port_initial();
  
  lcd.init();    
  lcd.backlight();
  lcd.noCursor();   
  lcd.setCursor(0, 0);
  lcd.print("inting ......");
}


void loop() {
  /*float temp1 = read_temp1();
  float temp2 = read_temp2();
  Serial.print("temp1:");
  Serial.print(temp1);
  Serial.print(" temp2:");
  Serial.println(temp2);

  lcd.setCursor(0, 0);
  lcd.print("T1:");
  lcd.print(temp1);

  lcd.setCursor(0, 1);
  lcd.print("T2:");
  lcd.print(temp2);

  delay(500);
*/
  Blynk.run();
  timer.run();
  
  
}
