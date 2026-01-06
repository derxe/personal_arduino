/*#include "ESP32_SoftWire.h"

// ----------------- Pins -----------------
const int PIN_VIN      = 36;   // power to AHT20 (optional, can use 3V3 instead)
const int PIN_GND      = 34;   // "ground" via GPIO (optional, can use real GND)
const int i2c_SDA_PIN  = 40;   // your custom SDA pin
const int i2c_SCL_PIN  = 38;   // your custom SCL pin

const uint8_t AHT20_ADDR = 0x38;  // AHT20 I2C address

SoftWire i2c;  // software I2C object

// ----------------- AHT20 helpers -----------------

bool aht20_init() {
  // Soft reset
  i2c.beginTransmission(AHT20_ADDR);
  i2c.write(0xBA);  // soft reset
  if (i2c.endTransmission() != 0) {
    return false;
  }
  delay(20);

  // Init / calibrate: 0xBE 0x08 0x00
  i2c.beginTransmission(AHT20_ADDR);
  i2c.write(0xBE);
  i2c.write(0x08);
  i2c.write(0x00);
  if (i2c.endTransmission() != 0) {
    return false;
  }

  delay(10);
  return true;
}

bool aht20_read(float &temperatureC, float &humidityRH) {
  // Trigger measurement: 0xAC 0x33 0x00
  i2c.beginTransmission(AHT20_ADDR);
  i2c.write(0xAC);
  i2c.write(0x33);
  i2c.write(0x00);
  if (i2c.endTransmission() != 0) {
    return false;
  }

  // Wait for conversion (~80 ms)
  delay(80);

  // Read 6 bytes: status + 5 data bytes
  uint8_t buf[6];
  uint8_t received = i2c.requestFrom(AHT20_ADDR, (uint8_t)6);
  if (received != 6) {
    return false;
  }
  i2c.readBytes(buf, received);

  uint8_t status = buf[0];
  if (status & 0x80) {
    // still busy
    return false;
  }

  // Decode 20-bit humidity and temperature (datasheet)
  uint32_t hum_raw  = ((uint32_t)buf[1] << 12)
                    | ((uint32_t)buf[2] << 4)
                    | ((uint32_t)(buf[3] & 0xF0) >> 4);

  uint32_t temp_raw = ((uint32_t)(buf[3] & 0x0F) << 16)
                    | ((uint32_t)buf[4] << 8)
                    | (uint32_t)buf[5];

  // RH = hum_raw / 2^20 * 100
  humidityRH   = (hum_raw  * 100.0f) / 1048576.0f;      // 2^20
  // T = temp_raw / 2^20 * 200 - 50
  temperatureC = (temp_raw * 200.0f) / 1048576.0f - 50.0f;

  return true;
}

// ----------------- Arduino setup/loop -----------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Optional: power AHT20 from GPIOs
  pinMode(PIN_VIN, OUTPUT);
  pinMode(PIN_GND, OUTPUT);
  digitalWrite(PIN_GND, LOW);
  digitalWrite(PIN_VIN, HIGH);

  delay(50); // sensor power-up

  // Init software I2C on your pins
  i2c.begin(i2c_SDA_PIN, i2c_SCL_PIN, 400000); // 400 kHz; 100 kHz also fine

  Serial.println("Init AHT20 via ESP32_SoftWire...");

  if (!aht20_init()) {
    Serial.println("AHT20 init FAILED (no ACK / wiring?).");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("AHT20 ready.");
}

void loop() {
  float t = 0.0f;
  float h = 0.0f;

  if (aht20_read(t, h)) {
    Serial.print("T = ");
    Serial.print(t, 2);
    Serial.print(" °C   H = ");
    Serial.print(h, 2);
    Serial.println(" %");
  } else {
    Serial.println("AHT20 read FAILED");
  }

  delay(1000); // 1 Hz
}*/



/*
// ---------------------- Pin config ----------------------
const int PIN_VIN      = 36;   // power to AHT20 (optional, can use 3V3 instead)
const int PIN_GND      = 34;   // "ground" via GPIO (optional, can use real GND)
const int PIN_SDA  = 40;   // your custom SDA pin
const int PIN_SCL  = 38;   // your custom SCL pin

constexpr uint8_t AHT20_ADDR = 0x38;  // 7-bit I2C address

// ---------------------- Soft I2C helpers ----------------------
// We simulate open-drain:
//  - "HIGH" = input with pull-up (line floats high via pull-up)
//  - "LOW"  = output driving low

static inline void i2c_delay() {
  delayMicroseconds(5); // ~100 kHz-ish, can adjust
}

static inline void SDA_HIGH() {
  pinMode(PIN_SDA, INPUT_PULLUP);
}

static inline void SDA_LOW() {
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
}

static inline void SCL_HIGH() {
  pinMode(PIN_SCL, INPUT_PULLUP);
}

static inline void SCL_LOW() {
  pinMode(PIN_SCL, OUTPUT);
  digitalWrite(PIN_SCL, LOW);
}

static inline uint8_t SDA_READ() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  return digitalRead(PIN_SDA);
}

// Start condition: SDA goes LOW while SCL HIGH
void i2c_start() {
  SDA_HIGH();
  SCL_HIGH();
  i2c_delay();
  SDA_LOW();
  i2c_delay();
  SCL_LOW();
}

// Stop condition: SDA goes HIGH while SCL HIGH
void i2c_stop() {
  SDA_LOW();
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  SDA_HIGH();
  i2c_delay();
}

// Write one byte, return ACK (0 = ACK, 1 = NACK)
uint8_t i2c_write_byte(uint8_t data) {
  for (int8_t i = 7; i >= 0; --i) {
    (data & (1 << i)) ? SDA_HIGH() : SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
  }

  // Release SDA for ACK bit
  SDA_HIGH();
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  uint8_t ack = SDA_READ();  // 0 = ACK
  SCL_LOW();
  return ack;
}

// Read one byte, send ACK if ack=0, NACK if ack=1
uint8_t i2c_read_byte(uint8_t ack) {
  uint8_t data = 0;
  SDA_HIGH();  // release SDA
  for (int8_t i = 7; i >= 0; --i) {
    SCL_HIGH();
    i2c_delay();
    uint8_t bit = SDA_READ();
    data |= (bit << i);
    SCL_LOW();
    i2c_delay();
  }

  // Send ACK/NACK bit
  if (ack == 0) {
    SDA_LOW();   // ACK = drive low
  } else {
    SDA_HIGH();  // NACK = release (high)
  }
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  SCL_LOW();
  SDA_HIGH();
  return data;
}

// ---------------------- AHT20 driver (minimal) ----------------------

bool aht20_init() {
  // Soft reset
  i2c_start();
  i2c_write_byte((AHT20_ADDR << 1) | 0); // write
  i2c_write_byte(0xBA);                  // soft reset
  i2c_stop();
  delay(20);

  // Init / calibrate: 0xBE 0x08 0x00
  i2c_start();
  if (i2c_write_byte((AHT20_ADDR << 1) | 0) != 0) { // address
    i2c_stop();
    return false;
  }
  if (i2c_write_byte(0xBE) != 0) { i2c_stop(); return false; }
  if (i2c_write_byte(0x08) != 0) { i2c_stop(); return false; }
  if (i2c_write_byte(0x00) != 0) { i2c_stop(); return false; }
  i2c_stop();

  delay(10);
  return true;
}

bool aht20_read(float &temperatureC, float &humidityRH) {
  // Trigger measurement: 0xAC 0x33 0x00
  i2c_start();
  if (i2c_write_byte((AHT20_ADDR << 1) | 0) != 0) {  // address + write
    i2c_stop();
    return false;
  }
  if (i2c_write_byte(0xAC) != 0) { i2c_stop(); return false; }
  if (i2c_write_byte(0x33) != 0) { i2c_stop(); return false; }
  if (i2c_write_byte(0x00) != 0) { i2c_stop(); return false; }
  i2c_stop();

  // Wait for conversion
  delay(80);

  // Read 6 bytes: status + 5 data bytes
  uint8_t buf[6];

  i2c_start();
  if (i2c_write_byte((AHT20_ADDR << 1) | 1) != 0) {  // address + read
    i2c_stop();
    return false;
  }

  for (int i = 0; i < 5; ++i) {
    buf[i] = i2c_read_byte(0); // ACK for first 5 bytes
  }
  buf[5] = i2c_read_byte(1);   // NACK on last byte
  i2c_stop();

  uint8_t status = buf[0];
  if (status & 0x80) {
    // Still busy
    return false;
  }

  // Decode 20-bit humidity and temperature (datasheet)
  uint32_t hum_raw  = ((uint32_t)buf[1] << 12)
                    | ((uint32_t)buf[2] << 4)
                    | ((uint32_t)(buf[3] & 0xF0) >> 4);

  uint32_t temp_raw = ((uint32_t)(buf[3] & 0x0F) << 16)
                    | ((uint32_t)buf[4] << 8)
                    | (uint32_t)buf[5];

  humidityRH   = (hum_raw * 100.0f) / 1048576.0f;       // 2^20
  temperatureC = (temp_raw * 200.0f) / 1048576.0f - 50.0f;

  return true;
}

// ---------------------- Arduino setup/loop ----------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Power pins
  pinMode(PIN_VIN, OUTPUT);
  pinMode(PIN_GND, OUTPUT);
  digitalWrite(PIN_GND, LOW);   // "ground"
  digitalWrite(PIN_VIN, HIGH);  // power ON sensor

  // I2C idle state
  SDA_HIGH();
  SCL_HIGH();

  delay(50); // sensor power-up

  Serial.println("Init AHT20 (soft I2C)...");
  if (!aht20_init()) {
    Serial.println("AHT20 init FAILED.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("AHT20 ready.");
}

void loop() {
  float t = 0.0f;
  float h = 0.0f;

  if (aht20_read(t, h)) {
    Serial.print("T = ");
    Serial.print(t, 2);
    Serial.print(" °C   H = ");
    Serial.print(h, 2);
    Serial.println(" %");
  } else {
    Serial.println("AHT20 read FAILED (busy/no ACK).");
  }

  delay(1000);
}
*/

#include "AHT20_SoftI2C.h"


// Power pins (you keep controlling those in the sketch)
const int PIN_VIN = 36;
const int PIN_GND = 34;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Power sensor from GPIOs (optional)
  pinMode(PIN_VIN, OUTPUT);
  pinMode(PIN_GND, OUTPUT);
  digitalWrite(PIN_GND, LOW);
  digitalWrite(PIN_VIN, HIGH);

  delay(50); // sensor power-up

  // Set SDA/SCL pins (if you want different than defaults)
  AHT20SoftI2C::setPins(40, 38);  // SDA=40, SCL=38

 
  AHT20SoftI2C::SDA_HIGH();
  AHT20SoftI2C::SCL_HIGH();

  Serial.println("Init AHT20 (soft I2C)...");
  if (!AHT20SoftI2C::aht20_init()) {
    Serial.println("AHT20 init FAILED.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("AHT20 ready.");
}

bool readSensorTempHum(float &tempC, float &humRH) {
  digitalWrite(PIN_VIN, HIGH);
  AHT20SoftI2C::SDA_HIGH();  // Put bus in idle state
  AHT20SoftI2C::SCL_HIGH();
  delay(100);

  if(!AHT20SoftI2C::aht20_init()) {
    digitalWrite(PIN_VIN, LOW);
    return false;
  }

  bool success = AHT20SoftI2C::aht20_read(tempC, humRH);
  digitalWrite(PIN_VIN, LOW);
  AHT20SoftI2C::SDA_LOW(); // put everythign to low
  AHT20SoftI2C::SCL_LOW();
  return success;
}

void loop() {
  float t = NAN;
  float h = NAN;

  if(readSensorTempHum(t, h)) {

  } else {
    Serial.println("AHT20 read FAILED (busy/no ACK).");
  }

    Serial.print("T = ");
    Serial.print(String(t, 2));
    Serial.print(" °C   H = ");
    Serial.print(h, 2);
    Serial.println(" %");
  delay(1000);
}












