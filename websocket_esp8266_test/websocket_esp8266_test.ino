#include <Arduino.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <Button2.h>
#include <LiquidCrystal_I2C.h>
#include <AsyncDelay.h>


// Define pins for SDA and SCL
#define SDA_PIN 0
#define SCL_PIN 2
#define OUT1_PIN 13

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
  float temp = Read_LM75(0);
  Serial.print("temp1:");
  Serial.println(temp);
  return temp;
}

float read_temp2() {
  float temp = Read_LM75(2);
  Serial.print("temp2:");
  Serial.println(temp);
  return temp;
}



// WiFi credentials
const char* ssid = "kuhna-wifi";
const char* password = "";

// WebSocket server details
const char* host = "192.168.0.5";
const int port = 8123;
const char* uri = "/api/websocket";

// Access token
const char* ACC_TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiI4MmI1ZDEzMjY1ODY0OTVkYTI4NDBkNGE1ZDY0Mjg5MiIsImlhdCI6MTczNzY0MzgxOCwiZXhwIjoyMDUzMDAzODE4fQ.-eaclNrlhhsdIIeya_TrOuGKanaEMG_QcAP0hOSUA3g";
int payload_id = 100;

// WebSocket client
WebSocketsClient webSocket;

// Function to process events
void processEvent(String jsonEvent) {
  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, jsonEvent);

  if (error) {
    Serial.println("Failed to parse event JSON");
    return;
  }

  // Extract specific details from the event
  const char* entity_id = doc["event"]["data"]["entity_id"];
  const char* new_state = doc["event"]["data"]["new_state"]["state"];
  
  Serial.print("Entity: ");
  Serial.println(entity_id);
  Serial.print("New State: ");
  Serial.println(new_state);

  // Add your logic here to handle the event
  if (String(entity_id) == "switch.your_switch_entity") {
    if (String(new_state) == "on") {
      // Do something when the switch is turned on
      Serial.println("Switch turned ON");
    } else {
      // Do something when the switch is turned off
      Serial.println("Switch turned OFF");
    }
  }
}

void send_subscribe_payload() {
  String payload = "{\n";
  payload += "  \"id\": " + String(payload_id++) + ",\n";
  payload += "  \"type\": \"subscribe_trigger\",\n";
  payload += "  \"trigger\": [\n";
  payload += "    {\n";
  payload += "      \"platform\": \"state\",\n";
  payload += "      \"entity_id\": \"input_boolean.pec_toggle\"\n";
  payload += "    },\n";
  payload += "    {\n";
  payload += "      \"platform\": \"state\",\n";
  payload += "      \"entity_id\": \"input_boolean.pump_toggle\"\n";
  payload += "    },\n";
  payload += "    {\n";
  payload += "      \"platform\": \"state\",\n";
  payload += "      \"entity_id\": \"climate.termostat_pec\",\n";
  payload += "      \"attribute\": \"target_temp\"\n";
  payload += "    },\n";
  payload += "    {\n";
  payload += "      \"platform\": \"state\",\n";
  payload += "      \"entity_id\": \"climate.termostat_pumpa\",\n";
  payload += "      \"attribute\": \"target_temp\"\n";
  payload += "    }\n";
  payload += "  ]\n";
  payload += "}";

  Serial.print("Subscribe payload: ");
  Serial.println(payload);
  webSocket.sendTXT(payload);
}

// WebSocket event handler
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");
      break;
    case WStype_CONNECTED: {
      Serial.println("WebSocket Connected");
      
      // Authenticate with Home Assistant
      String authPayload = "{\"type\": \"auth\", \"access_token\": \"" + String(ACC_TOKEN) + "\"}";
      webSocket.sendTXT(authPayload);
      break;
    }
    case WStype_TEXT: {
      String response = String((char*)payload);
      Serial.println("Received: " + response);

      // Check if it's an auth success response
      if (response.indexOf("\"type\":\"auth_ok\"") >= 0) {
        // Subscribe to state_changed events
        send_subscribe_payload();
      } else if (response.indexOf("\"event\"") >= 0) {
        // Process incoming event
        processEvent(response);
      }
      break;
    }
    default:
      break;
  }
}


void sendInputNumberValue(const char* entity_id, float value) {
  String payload = "";
  payload += "{\n";
  payload += "  \"id\": " + String(payload_id++) + ",\n";
  payload += "  \"type\": \"call_service\",\n";
  payload += "  \"domain\": \"input_number\",\n";
  payload += "  \"service\": \"set_value\",\n";
  payload += "  \"service_data\": {\n";
  payload += "    \"entity_id\": \"" + String(entity_id) + "\",\n";
  payload += "    \"value\": " + String(value) + "\n";
  payload += "  }\n";
  payload += "}";

  Serial.print("Payload: ");
  Serial.println(payload);
  // Send the payload via WebSocket
  webSocket.sendTXT(payload);
}


void sendThermostatTargetTemp(const char* entity_id, float target_temp) {
  String payload = "";
  payload += "{\n";
  payload += "  \"id\": " + String(payload_id++) + ",\n";
  payload += "  \"type\": \"call_service\",\n";
  payload += "  \"domain\": \"climate\",\n";
  payload += "  \"service\": \"set_temperature\",\n";
  payload += "  \"service_data\": {\n";
  payload += "    \"entity_id\": \"" + String(entity_id) + "\",\n";
  payload += "    \"temperature\": " + String(target_temp) + "\n";
  payload += "  }\n";
  payload += "}";

  Serial.print("Payload: ");
  Serial.println(payload);
  // Send the payload via WebSocket
  webSocket.sendTXT(payload);
}

#define BUTTON_A_PIN  12
#define BUTTON_B_PIN  14
#define BUTTON_C_PIN  13

Button2 buttonA, buttonB, buttonC;

void init_buttons() {
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(clickA);

  buttonB.begin(BUTTON_B_PIN);
  buttonB.setTapHandler(clickB);

  buttonC.begin(BUTTON_C_PIN);
  buttonC.setTapHandler(clickC_no_fun);
}

struct TermostatValue {
    float value;       // Current value
    int edit_min;      
    int edit_max;      
    float edit_increment; // Increment step size, default to 2
    const char* entity_id; // Entity ID for the setting

    // Constructor for initialization
    TermostatValue(const char* id)
        : value(-1), edit_min(-1), edit_max(-1), edit_increment(-1), entity_id(id) {}

    // Print the current state of the setting
    void print() const {
        Serial.print("Entity ID: "); Serial.print(entity_id);
        Serial.print(", Value: "); Serial.print(value);
        Serial.print(", Min: "); Serial.print(edit_min);
        Serial.print(", Max: "); Serial.print(edit_max);
        Serial.print(", Increment: "); Serial.println(edit_increment);
    }

    void load_values_from_hs() {
        StaticJsonDocument<2048> doc;
        String response = sendApiRequest("/api/states/" + String(entity_id));

        if (response == "") {
          Serial.println("Unable to get an answer from the server.");
          return;  
        }
        
        DeserializationError error = deserializeJson(doc, response);
      
        if (error) {
          Serial.println("Failed to parse event JSON");
          return;
        }

        edit_min = doc["attributes"]["min_temp"];
        edit_max = doc["attributes"]["max_temp"];
        edit_increment = doc["attributes"]["target_temp_step"];
        value = doc["attributes"]["temperature"];

        Serial.println("Loaded new values into the setting");
    }

    // Increase the value with constraints
    void increase_value() {
        value += edit_increment;
        value = constrain(value, edit_min, edit_max);
    }

    // Decrease the value with constraints
    void decrease_value() {
        value -= edit_increment;
        value = constrain(value, edit_min, edit_max);
    }

    // Send value if too old
    void sendValue() {
      sendThermostatTargetTemp(entity_id, value);
    }
};



int state = 0;
#define NUM_STATES 5
bool refresh_lcd = false;

TermostatValue pec_termo("climate.termostat_pec");
TermostatValue pumpa_termo("climate.termostat_pumpa");


TermostatValue *value_edited = NULL;

int pec_state = 0;
bool editing = false;

void clickA(Button2& btn) {
  state = (state+1) % NUM_STATES;
  refresh_lcd = true;

  Serial.print("Btn A");
  Serial.println(state);
}

void clickB(Button2& btn) {
  state = (state-1+NUM_STATES) % NUM_STATES;
  refresh_lcd = true;

  Serial.print("Btn B");
  Serial.println(state);
}

void clickC_no_fun(Button2& btn) {
  Serial.println("Btn C no fun");
}

void clickC_toggle_eddit_mode(Button2& btn) {
  Serial.println("Btn C edit mode");
  editing = !editing;
  refresh_lcd = true;

  if (editing) {
    buttonA.setTapHandler(clickA_edit);
    buttonA.setLongClickDetectedHandler(longClickDetectedA_edit);
    buttonA.setLongClickDetectedRetriggerable(true);

    buttonB.setTapHandler(clickB_edit);
    buttonB.setLongClickDetectedHandler(longClickDetectedB_edit);
    buttonB.setLongClickDetectedRetriggerable(true);
  } else {
    buttonA.setTapHandler(clickA);    
    buttonA.setLongClickDetectedHandler(NULL);

    buttonB.setTapHandler(clickB);
    buttonB.setLongClickDetectedHandler(NULL);
    value_edited->sendValue();
  }
}

void clickC_change_pec_state(Button2& btn) {
  Serial.println("Btn C pec state");
  pec_state = (pec_state+1)%3;
  refresh_lcd = true;
}

void increase_value_edited() {
  value_edited->increase_value();
  refresh_lcd = true;  
}

void decrease_value_edited() {
  value_edited->decrease_value();
  refresh_lcd = true;  
}

void clickA_edit(Button2& btn) {
  increase_value_edited();

  Serial.print("Btn A edit: ");
  value_edited->print();
}

void clickB_edit(Button2& btn) {
  decrease_value_edited();

  Serial.print("Btn B edit: ");
  value_edited->print();
}


void longClickDetectedA_edit(Button2& btn) {
    Serial.print("long click A #");
    Serial.print(btn.getLongClickCount());
    Serial.println(" detected");

    increase_value_edited();
}

void longClickDetectedB_edit(Button2& btn) {
    Serial.print("long click B #");
    Serial.print(btn.getLongClickCount());
    Serial.println(" detected");

    decrease_value_edited();
}


#define LCD_I2C_ADRESS  0x27
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
LiquidCrystal_I2C lcd(LCD_I2C_ADRESS, LCD_WIDTH, LCD_HEIGHT); 


void init_screen() {
  lcd.init();
  lcd.backlight();
  lcd.noCursor();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nalagam ...");
}

AsyncDelay delay_refresh_lcd;



void lcd_loop() {
  if(delay_refresh_lcd.isExpired() || refresh_lcd) {
    refresh_lcd = false;
    Serial.print("State: ");
    Serial.println(state);
    
    switch(state) {
      case 0:
        buttonC.setTapHandler(clickC_no_fun);
        
        lcd.setCursor(0, 0);       
        lcd.print("temperature:    ");
        lcd.setCursor(0, 1);
        lcd.printf("1:%4.1f 2:%4.1f %cC", read_temp1(), read_temp2(), (char)223); 

        delay_refresh_lcd.start(500);
        break;

      case 1:
        buttonC.setTapHandler(clickC_no_fun);
      
        lcd.setCursor(0, 0);       
        lcd.print("temperature:    ");
        lcd.setCursor(0, 1);
        lcd.printf("3:%4.1f 4:%4.1f %cC", -1.0, -1.0, (char)223); 

        delay_refresh_lcd.start(500);
        break;

      case 2:
        buttonC.setTapHandler(clickC_toggle_eddit_mode);
        value_edited = &pec_termo;

        if(editing) {
          lcd.setCursor(0, 0);       
          lcd.print("pec termostat: ");
          lcd.setCursor(0, 1);
          lcd.printf("nastavi: %4.1f %cC", pec_termo.value, (char)223); 
        } else {
          lcd.setCursor(0, 0);       
          lcd.print("pec termostat: ");
          lcd.setCursor(0, 1);
          lcd.printf("%4.1f %cC", pec_termo.value, (char)223);   
        }
        delay_refresh_lcd.start(12345678);
        break;

      case 3:
        buttonC.setTapHandler(clickC_toggle_eddit_mode);
        value_edited = &pumpa_termo;

        if(editing) {
          lcd.setCursor(0, 0);       
          lcd.print("pumpa termostat");
          lcd.setCursor(0, 1);
          lcd.printf("nastavi: %4.1f %cC", pumpa_termo.value, (char)223); 
        } else {
          lcd.setCursor(0, 0);       
          lcd.print("pumpa termostat");
          lcd.setCursor(0, 1);
          lcd.printf("%4.1f %cC", pumpa_termo.value, (char)223);   
        }
        delay_refresh_lcd.start(12345678);
        break;

      case 4:
        buttonC.setTapHandler(clickC_change_pec_state);


        lcd.setCursor(0, 0);       
        lcd.print("stanje pec:    ");
        lcd.setCursor(0, 1);

        switch(pec_state) {
          case 0: lcd.print("izklopljena     "); break;
          case 1: lcd.printf("termostat %4.1f", pec_termo.value); break;
          case 2: lcd.print("vklopljena     "); break;
        }

        delay_refresh_lcd.start(12345678);
        break;
    }
  }
}

String sendApiRequest(String uri) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected.");
        return "";
    }
    
    HTTPClient http;
    WiFiClient client;

    // Construct the full URL
    String url = "http://" + String(host) + ":" + String(port) + String(uri);

    // Begin the HTTP request
    http.begin(client, url);

    // Set headers
    http.addHeader("Authorization", "Bearer " + String(ACC_TOKEN));
    http.addHeader("Content-Type", "application/json");

    // Send GET request
    int httpCode = http.GET();

    // Print HTTP status and response
    if (httpCode > 0) {
        Serial.println("HTTP Response Code: " + String(httpCode));
        String payload = http.getString();
        Serial.println("Response: " + payload);
        return payload;
    } else {
        Serial.println("Error on HTTP request: " + String(http.errorToString(httpCode).c_str()));
        return "";
    }

    // End the HTTP request
    http.end();
}

void setup() {
  // Start Serial
  Serial.begin(115200);
  init_buttons();
  init_screen();
  
  i2c_port_initial();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Setup WebSocket
  webSocket.begin(host, port, uri);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  pec_termo.load_values_from_hs();
  pumpa_termo.load_values_from_hs();
}



void loop() {
  webSocket.loop();

  buttonA.loop();
  buttonB.loop();
  buttonC.loop();

  lcd_loop();

  // Example: Send new values every 10 seconds
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 10000) {
    sendInputNumberValue("input_number.temp1", read_temp1()); 
    sendInputNumberValue("input_number.temp2", read_temp2());
    lastUpdate = millis();
  }
}
