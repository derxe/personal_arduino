#include <Button2.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <time.h>
#include <AsyncDelay.h>
#include <DFRobot_RGBLCD1602.h>

DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x2D, /*lcdCols*/16, /*lcdRows*/2);  // 16 characters, 2 lines

const int led = LED_BUILTIN;

#define WIFI_SSID "kuhna-wifi"
#define WIFI_PASS ""

#define SERVER_ADDR "http://192.168.0.1:5000"

#define BUTTON_A_PIN  12
#define BUTTON_B_PIN  14
#define BUTTON_C_PIN  13
#define BUZZER_PIN    15
Button2 buttonA;


void init_buttons() {
  buttonA.begin(BUTTON_A_PIN);
  buttonA.setTapHandler(clickA);
}

void init_screen() {
  lcd.init();
  lcd.noCursor();
  lcd.display();
  lcd.setHue(220);
}

void inti_wifi() {
  int32_t rssi = INT32_MIN;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial1.printf("Connect to WiFi...\n");
  lcd.setCursor(0, 0);
  lcd.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial1.printf(".");
  }

  Serial1.printf("Connected!\n");
  Serial1.println("");
  Serial1.print("Connected to ");
  Serial1.println(WIFI_SSID);
  Serial1.print("IP address: ");
  Serial1.println(WiFi.localIP());

  lcd.setCursor(0, 0);
  lcd.print("Connected!    ");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
}

void setColorLastTime(int timeLastMins) {
  int hue = max(0, 86 - timeLastMins*86 / (12*60));
  lcd.setHue(hue);  
}

void process_server_response(String payload) {
  Serial1.println(payload);  // Debug: Print the response
  
  int statusBreakIndex = payload.indexOf(';');
  int timeLastMins = payload.substring(0, statusBreakIndex).toInt();
  setColorLastTime(timeLastMins);
  
  String displayStr = payload.substring(statusBreakIndex + 1);

  // Split response into lines
  int lineBreakIndex = displayStr.indexOf('\n');
  String firstLine = displayStr.substring(0, lineBreakIndex);
  String secondLine = displayStr.substring(lineBreakIndex + 1);
  
  // Display the response on the LCD           // Clear the display
  lcd.setCursor(0, 0);       // Set cursor to first line
  lcd.print(firstLine);      // Print the first line of the response
  lcd.setCursor(0, 1);       // Set cursor to second line
  lcd.print(secondLine);     // Print the second line of the response
}


void get_status_from_server() {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(1000);

  Serial1.print("[HTTP] begin...\r\n");
  if (http.begin(client, SERVER_ADDR "/status")) {  // Update to match your endpoint

    Serial1.print("[HTTP] GET...\r\n");
    // Start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been sent and Server response header has been handled
      Serial1.printf("[HTTP] GET code: %d\r\n", httpCode);

      // File found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String payload = http.getString();
        process_server_response(payload);
      }
    } else {
      Serial1.printf("[HTTP] GET failed, error: %s\n", http.errorToString(httpCode).c_str());

      // Optionally display an error message on the LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Server offline?");
    }

    http.end();
  } else {
    Serial1.println("[HTTP] Unable to connect");

    // Optionally display a connection error on the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ni povezave!");
  }
}

int signal_server_cat_fed() {
  WiFiClient client;
  HTTPClient http;

  Serial1.print("[HTTP] begin...\r\n");
  if (http.begin(client, SERVER_ADDR "/cat_fed")) {  // HTTP

    Serial1.print("[HTTP] GET...\r\n");
    // Start the connection and send the HTTP header + payload
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been sent and Server response header has been handled
      Serial1.printf("[HTTP] GET code: %d\r\n", httpCode);

      // File found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        String response = http.getString();
        Serial1.println(response);
      }
    } else {
      Serial1.printf("[HTTP] GET failed, error: %s\r\n", http.errorToString(httpCode).c_str());
      return 0;
    }

    http.end();
    return 1;
  } else {
    Serial1.println("[HTTP] Unable to connect");
    return 0;
  }
}

AsyncDelay delay_get_status_from_server;

void setup() {
  Serial1.begin(115200);
  delay(50);
  init_screen();
  init_buttons();
  pinMode(BUZZER_PIN, OUTPUT);
  inti_wifi();

  delay_get_status_from_server.start(1000*60*5, AsyncDelay::MILLIS);
  delay_get_status_from_server.expire();
}



void loop() {
  buttonA.loop();

  if(delay_get_status_from_server.isExpired()) {
    get_status_from_server();
    delay_get_status_from_server.repeat(1000*60*5);  
  }
}

void beep_success() {
  tone(BUZZER_PIN, 1318);
  delay(100);
  tone(BUZZER_PIN, 1760);
  delay(150);
  tone(BUZZER_PIN, 2217);
  delay(200);
  noTone(BUZZER_PIN);
}

void beep_error() {
  tone(BUZZER_PIN, 987);
  delay(100);
  tone(BUZZER_PIN, 689);
  delay(100);
  noTone(BUZZER_PIN);
}


void clickA(Button2& btn) {
  Serial1.println("Btn A");
  lcd.setCursor(0, 1);
  lcd.print("A clicked");

  if(signal_server_cat_fed()) {
    lcd.setCursor(0, 1);
    lcd.print("Nafutran :)     ");  
    setColorLastTime(0);
    beep_success();
    delay_get_status_from_server.start(2000);
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Napaka :(       ");  
    beep_error();
  }
}
