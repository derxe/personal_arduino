#include <ESP8266WiFi.h>

const char* ssid = "kuhna-wifi";
const char* password = "";
const char* host = "192.168.0.120"; // Replace with your server's IP
const uint16_t port = 12345;        // Replace with your server's port
WiFiClient client;

// Function to toggle a pin based on parsed server response
void toggle_pin(int pin, const String& state) {
    pinMode(pin, OUTPUT);
    if (state == "ON") {
        digitalWrite(pin, HIGH);
        Serial.printf("Pin %d set to ON\r\n", pin);
    } else if (state == "OFF") {
        digitalWrite(pin, LOW);
        Serial.printf("Pin %d set to OFF\r\n", pin);
    } else {
        Serial.println("Invalid state");
    }
}

// Function to parse server response
bool parse_server_response(const String& response, int& pin, String& state) {
    if (response.startsWith("set:")) {
        int firstColon = response.indexOf(':');
        int secondColon = response.indexOf(':', firstColon + 1);
        if (firstColon > 0 && secondColon > firstColon) {
            pin = response.substring(firstColon + 1, secondColon).toInt();
            state = response.substring(secondColon + 1);
            state.trim(); // Remove any extra spaces or newlines
            return true;
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    if (!client.connect(host, port)) {
        Serial.println("Connection to server failed");
    } else {
        Serial.println("Connected to server");
    }
}

void loop() {
    if (client.connected()) {
        //client.println("Hello from ESP8266");
        delay(10);
        //Serial.print(client.available());

        if (client.available()) {
            String response = client.readStringUntil(';');
            Serial.println("Server says: " + response);

            int pin;
            String state;
            if (parse_server_response(response, pin, state)) {
                toggle_pin(pin, state);
                client.println("OK");
            } else {
                Serial.println("Invalid response format");
                client.println("FAIL");
            }
        }
    } else {
        Serial.println("Disconnected from server. Reconnecting...");
        if (client.connect(host, port)) {
            Serial.println("Reconnected to server");
        }
        Serial.println("Waiting 4 s");
        delay(4000);
    }
}
