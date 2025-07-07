
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
//#include <Wire.h>

// Define pins for SDA and SCL
#define SDA_PIN 4
#define SCL_PIN 5

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
uint16_t Read_LM75(uint8_t address) {
    uint16_t Temp = 0;
    uint8_t read_d[2];

    for (int i = 0; i < 3; i++) { // Try up to 3 times
        i2c_start_condition();
        i2c_write_byte((address << 1) | 0x01); // Send address with read bit
        i2c_send_ack();

        read_d[0] = i2c_read_byte();
        i2c_send_ack();
        read_d[1] = i2c_read_byte();
        i2c_send_nack();

        i2c_stop_condition();

        Temp = (read_d[0] << 8) | read_d[1];
        Temp = 5 * (Temp >> 7) / 10;

        if (Temp <= 126) {
            return Temp;
        }
    }

    //Serial.println("LM75 read error");
    return -1;
}


// Wi-Fi credentials
const char* ssid = "kuhna-wifi";
const char* password = "";

// Furnace state
bool furnaceOn = false;
float temperatures[3] = {0.0, 0.0, -1.0}; // Placeholder temperatures

// Initialize temperature sensors
//Generic_LM75 temperature1(72);
//Generic_LM75 temperature2(74);

// Initialize web server on port 80
ESP8266WebServer server(80);

// Embedded HTML page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Furnace Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
            background-color: #f4f4f4;
            color: #333;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 100vh;
        }

        header {
            font-size: 2rem;
            margin-bottom: 20px;
            text-align: center;
        }

        .temperature-display {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
        }

        .temperature {
            background: #fff;
            padding: 15px;
            border: 1px solid #ccc;
            border-radius: 8px;
            text-align: center;
            box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }

        .status {
            font-size: 1.5rem;
            margin-bottom: 20px;
            color: green;
        }

        .status.off {
            color: red;
        }

        button {
            background-color: #007BFF;
            color: white;
            border: none;
            padding: 10px 20px;
            margin-bottom: 10px;
            font-size: 1rem;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s;
        }

        button:hover {
            background-color: #0056b3;
        }

        button:disabled {
            background-color: #cccccc;
            color: #666666;
            cursor: not-allowed;
            opacity: 0.7;
        }

        footer {
            margin-top: 20px;
            font-size: 0.9rem;
            color: #777;
        }
    </style>
</head>
<body>
    <header>Ogrevanje Kropa 84</header>

    <div class="temperature-display">
        <div class="temperature" id="temp1">Temp1: --°C</div>
        <div class="temperature" id="temp2">Temp2: --°C</div>
    </div>

    <div class="status" id="furnace-status">--</div>

    <button onclick="toggleFurnace()" id="toggle-btn">Turn ON</button>

    <button onclick="refresh()" id="refresh-btn">Osveži</button>

    <footer>Powered by ESP8266</footer>

    <script>
        function updateStatus(data) {
            document.getElementById('temp1').textContent = `Temperature 1: ${data.temp1 || '--'}°C`;
            document.getElementById('temp2').textContent = `Temperature 2: ${data.temp2 || '--'}°C`;

            const statusEl = document.getElementById('furnace-status');
            const buttonEl = document.getElementById('toggle-btn');

            if (data.isOn) {
                statusEl.textContent = 'Peč GORI';
                statusEl.classList.remove('off');
                buttonEl.textContent = 'Ugasni';
            } else {
                statusEl.textContent = 'Peč IZKLOPLJENA';
                statusEl.classList.add('off');
                buttonEl.textContent = 'Vklopi';
            }
        }

        async function fetchStatus() {
            const response = await fetch('/status');
            const data = await response.json();
            updateStatus(data);
        }

        async function toggleFurnace() {
            const toggleBtn = document.getElementById('toggle-btn');
            toggleBtn.disabled = true;
            
            try {
                await fetch('/toggle-furnace', { method: 'POST' });
                await fetchStatus();
            } catch {

            } finally {
                toggleBtn.disabled = false;
            }
           

            
        }

        async function refresh() {
            const refreshBtn = document.getElementById('refresh-btn');
            refreshBtn.textContent = "Osvežujem ...";
            refreshBtn.disabled = true;
            
            try {
                await fetchStatus();
            } finally {
                refreshBtn.textContent = "Osveži";
                refreshBtn.disabled = false;
            }
        }

        // Fetch initial data on page load
        fetchStatus();
    </script>
</body>
</html>
)rawliteral";

// Function to serve the embedded HTML
void handleRoot() {
  Serial.println("Serving root endpoint");
  server.send(200, "text/html", index_html);
}

void updateTemperatures() {
  temperatures[0] = 1;//temperature1.readTemperatureC();
  temperatures[1] = 1;//temperature2.readTemperatureC();
  Serial.print("Updated Temperature 1: ");
  Serial.println(temperatures[0]);
  Serial.print("Updated Temperature 2: ");
  Serial.println(temperatures[1]);
}

void handleStatus() {
  Serial.println("Handling status endpoint");
  updateTemperatures();

  StaticJsonDocument<200> json;
  json["isOn"] = furnaceOn;
  json["temp1"] = temperatures[0];
  json["temp2"] = temperatures[1];
  json["temp3"] = temperatures[2];

  String response;
  serializeJson(json, response);
  server.send(200, "application/json", response);
}

void handleToggleFurnace() {
  Serial.println("Handling toggle furnace endpoint");
  furnaceOn = !furnaceOn;
  setFurnaceLed(); 
  
  Serial.print("Furnace toggled to: ");
  Serial.println(furnaceOn ? "ON" : "OFF");

  handleStatus(); // Return updated status
}

void setupEndpoints() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/toggle-furnace", HTTP_POST, handleToggleFurnace);
  server.begin();
  Serial.println("HTTP server started");
}

void setupWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setFurnaceLed() {
  digitalWrite(LED_BUILTIN, furnaceOn ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  //Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  setFurnaceLed();
  i2c_port_initial();
  //setupWiFi();
  //setupEndpoints();
}

void loop() {
  //server.handleClient();

  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  
  digitalWrite(SDA_PIN, HIGH);
  delay(1);
  digitalWrite(SDA_PIN, LOW);

  digitalWrite(SCL_PIN, HIGH);
  delay(2);
  digitalWrite(SCL_PIN, LOW);

  for(int i=0; i<256; i++) {
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(Read_LM75(i));  
    //delay(2);
  }
  delay(2);
}
