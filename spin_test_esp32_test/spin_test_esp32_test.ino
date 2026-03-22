#define HAL_SENSOR_PIN 11 //7
#define HAL_POWER_PIN  12 //6
#define SPIN_LED_PIN   38

#define TX_PIN  39
#define RX_PIN  37 
#define Serial Serial1

const unsigned long BLINK_INTERVAL = 150;   // ms between LED toggles
const int BLINK_TOGGLES = 6;                // 3 blinks (on/off pairs)

bool lastSensorState;
bool blinking = false;
const unsigned long BLINK_DURATION = 10; // ms
unsigned long blinkStart = 0;

void startBlink()
{
    digitalWrite(SPIN_LED_PIN, HIGH);
    blinkStart = millis();
    blinking = true;
}

void updateBlink()
{
    if (!blinking) return;

    if (millis() - blinkStart >= BLINK_DURATION)
    {
        digitalWrite(SPIN_LED_PIN, LOW);
        blinking = false;
    }
}

void setup()
{
    Serial.begin(921600, SERIAL_8N1, TX_PIN, TX_PIN);
    while (!Serial) delay(1);

    Serial.println("Program start");

    pinMode(HAL_POWER_PIN, OUTPUT);
    pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);
    pinMode(SPIN_LED_PIN, OUTPUT);

    digitalWrite(HAL_POWER_PIN, LOW);  // power the sensor

    lastSensorState = digitalRead(HAL_SENSOR_PIN);
}

bool hasSpeedSensorPower = false;

bool isSpeedSensorConnected() {
    pinMode(HAL_SENSOR_PIN, INPUT_PULLDOWN);
    int readDown = digitalRead(HAL_SENSOR_PIN);

    pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);
    int readUp = digitalRead(HAL_SENSOR_PIN);

    speedSensorSetPower(hasSpeedSensorPower); // restart the pinMode pull up/down as it was before changing them

    if(readDown == 0 && readUp == 0) return true;  // magnet detected
    if(readDown == 1 && readUp == 1) return true;  // magnet not detected

    return false; // pull down should be 0 and pull up should 1
}


void speedSensorSetPower(bool enable) {
    hasSpeedSensorPower = enable;

    if(enable) {
        digitalWrite(HAL_POWER_PIN, LOW);
        pinMode(HAL_SENSOR_PIN, INPUT_PULLUP);
    } else {
        digitalWrite(HAL_POWER_PIN, HIGH);
        pinMode(HAL_SENSOR_PIN, INPUT_PULLDOWN);
    }
}


struct SensorEvent {
    uint8_t state;
    uint32_t dtUs;
};

#define MAX_EVENTS 128
SensorEvent events[MAX_EVENTS];
uint8_t eventCount = 0;


void loop()
{
    static uint32_t prevEdgeUs = 0;
    static uint32_t lastPrint = 0;

    bool currentState = digitalRead(HAL_SENSOR_PIN);

    if (lastSensorState != currentState) {
        uint32_t nowUs = micros();

        if (eventCount < MAX_EVENTS) {
            events[eventCount].state = currentState;
            events[eventCount].dtUs = nowUs - prevEdgeUs;
            eventCount++;
        }

        prevEdgeUs = nowUs;
    }

    lastSensorState = currentState;

    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();

        for (uint8_t i = 0; i < eventCount; i++) {
            Serial.printf("%3d us:%6lu ", events[i].state, events[i].dtUs);
            if (events[i].state == 0) Serial.println();
        }

        if (eventCount > 0) Serial.println();
        eventCount = 0;
    }
}