#include <Arduino.h>
#include "esp_wifi.h"
#include <WiFi.h>
#include "Button2.h"
#include <HardwareSerial.h>


#define BUTTON_PIN1  2
#define BUTTON_PIN2  3
#define BUTTON_PIN3  4

Button2 button1;
Button2 button2;
Button2 button3;

#define Serial_print(x)    do { Serial.print(x); Serial1.print(x); } while (0)
#define Serial_println(x)  do { Serial.println(x); Serial1.println(x); } while (0)


void tap1(Button2& btn) {
    Serial_println("Starting 5s duty-cycled sleep loop");

    uint32_t t_start = millis();

    while (millis() - t_start < 5000) {
        // Go to light sleep for 200 ms
        esp_sleep_enable_timer_wakeup(10 * 1000);  // in microseconds
        esp_light_sleep_start();

        // Wake active period
        //Serial_println("Awake for 10 ms");
        delay(2);  // simulate activity
    }

    Serial_println("5s cycle complete.");
}

void tap2(Button2& btn) {
        static int current_freq = 240;  // default startup frequency

    if (current_freq == 240) {
        setCpuFrequencyMhz(80);
        current_freq = 80;
        Serial_println("CPU frequency set to 80 MHz");
    } else {
        setCpuFrequencyMhz(240);
        current_freq = 240;
        Serial_println("CPU frequency set to 240 MHz");
    }
}

void tap3(Button2& btn) {
  // Begin Wi-Fi scan
  Serial_println("Starting Wi-Fi scan...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);  // Disconnect and clear previous state
  delay(100);             // Let Wi-Fi hardware settle

  int n = WiFi.scanNetworks(false, true);  // sync scan, show hidden
  Serial_print("Scan done. Found ");
  Serial_print(n);
  Serial_println(" networks:");

  for (int i = 0; i < n; ++i) {
    Serial_print(i);
    Serial_print(": ");
    Serial_print(WiFi.SSID(i));
    Serial_print(" (RSSI: ");
    Serial_print(WiFi.RSSI(i));
    Serial_println(" dBm)");
  }

  // Fully power down Wi-Fi after scan
  WiFi.disconnect(true, true);  // Disconnect & clear config again
  delay(50);                    // Short delay for clean stop
  esp_wifi_stop();
  esp_wifi_deinit();
  WiFi.mode(WIFI_OFF);

  Serial_println("Wi-Fi fully disabled after scan.");
}


void setup() {
  // Start serial for debugging if needed
  //Serial.begin(115200);
  //while(!Serial) delay(1);

  Serial1.begin(115200, SERIAL_8N1, 11, 10);  // RX, TX

  Serial_println("Program Start!");

  button1.begin(BUTTON_PIN1);
  button1.setTapHandler(tap1);
  
  button2.begin(BUTTON_PIN2);
  button2.setTapHandler(tap2);

  button3.begin(BUTTON_PIN3);
  button3.setTapHandler(tap3);

  // Kill Wi-Fi completely
  //WiFi.mode(WIFI_OFF);
  //esp_wifi_stop();

  // Reduce CPU frequency
  //setCpuFrequencyMhz(80);
}

IRAM_ATTR void run_benchmark() {
    volatile int sink = 1;
    for (int i = 0; i < 10000; ++i) {
        sink *= 123;
    }
}

void loop() {
  static uint32_t lastTimePrint = 0;
  uint32_t now = millis();

if (now - lastTimePrint >= 400) {
    lastTimePrint = now;

    uint32_t t0 = micros();  // Start timing

    noInterrupts();
    run_benchmark();
    interrupts();

    uint32_t t1 = micros();  // End timing

    Serial_print(" | Duration (us): ");
    Serial_println(t1 - t0);
}
  //delay(1000);  // loop does nothing
  button1.loop();
  button2.loop();
  button3.loop();
}