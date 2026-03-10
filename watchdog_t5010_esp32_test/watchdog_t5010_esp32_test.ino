#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <sys/time.h>

#define WAKE_PIN       6
#define DONE_PIN       10
#define OVERRIDE_PIN   9
#define DONE_PULSE_MS  5

// Persists across deep-sleep resets (RTC memory)
RTC_DATA_ATTR int64_t last_rtc_us = -1;
RTC_DATA_ATTR uint32_t bootCount  = 0;

static int64_t now_rtc_us() {
  timeval tv;
  gettimeofday(&tv, nullptr); // backed by RTC timer across deep sleep
  return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
}

static void send_done_pulse() {
  Serial.println("Sending DONE");
  digitalWrite(DONE_PIN, HIGH);
  delay(DONE_PULSE_MS);
  digitalWrite(DONE_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  bootCount++;
  Serial.printf("Boot #%lu\n", (unsigned long)bootCount);

  // GPIO setup
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);

  pinMode(OVERRIDE_PIN, INPUT_PULLUP); // button to GND (override active when LOW)

  // Measure time since last wake (RTC-based)
  int64_t now_us = now_rtc_us();
  if (last_rtc_us >= 0) {
    int64_t delta_us = now_us - last_rtc_us;
    Serial.printf("RTC elapsed: %.3f s (%lld us)\n", delta_us / 1e6, (long long)delta_us);
  } else {
    Serial.println("RTC elapsed: (first boot)");
  }
  last_rtc_us = now_us;

  // Why did we wake?
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake cause: %d\n", (int)cause);

  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by GPIO");
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by RTC timer");
      break;

    default:
      Serial.println("Other wakeup cause");
      break;
  }


  // If override button is pressed, skip DONE pulse
  bool override_active = (digitalRead(OVERRIDE_PIN) == LOW);
  if (override_active) {
    Serial.println("Override active (button pressed) -> skipping DONE pulse");
  } else {
    send_done_pulse();
  }

  // Configure EXT0 wake on WAKE_PIN.
  // ESP-IDF deep_sleep example: ESP32-S2 uses GPIO3 for EXT0; wake when pin is HIGH. :contentReference[oaicite:0]{index=0}
  // EXT0 uses RTC IO; configure pulls if needed.
  rtc_gpio_deinit((gpio_num_t)WAKE_PIN);
  rtc_gpio_pullup_dis((gpio_num_t)WAKE_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)WAKE_PIN);

#define SLEEP_TIME_SEC 20

  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_PIN, 1); // 1 = wake when HIGH :contentReference[oaicite:1]{index=1}
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_TIME_SEC * 1000000ULL);

  //Serial.println("Going to deep sleep now...");
  //delay(50);
  //esp_deep_sleep_start();
}

void loop() {
  unsigned long now = millis();
  static unsigned long lastCall = 0;

  if (now - lastCall >= 4*1000) {
    lastCall = now;
    
    bool override_active = (digitalRead(OVERRIDE_PIN) == LOW);
    if (override_active) {
      Serial.println("Override active (button pressed) -> skipping DONE pulse");
    } else {
      send_done_pulse();
    }
  }
}