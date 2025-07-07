/*void setup() {
  // A5 on the Arduino Uno corresponds to PC5.
  // Set PC5 as an output by setting bit 5 of DDRC.
  DDRC |= (1 << DDC5);  
}

void loop() {
  // Toggle A5 (PC5) as fast as possible.
  // The XOR operation flips the bit, producing a fast square wave.
  PORTC ^= (1 << PORTC5);
  DDRC |= (1 << DDC5);  
  DDRC |= (1 << DDC5);  
}
*/
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"

#define PIN 15

void setup() {
    pinMode(PIN, OUTPUT);
}

int i = 0;
void loop() {
    while (i++ < 1000000) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN)); // Set PIN HIGH
        delayMicroseconds(1);
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN)); // Set PIN LOW
        delayMicroseconds(5);
    }
    i = 0;

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN)); // Set PIN HIGH
    delay(1000);
}
