#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// OLED display TWI address
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(-1);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("Start!");
  
  // initialize and clear display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.println("begin done!");
  //display.clearDisplay();
  display.display();

  Serial.println("init done!");

  // display a pixel in each corner of the screen
  display.drawPixel(0, 0, WHITE);
  display.drawPixel(127, 0, WHITE);
  display.drawPixel(0, 63, WHITE);
  display.drawPixel(127, 63, WHITE);

  // display a line of text
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(27,30);
  display.print("Hello, world!");

  // update display with all of the above graphics
  display.display();
  Serial.print("Displasy???");
}

void loop() {
  // put your main code here, to run repeatedly:

}
