/*!
 * @file HelloWorld.ino
 * @brief Show helloworld with cycling RGB hue.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd
 * @licence   The MIT License (MIT)
 * @maintainer [yangfeng](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-24
 * @url https://github.com/DFRobot/DFRobot_RGBLCD1602
 */

#include "DFRobot_RGBLCD1602.h"

// We won't use colorR, colorG, colorB as "fixed" values anymore.
// Instead, we'll adjust hue dynamically.
static unsigned char hue = 0;  // track our hue 0-255

DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x2D, /*lcdCols*/16, /*lcdRows*/2);  // 16 characters, 2 lines


void setup() {
  lcd.init();

  // Print a message to the LCD.
  lcd.print("hello, world!");
}

void loop() {
  // Move cursor to the second row
  lcd.setCursor(0, 1);
  // Display the number of seconds since Arduino reset
  lcd.print(millis() / 1000);

  // Increase hue by 10 each loop, wrap at 256
  hue = (hue + 1) % 90;
  hue = 120;

  // Use the new RGB values
  lcd.setHue(hue);
  //lcd.setColorWhite();

  delay(10);
}
