//
//    FILE: AS5600_position.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include "AS5600.h"
#include "TLx493D_inc.hpp"

using namespace ifx::tlx493d;

TLx493D_A1B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_A1B6 dut2(Wire1, TLx493D_IIC_ADDR_A0_e);


//  Uncomment the line according to your sensor type
//AS5600L as5600;   //  use default Wire
AS5600 as5600;   //  use default Wire


void setup()
{
  Serial1.begin(115200, SERIAL_8N1, 7, 9);
  Serial.begin(115200);
  Serial.print("Hello world");
  Serial1.println(__FILE__);
  Serial1.print("AS5600_LIB_VERSION: ");
  Serial1.println(AS5600_LIB_VERSION);
  Serial1.println();

  Wire.begin();
  Wire1.begin(37, 39, 4000000);

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  Serial1.println(as5600.getAddress());

  //  as5600.setAddress(0x40);  //  AS5600L only

  int b = as5600.isConnected();
  Serial1.print("Connect: ");
  Serial1.println(b);

  dut.begin();
  dut2.begin();

  delay(1000);
}


void loop()
{
  static uint32_t lastTime = 0;

  //  set initial position
  as5600.getCumulativePosition();

  //  update every 100 ms
  //  should be enough up to ~200 RPM
  if (millis() - lastTime >= 10)
  {
    lastTime = millis();
    float dir = as5600.readAngle() * 10 / 4096.0;
    double t, x, y, z;
    dut.getMagneticFieldAndTemperature(&x, &y, &z, &t);

    double t2, x2, y2, z2;
    dut2.getMagneticFieldAndTemperature(&x2, &y2, &z2, &t2);

    Serial1.printf("dir:%08.2f T:%08.4f X:%08.4f Y:%08.4f Z:%08.4f T2:%08.4f X2:%08.4f Y2:%08.4f Z2:%08.4f max:10 min:-1\r\n", dir, t, x, y, z, t2, x2, y2, z2);
    Serial.printf( "dir:%08.2f T:%08.4f X:%08.4f Y:%08.4f Z:%08.4f T2:%08.4f X2:%08.4f Y2:%08.4f Z2:%08.4f max:10 min:-1\r\n", dir, t, x, y, z, t2, x2, y2, z2);
  }
}


//  -- END OF FILE --
