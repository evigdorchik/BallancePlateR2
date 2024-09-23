#include"TSC2007.h"

/*==============================================================================================================*
    CONSTRUCTOR
 *==============================================================================================================*/

TSC2007::TSC2007() {}

/*==============================================================================================================*
    DESTRUCTOR
 *==============================================================================================================*/

TSC2007::~TSC2007() {}

uint16_t TSC2007::command(tsc2007_function func,
                          tsc2007_power pwr,
                          tsc2007_resolution res) {
    uint8_t cmd = (uint8_t)func << 4;
    cmd |= (uint8_t)pwr << 2;
    cmd |= (uint8_t)res << 1;

    uint8_t reply[2];

    Wire.beginTransmission(DEV_ADDR); // transmit to device
    Wire.write(cmd);        // sends command (one byte)
    Wire.endTransmission();    // stop transmitting

    // Wait 1/2ms for conversion
    delayMicroseconds(500);

    // Reading data
    Wire.requestFrom(DEV_ADDR, 2);    // request 2 bytes from slave device

    uint8_t i = 0;

    while (Wire.available()) { // slave may send less than requested
      reply[i] = Wire.read(); // receive a byte
      i++;
    }

    return ((uint16_t)reply[0] << 4) | (reply[1] >> 4); // 12 bits

}