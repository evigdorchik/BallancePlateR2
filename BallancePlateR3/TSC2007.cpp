#include"TSC2007.h"

/*==============================================================================================================*
    CONSTRUCTOR
 *==============================================================================================================*/

PCA9536::PCA9536() {}

/*==============================================================================================================*
    DESTRUCTOR
 *==============================================================================================================*/

PCA9536::~PCA9536() {}

byte PCA9536::ping() {
    Wire.beginTransmission(DEV_ADDR);
    return Wire.endTransmission();
}

/*==============================================================================================================*
    INITIATE I2C COMMUNICATION
 *==============================================================================================================*/

byte PCA9536::MeasureX() {
    Wire.beginTransmission(DEV_ADDR);
    Wire.write(12);

}

uint16_t Adafruit_TSC2007::command(tsc2007_function func,
                                   tsc2007_power pwr,
                                   tsc2007_resolution res) {
    uint8_t cmd = (uint8_t)func << 4;
    cmd |= (uint8_t)pwr << 2;
    cmd |= (uint8_t)res << 1;

    uint8_t reply[2];

    Wire.beginTransmission(DEV_ADDR); // transmit to device
    Wire.write(cmd);        // sends command (one byte)
    Wire.endTransmission();    // stop transmitting

    if (wire.write(&cmd, 1)) {
    return 0;
    }

    // Wait 1/2ms for conversion
    delayMicroseconds(500);
    if (!i2c_dev->read(reply, 2)) {
    return 0;
    }

    return ((uint16_t)reply[0] << 4) | (reply[1] >> 4); // 12 bits

}