extern "C" {
    #include "board.h"
}

#include "sensors.h"


SensorArray::SensorArray() {
    sensors_detected = 0x00;
}

void SensorArray::i2c_write8(int16_t deviceAddress, uint8_t registerAddress, int16_t registerValue) {
    i2cWrite(deviceAddress, registerAddress, registerValue);
}

SensorArray sensors;