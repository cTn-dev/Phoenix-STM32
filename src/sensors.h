#pragma once

class SensorArray {
    public:
        uint16_t sensors_detected;
        
        SensorArray();
        void initializeGyro();
        void readGyroSum();
        void evaluateGyro();
        void initializeAccel();
        void calibrateAccel();
        void readAccelSum();
        void evaluateAccel();
        void initializeMag();
        void readMag();
        void evaluateMag();
        void initializeBaro();
        void readBaroSum();
        void evaluateBaroAltitude();
        void initializeGPS();
        void readGPS();
        void evaluateGPS();
        
        // I2C stuff
        void i2c_write8 (int16_t deviceAddress, uint8_t registerAddress, int16_t registerValue);
};

extern SensorArray sensors;        