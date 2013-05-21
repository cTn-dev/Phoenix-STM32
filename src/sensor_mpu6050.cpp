/*  10 DOF stick featuring MPU6050, HMC5883L, MS5611

    According to MPU6050 datasheet, this chip should support I2C speeds up to 400kHz (Fast-mode Fm)
    
    However on Teensy 3.0 i am able to reach 2.4MHz (High-speed mode) without any problems.
    (which cuts down the reading time of accel + gyro to about 180us)
    
    Please note that external pullup resistors are required on Teensy 3.0,
    while they are not "required" on other platforms, i highly recommend adding them.
    1000 ohm pullup seems to work best in my case.
*/
#include "main.h"
#include "sensor_mpu6050.h"

class MPU6050 {
    public: 
        // Constructor
        MPU6050() {
            // Range = +-1000 dps
            // Scale factor = positive range / positive sensitivity
            // or you can use full range / full sensitivity, which will result in the same output.
            gyroScaleFactor = radians(1000.0 / 32768.0); // 0.030517578125
            
            // Accel scale factor = 9.81 m/s^2 / scale
            accelScaleFactor = 9.81 / 8192.0; // 0.001197509765625
            
            gyroSamples = 0;
            accelSamples = 0;
        };

        void initialize() {
            // Chip reset
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_1, BIT_H_RESET);
            
            // Startup delay 
            delay(100);  
            
            // Check if sensor is alive
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_WHOAMI);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 1);
            
            uint8_t register_value = Wire.read();
            
            if (register_value == 0x68) {
                sensors.sensors_detected |= GYROSCOPE_DETECTED;
                sensors.sensors_detected |= ACCELEROMETER_DETECTED;
            } else {
                return;
            }            
            
            // Enable auxiliary I2C bus bypass
            // *NOT* Necessary for all setups, but some boards have magnetometer attached to the auxiliary I2C bus
            // and without this settings magnetometer won't be accessible.
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_INT_PIN_CFG, 0x02); // I2C _BYPASS _EN 1
            
            // Wake Up device and select GyroZ clock (better performance)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_2, 0);    
            
            // Sample rate = 1kHz 
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_SMPLRT_DIV, 0x00);

            // FS & DLPF, FS = 1000 degrees/s (dps), DLPF = 42Hz (low pass filter)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_CONFIG, BITS_DLPF_CFG_42HZ); 

            // Gyro scale 1000 degrees/s (dps)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
            
            // Accel scale +-4g (8192LSB/g)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_ACCEL_CONFIG, 0x08);    

            // Initial delay after proper configuration
            // let sensors heat up (especially gyro)
            delay(1500);
        };
        
        // ~1280ms (in case of error ~ms = calibration sanity passed)
        void calibrate_gyro() {
            static uint8_t retry = 0;
            uint8_t i, count = 128;
            int16_t xSum = 0, ySum = 0, zSum = 0;

            for (i = 0; i < count; i++) {
                readGyroRaw();
                xSum += gyroRaw[XAXIS];
                ySum += gyroRaw[YAXIS];
                zSum += gyroRaw[ZAXIS];
                delay(10);
            }
            
            gyro_offset[XAXIS] = -xSum / count;
            gyro_offset[YAXIS] = -ySum / count;
            gyro_offset[ZAXIS] = -zSum / count; 
            
            // Calibration sanity check
            // if suitable offset couldn't be established, break out of the loop after 10 retries
            if ((abs(gyro_offset[XAXIS]) > 300 || abs(gyro_offset[YAXIS]) > 300 || abs(gyro_offset[ZAXIS]) > 300) && retry < 10) {
                // gyro calibration failed, run again
                retry++;
                
                // small delay before next gyro calibration
                delay(500);
                
                calibrate_gyro();
            }
        };
        
        // ~1280ms (only runs when requested)
        void calibrate_accel() {
            uint8_t i, count = 128;
            int32_t xSum = 0, ySum = 0, zSum = 0;

            for (i = 0; i < count; i++) {
                readAccelRaw();
                xSum += accelRaw[XAXIS];
                ySum += accelRaw[YAXIS];
                zSum += accelRaw[ZAXIS];
                delay(10);
            }
            
            CONFIG.data.ACCEL_BIAS[XAXIS] = xSum / count;
            CONFIG.data.ACCEL_BIAS[YAXIS] = ySum / count;
            CONFIG.data.ACCEL_BIAS[ZAXIS] = (zSum / count) - 8192; // - 1G;

            // Reverse calibration forces
            CONFIG.data.ACCEL_BIAS[XAXIS] *= -1;
            CONFIG.data.ACCEL_BIAS[YAXIS] *= -1;
            CONFIG.data.ACCEL_BIAS[ZAXIS] *= -1;
        };

        // Order and +- signs of each axis depends on the chip orientation.
        // Default order: +X, +Y, +Z
        void readGyroRaw() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_GYRO_XOUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 6);
            
            gyroRaw[YAXIS] = (Wire.read() << 8) | Wire.read();
            gyroRaw[XAXIS] = (Wire.read() << 8) | Wire.read();
            gyroRaw[ZAXIS] = (Wire.read() << 8) | Wire.read();
        };

        // Order and +- signs of each axis depends on the chip orientation.
        // Default order: -X, +Y, +Z        
        void readAccelRaw() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_ACCEL_XOUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 6);
            
            accelRaw[YAXIS] = -((Wire.read() << 8) | Wire.read());
            accelRaw[XAXIS] = (Wire.read() << 8) | Wire.read(); 
            accelRaw[ZAXIS] = (Wire.read() << 8) | Wire.read();
        };        
        
        void readGyroSum() {
            readGyroRaw();
            
            gyroSum[XAXIS] += gyroRaw[XAXIS];
            gyroSum[YAXIS] += gyroRaw[YAXIS];
            gyroSum[ZAXIS] += gyroRaw[ZAXIS];
            
            gyroSamples++;
        };        
        
        void readAccelSum() {
            readAccelRaw();
            
            accelSum[XAXIS] += accelRaw[XAXIS];
            accelSum[YAXIS] += accelRaw[YAXIS];
            accelSum[ZAXIS] += accelRaw[ZAXIS];  

            accelSamples++;
        };
        
        void evaluateGyro() {
            // Calculate average
            gyro[XAXIS] = gyroSum[XAXIS] / gyroSamples;
            gyro[YAXIS] = gyroSum[YAXIS] / gyroSamples;
            gyro[ZAXIS] = gyroSum[ZAXIS] / gyroSamples;    
            
            // Apply offsets
            gyro[XAXIS] += gyro_offset[XAXIS];
            gyro[YAXIS] += gyro_offset[YAXIS];
            gyro[ZAXIS] += gyro_offset[ZAXIS];         
            
            // Apply correct scaling (at this point gyro is in radians)
            gyro[XAXIS] *= gyroScaleFactor;
            gyro[YAXIS] *= gyroScaleFactor;
            gyro[ZAXIS] *= gyroScaleFactor;
            
            // Reset SUM variables
            gyroSum[XAXIS] = 0;
            gyroSum[YAXIS] = 0;
            gyroSum[ZAXIS] = 0;
            gyroSamples = 0;            
        };
        
        void evaluateAccel() {
            // Calculate average
            accel[XAXIS] = accelSum[XAXIS] / accelSamples;
            accel[YAXIS] = accelSum[YAXIS] / accelSamples;
            accel[ZAXIS] = accelSum[ZAXIS] / accelSamples;  
            
            // Apply offsets
            accel[XAXIS] += CONFIG.data.ACCEL_BIAS[XAXIS];
            accel[YAXIS] += CONFIG.data.ACCEL_BIAS[YAXIS];
            accel[ZAXIS] += CONFIG.data.ACCEL_BIAS[ZAXIS];
            
            // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
            accel[XAXIS] *= accelScaleFactor;
            accel[YAXIS] *= accelScaleFactor;
            accel[ZAXIS] *= accelScaleFactor;
            
            // Reset SUM variables
            accelSum[XAXIS] = 0;
            accelSum[YAXIS] = 0;
            accelSum[ZAXIS] = 0;
            accelSamples = 0;
        };
        
        void readGyroTemperatutre() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_TEMP_OUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 2);
            
            gyro_temperature = (Wire.read() << 8) | Wire.read();         
        };
    private:
        int16_t gyro_offset[3];
        
        float gyroScaleFactor;
        float accelScaleFactor; 

        int16_t gyroRaw[3];
        float gyroSum[3];

        int16_t accelRaw[3];
        float accelSum[3];
        
        uint8_t gyroSamples;
        uint8_t accelSamples;        
};

MPU6050 mpu;

void SensorArray::initializeGyro() {
    mpu.initialize();
    mpu.calibrate_gyro();
}

void SensorArray::initializeAccel() {
}

void SensorArray::calibrateAccel() {
    mpu.calibrate_accel();
}

void SensorArray::readGyroSum() {
    mpu.readGyroSum();
}

void SensorArray::readAccelSum() {
    mpu.readAccelSum();
}

void SensorArray::evaluateGyro() {
    mpu.evaluateGyro();
}

void SensorArray::evaluateAccel() {
    mpu.evaluateAccel();
}
