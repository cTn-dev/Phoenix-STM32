#pragma once

struct __attribute__((packed)) CONFIG_struct {
    uint8_t version;
    bool calibrateESC;
    uint16_t minimumArmedThrottle;
    
    // Accelerometer
    int16_t ACCEL_BIAS[3];    
    
    // RX
    uint8_t CHANNEL_ASSIGNMENT[8];
    uint16_t CHANNEL_FUNCTIONS[4];
    
    // Attitude
    float PID_YAW_c[4];
    float PID_PITCH_c[4];
    float PID_ROLL_c[4];
    
    // Rate
    float PID_YAW_m[4];
    float PID_PITCH_m[4];
    float PID_ROLL_m[4];    
    
    float PID_BARO[4];
    float PID_SONAR[4]; 
    
    // GPS
    float PID_GPS[4];
};

union CONFIG_union {
    struct CONFIG_struct data;
    uint8_t raw[sizeof(struct CONFIG_struct)];
} CONFIG;

void initializeEEPROM();
void writeEEPROM();
void readEEPROM();