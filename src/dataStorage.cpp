#include "board.h"
#include "dataStorage.h"

union CONFIG_union CONFIG;

void initializeEEPROM() {
    // Default settings should be initialized here
    CONFIG.data.version = EEPROM_VERSION;
    CONFIG.data.calibrateESC = 0;
    CONFIG.data.minimumArmedThrottle = 1100;
    
    // Accelerometer
    CONFIG.data.ACCEL_BIAS[XAXIS] = 0;
    CONFIG.data.ACCEL_BIAS[YAXIS] = 0;
    CONFIG.data.ACCEL_BIAS[ZAXIS] = 0;
    
    // RX
    CONFIG.data.CHANNEL_ASSIGNMENT[0] = 0;
    CONFIG.data.CHANNEL_ASSIGNMENT[1] = 1;
    CONFIG.data.CHANNEL_ASSIGNMENT[2] = 2;
    CONFIG.data.CHANNEL_ASSIGNMENT[3] = 3;
    CONFIG.data.CHANNEL_ASSIGNMENT[4] = 4;
    CONFIG.data.CHANNEL_ASSIGNMENT[5] = 5;
    CONFIG.data.CHANNEL_ASSIGNMENT[6] = 6;
    CONFIG.data.CHANNEL_ASSIGNMENT[7] = 7;
    CONFIG.data.CHANNEL_ASSIGNMENT[8] = 8;
    CONFIG.data.CHANNEL_ASSIGNMENT[9] = 9;
    CONFIG.data.CHANNEL_ASSIGNMENT[10] = 10;
    CONFIG.data.CHANNEL_ASSIGNMENT[11] = 11;
    CONFIG.data.CHANNEL_ASSIGNMENT[12] = 12;
    CONFIG.data.CHANNEL_ASSIGNMENT[13] = 13;
    CONFIG.data.CHANNEL_ASSIGNMENT[14] = 14;
    CONFIG.data.CHANNEL_ASSIGNMENT[15] = 15;

    CONFIG.data.CHANNEL_FUNCTIONS[0] = 0x04; // mode select ("stable mode" is set to trigger on AUX1-HIGH by default)
    CONFIG.data.CHANNEL_FUNCTIONS[1] = 0x00; // baro select
    CONFIG.data.CHANNEL_FUNCTIONS[2] = 0x00; // sonar select
    CONFIG.data.CHANNEL_FUNCTIONS[3] = 0x00; // GPS select
    
    // Altitude
    CONFIG.data.PID_YAW_c[P]  = 4.0;
    CONFIG.data.PID_YAW_c[I]  = 0.0;
    CONFIG.data.PID_YAW_c[D]  = 0.0;
    CONFIG.data.PID_YAW_c[WG] = 25.0;
    
    CONFIG.data.PID_PITCH_c[P]  = 4.0;
    CONFIG.data.PID_PITCH_c[I]  = 0.0;
    CONFIG.data.PID_PITCH_c[D]  = 0.0;
    CONFIG.data.PID_PITCH_c[WG] = 25.0;

    CONFIG.data.PID_ROLL_c[P]  = 4.0;
    CONFIG.data.PID_ROLL_c[I]  = 0.0;
    CONFIG.data.PID_ROLL_c[D]  = 0.0;
    CONFIG.data.PID_ROLL_c[WG] = 25.0;

    // Rate
    CONFIG.data.PID_YAW_m[P]  = 200.0;
    CONFIG.data.PID_YAW_m[I]  = 5.0;
    CONFIG.data.PID_YAW_m[D]  = 0.0;
    CONFIG.data.PID_YAW_m[WG] = 100.0;
    
    CONFIG.data.PID_PITCH_m[P]  = 80.0;
    CONFIG.data.PID_PITCH_m[I]  = 0.0;
    CONFIG.data.PID_PITCH_m[D]  = -3.0;
    CONFIG.data.PID_PITCH_m[WG] = 1000.0;

    CONFIG.data.PID_ROLL_m[P]  = 80.0;
    CONFIG.data.PID_ROLL_m[I]  = 0.0;
    CONFIG.data.PID_ROLL_m[D]  = -3.0;
    CONFIG.data.PID_ROLL_m[WG] = 1000.0;    
    
    // Baro
    CONFIG.data.PID_BARO[P]  = 25.0;
    CONFIG.data.PID_BARO[I]  = 0.6;
    CONFIG.data.PID_BARO[D]  = -1.0;
    CONFIG.data.PID_BARO[WG] = 25.0;       
    
    // Sonar
    CONFIG.data.PID_SONAR[P]  = 50.0;
    CONFIG.data.PID_SONAR[I]  = 0.6;
    CONFIG.data.PID_SONAR[D]  = -1.0;
    CONFIG.data.PID_SONAR[WG] = 25.0;  

    // GPS
    CONFIG.data.PID_GPS[P] = 0.0;
    CONFIG.data.PID_GPS[I] = 0.0;
    CONFIG.data.PID_GPS[D] = 0.0;
    CONFIG.data.PID_GPS[WG] = 0.0;
    
    // This function will only initialize data variables
    // writeEEPROM() needs to be called manually to store this data in EEPROM
}

void writeEEPROM() {
    //empty for now
}

void readEEPROM() {
    initializeEEPROM();
}