extern "C" {
    #include "board.h"
}

#include "main.h"
#include "pid.h"
#include "dataStorage.h"
#include "serialCommunication.h"
#include "sensors.h"
#include "sensor_mpu6050.h"
#include "kinematics.h"

static void _putc(void *p, char c) {
    uartWrite(c);
}

void checkReflash() {
    uint32_t time, i = 0;
    time = millis();
    
    while ((millis() - time) < 6000) {
        delay(100);
        
        if ((i++)&1) {
            LEDG_ON;
        } else {
            LEDG_OFF;
        }

        if (uartAvailable() && ('R' == uartRead())) {
            systemReset(true); // reboot to bootloader
        }
    }
    
    LEDG_OFF;
}

// Global PID object definitions
PID yaw_command_pid;
PID pitch_command_pid;
PID roll_command_pid;

PID yaw_motor_pid;
PID pitch_motor_pid;
PID roll_motor_pid;

// Function to reset I terms inside PID objects
void reset_PID_integrals() {
    yaw_command_pid.IntegralReset();
    pitch_command_pid.IntegralReset();
    roll_command_pid.IntegralReset();
    
    yaw_motor_pid.IntegralReset();
    pitch_motor_pid.IntegralReset();
    roll_motor_pid.IntegralReset();    
}

int main(void) {
    systemInit();
    init_printf(NULL, _putc);
    uartInit(115200);
    delay(100);
    checkReflash(); // emergency reflash
    i2cInit(I2C2);
    
    // Read data from EEPROM to CONFIG union
    readEEPROM();

    // Initialize PID objects with data from EEPROM
    yaw_command_pid = PID(&headingError, &YawCommandPIDSpeed, &headingSetpoint, (float*) &CONFIG.data.PID_YAW_c);
    pitch_command_pid = PID(&kinematicsAngle[YAXIS], &PitchCommandPIDSpeed, &commandPitch, (float*) &CONFIG.data.PID_PITCH_c);
    roll_command_pid = PID(&kinematicsAngle[XAXIS], &RollCommandPIDSpeed, &commandRoll, (float*) &CONFIG.data.PID_ROLL_c);
    
    yaw_motor_pid = PID(&gyro[ZAXIS], &YawMotorSpeed, &YawCommandPIDSpeed, (float*) &CONFIG.data.PID_YAW_m);
    pitch_motor_pid = PID(&gyro[YAXIS], &PitchMotorSpeed, &PitchCommandPIDSpeed, (float*) &CONFIG.data.PID_PITCH_m);
    roll_motor_pid = PID(&gyro[XAXIS], &RollMotorSpeed, &RollCommandPIDSpeed, (float*) &CONFIG.data.PID_ROLL_m);      
    
    // Initialize motors/receivers/sensors
    sensors.initializeGyro();
    sensors.initializeAccel();    
    
    // main loop
    while (1) {
        // Timer
        currentTime = micros();   

        // Read data (not faster then every 1 ms)
        if (currentTime - sensorPreviousTime >= 1000) {
            sensors.readGyroSum();
            sensors.readAccelSum();     
            
            sensorPreviousTime = currentTime;
        }
    
        // 100 Hz task loop (10 ms)
        if (currentTime - previousTime > 10000) {
            frameCounter++;
            
            process100HzTask();
            
            // 50 Hz tak (20 ms)
            if (frameCounter % TASK_50HZ == 0) {
                process50HzTask();
            }
            
            // 10 Hz task (100 ms)
            if (frameCounter % TASK_10HZ == 0) {
                process10HzTask();
            }  
            
            // 1 Hz task (1000 ms)
            if (frameCounter % TASK_1HZ == 0) {
                process1HzTask();
            }
            
            // Reset frameCounter back to 0 after reaching 100 (1s)
            if (frameCounter >= 100) {
                frameCounter = 0;
            }
            
            previousTime = currentTime;
        }
    }
}

void process100HzTask() {
    sensors.evaluateGyro();
    sensors.evaluateAccel();
    
    // Listens / read Serial commands on Serial1 interface (used to pass data from configurator)
    readSerial();
    
    // Update kinematics with latest data
    kinematics_update(gyro[XAXIS], gyro[YAXIS], gyro[ZAXIS], accel[XAXIS], accel[YAXIS], accel[ZAXIS]);    
    
    // Update heading
    headingError = kinematicsAngle[ZAXIS] - commandYawAttitude;
    NORMALIZE(headingError); // +- PI
    
    // Update PIDs according the selected mode
    if (flightMode == ATTITUDE_MODE) {
        // Compute command PIDs (with kinematics correction)
        yaw_command_pid.Compute();
        pitch_command_pid.Compute();
        roll_command_pid.Compute();
    } else if (flightMode == RATE_MODE) {
        // Stick input, * 4.0 is the rotation speed factor
        YawCommandPIDSpeed = commandYaw * 4.0;
        PitchCommandPIDSpeed = commandPitch * 4.0;
        RollCommandPIDSpeed = commandRoll * 4.0;        
    }   
    
    // Compute motor PIDs (rate-based)    
    yaw_motor_pid.Compute();
    pitch_motor_pid.Compute();
    roll_motor_pid.Compute();  

    // This is the place where the actual "force" gets applied
    if (armed) {
        // TODO
        
        /*
        updateMotorsMix(); // Frame specific motor mix
        updateMotors(); // Update ESCs
        */
    }     
}

void process50HzTask() {
    static uint8_t leds = 7;
    
    if (leds&8) {
        LEDR_ON;
    } else {
        LEDR_OFF;
    }
    if (leds&16) {
        LEDG_ON;
    } else {
        LEDG_OFF;
    }
    
    leds <<= 1;
    if (!leds) leds = 7;
}

void process10HzTask() {
}

void process1HzTask() {
}
