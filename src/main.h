#pragma once

// Modulo definitions (integer remainder)
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

// Main loop variables
extern unsigned long currentTime;
extern unsigned long previousTime;
extern unsigned long sensorPreviousTime;
extern uint8_t frameCounter;

extern bool armed;
extern bool flightMode;
extern bool altitudeHoldBaro;
extern bool altitudeHoldSonar;
extern bool positionHoldGPS;

// FlightController commands definitions
extern float commandYaw, commandYawAttitude, commandPitch, commandRoll, commandThrottle;

// Heading related variables
extern float headingError;
extern float headingSetpoint;

// PID variables
extern float YawCommandPIDSpeed, PitchCommandPIDSpeed, RollCommandPIDSpeed;
extern float YawMotorSpeed, PitchMotorSpeed, RollMotorSpeed, AltitudeHoldMotorSpeed;
extern int16_t throttle;

void process100HzTask();
void process50HzTask();
void process10HzTask();
void process1HzTask();

void reset_PID_integrals();