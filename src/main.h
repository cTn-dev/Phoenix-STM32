#pragma once

// Modulo definitions (integer remainder)
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;

bool armed = false;
bool flightMode = false;

// FlightController commands definitions
float commandYaw, commandYawAttitude, commandPitch, commandRoll, commandThrottle;

// Heading related variables
float headingError = 0.0;
float headingSetpoint = 0.0;

// PID variables
float YawCommandPIDSpeed, PitchCommandPIDSpeed, RollCommandPIDSpeed;
float YawMotorSpeed, PitchMotorSpeed, RollMotorSpeed, AltitudeHoldMotorSpeed;
int16_t throttle = 1000;

void process100HzTask();
void process50HzTask();
void process10HzTask();
void process1HzTask();