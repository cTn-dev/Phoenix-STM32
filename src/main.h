#pragma once

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;

void process100HzTask();
void process50HzTask();
void process10HzTask();
void process1HzTask();