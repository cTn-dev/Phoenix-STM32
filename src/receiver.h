#pragma once

// Defined by the user, can vary from 4 to 16 channels
#define RX_USER_CHANNELS_OPERATIONAL 8
#define RX_PPM_SYNCPULSE 12000 // 4ms >

#define RX_CHANNELS 16 // dont change this

extern volatile uint16_t RX[RX_CHANNELS];
extern volatile uint16_t PPM_temp[RX_CHANNELS]; // Temporary buffer

extern volatile uint16_t startPulse;
extern volatile uint8_t  ppmCounter;
extern volatile uint16_t PPM_error;

extern volatile uint8_t RX_signalReceived;

extern bool failsafeEnabled;

void initializeReceiver();
void RX_failSafe();