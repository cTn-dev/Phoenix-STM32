extern "C" {
    #include "board.h"
}

#include "receiver.h"

volatile uint16_t RX[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
volatile uint16_t PPM_temp[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Temporary buffer

volatile uint16_t startPulse = 0;
volatile uint8_t  ppmCounter = RX_CHANNELS;
volatile uint16_t PPM_error = 0;

volatile uint8_t RX_signalReceived = 0;

bool failsafeEnabled = false;