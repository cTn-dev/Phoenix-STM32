extern "C" {
    #include "board.h"
}

#include "dataStorage.h"
#include "receiver.h"

volatile uint16_t RX[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
volatile uint16_t PPM_temp[RX_CHANNELS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // Temporary buffer

volatile uint16_t startPulse = 0;
volatile uint8_t  ppmCounter = RX_CHANNELS;
volatile uint16_t PPM_error = 0;

volatile uint8_t RX_signalReceived = 0;

bool failsafeEnabled = false;

void initializeReceiver() {
}

void RX_failSafe() {
    RX_signalReceived++; // if this flag reaches 10, an auto-descent routine will be triggered.
    
    if (RX_signalReceived > 10) {
        RX_signalReceived = 10; // don't let the variable overflow
        failsafeEnabled = true; // this ensures that failsafe will operate in attitude mode
        
        // Bear in mind that this is here just to "slow" the fall, if you have lets say 500m altitude,
        // this probably won't help you much (sorry).
        // This will slowly (-2 every 100ms) bring the throttle to 1000 (still saved in the PPM array)
        // 1000 = 0 throttle;
        // Descending from FULL throttle 2000 (most unlikely) would take about 1 minute and 40 seconds
        // Descending from HALF throttle 1500 (more likely) would take about 50 seconds
        RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] -= 2;
        
        if (RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] < 1000) {
            RX[CONFIG.data.CHANNEL_ASSIGNMENT[THROTTLE]] = 1000; // don't let the value fall below 1000
        }    
    } else {
        failsafeEnabled = false;
    }
}