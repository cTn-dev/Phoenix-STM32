extern "C" {
    #include "board.h"
}

#include "main.h"
#include "dataStorage.h"
#include "SerialCommunication.h"

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

// Modulo definitions (integer remainder)
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100

int main(void) {
    systemInit();
    init_printf(NULL, _putc);
    uartInit(115200);
    delay(100);
    checkReflash(); // emergency reflash
    
    // Read data from EEPROM to CONFIG union
    readEEPROM();

    // main loop
    while (1) {
        // Timer
        currentTime = micros();   

        // Read data (not faster then every 1 ms)
        if (currentTime - sensorPreviousTime >= 1000) {
            // read stuff goes here
            
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
    // Listens/read Serial commands on Serial1 interface (used to pass data from configurator)
    readSerial();
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
    /*
    while (uartAvailable()) {
        uint8_t c = uartRead();
        printf("Got (%c)\r\n", c);
        
        if (c == 'R') {
            systemReset(true); // reboot to bootloader
        }
    }
    */
}

void process1HzTask() {
}
