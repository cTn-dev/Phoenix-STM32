extern "C" {
    #include "main.h"
}
#include "pid.h"    

static void _putc(void *p, char c) {
    uartWrite(c);
}

void checkReflash() {
    uint32_t time, i = 0;
    time = millis();
    
    while ((millis() - time) < 2000) {
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

int main(void) {
    systemInit();
    init_printf(NULL, _putc);
    uartInit(115200);
    delay(100);
    checkReflash(); // emergency reflash
    
    uint8_t leds = 7;

    // main loop
    while (1) {
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

        while (uartAvailable()) {
            uint8_t c = uartRead();
            printf("Got (%c)\r\n", c);
            
            if (c == 'R') {
                systemReset(true); // reboot to bootloader
            }
        }

        delay(100);
    }
}
