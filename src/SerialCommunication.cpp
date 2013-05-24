/*  PSP - Phoenix Serial Protocol
    
    Inspired by uBlox serial protocol and multiwii serial protocol
    
    Data structure is subject to change without further notice.  

    Protocol data structure:
    [SYNC1][SYNC2][CODE][LENGTH_L][LENGTH_H][DATA/DATA ARRAY][CRC]
*/

extern "C" {
    #include "board.h"
}

#include "dataStorage.h"
#include "serialCommunication.h"
#include "sensors.h"
#include "sensor_mpu6050.h"
#include "kinematics.h"

Configurator::Configurator() {
    state = 0;
    
    payload_length_expected = 0;
    payload_length_received = 0;
}

void Configurator::read_packet() {
    while (uartAvailable()) {
        data = uartRead();
        
        switch (state) {
            case 0:
                if (data == PSP_SYNC1) {
                    state++;
                }
                break;
            case 1:
                if (data == PSP_SYNC2) {
                    state++;
                } else {
                    state = 0; // Restart and try again
                }
                break;
            case 2:
                code = data;
                message_crc = data;
                
                state++;
                break;
            case 3: // LSB
                payload_length_expected = data;
                message_crc ^= data;
                
                state++;
                break;
            case 4: // MSB
                payload_length_expected |= data << 8;
                message_crc ^= data;
                
                state++;
                break;
            case 5:
                data_buffer[payload_length_received] = data;
                message_crc ^= data;
                payload_length_received++;
                
                if (payload_length_received >= payload_length_expected) {
                    state++;
                }
                break;
            case 6:
                if (message_crc == data) {
                    // CRC is ok, process data
                    process_data();
                } else {
                    // respond that CRC failed
                    CRC_FAILED(code, message_crc);
                }
                
                // reset variables
                memset(data_buffer, 0, sizeof(data_buffer));
                
                payload_length_received = 0;
                state = 0;
                break;
        }
    }   
}

void Configurator::process_data() {
    switch (code) {
        case PSP_REQ_CONFIGURATION:
            send_UNION();
            break;              
        case PSP_REQ_GYRO_ACC: {
            protocol_head(PSP_REQ_GYRO_ACC, 24);
            
            // gyro
            for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
                serialize_float32(gyro[axis]);
            }

            // accel
            float norm = sqrt(accel[XAXIS] * accel[XAXIS] + accel[YAXIS] * accel[YAXIS] + accel[ZAXIS] * accel[ZAXIS]);
            
            for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
                serialize_float32((float)(accel[axis] / norm));
            } 
            
            }
            break;                  
        case PSP_REQ_RC:
            /*
            protocol_head(PSP_REQ_RC, RX_CHANNELS * 2);
            
            for (uint8_t channel = 0; channel < RX_CHANNELS; channel++) {
                serialize_uint16(RX[channel]);
            }
            */
            break;
        case PSP_REQ_KINEMATICS:
            protocol_head(PSP_REQ_KINEMATICS, 12);

            for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
                serialize_float32(kinematicsAngle[axis]);
            }
            break;
        case PSP_REQ_MOTORS_OUTPUT:
            /*
            protocol_head(PSP_REQ_MOTORS_OUTPUT, MOTORS * 2);

            for (uint8_t motor = 0; motor < MOTORS; motor++) {
                serialize_uint16(MotorOut[motor]);
            }
            */
            break;
        case PSP_REQ_MOTORS_COUNT:
            protocol_head(PSP_REQ_MOTORS_COUNT, 1);
            
            //serialize_uint8(MOTORS);
            serialize_uint8(4); // temporary
            break;
        case PSP_REQ_SENSORS_ALIVE:
            protocol_head(PSP_REQ_SENSORS_ALIVE, 2);
            
            serialize_uint16(sensors.sensors_detected);
            break;
        case PSP_REQ_AUX_TRIGGERED:
            /*
            protocol_head(PSP_REQ_AUX_TRIGGERED, 8);
            
            serialize_uint64(AUX_chan_mask);
            */
            break;
         
        // SET
        case PSP_SET_CONFIGURATION:
            if (payload_length_received == sizeof(CONFIG)) {
                // process data from buffer (throw it inside union)
                for (uint16_t i = 0; i < sizeof(CONFIG); i++) {
                    CONFIG.raw[i] = data_buffer[i];
                }

                // Write config to EEPROM
                writeEEPROM();

                ACK();
            } else {
                // Refuse (buffer size doesn't match struct memory size)
                REFUSED();
            }
            break;
        case PSP_SET_EEPROM_REINIT:
            ACK();
            
            initializeEEPROM(); // initializes default values
            writeEEPROM(); // writes default values to eeprom

            // Send back configuration union                    
            send_UNION();                    
            break;
        case PSP_SET_ACCEL_CALIBRATION:
            sensors.calibrateAccel();
            
            // Write config to EEPROM
            writeEEPROM();

            // Send over the accel calibration data
            protocol_head(PSP_SET_ACCEL_CALIBRATION, 6);
            
            for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
                serialize_uint16(CONFIG.data.ACCEL_BIAS[axis]);
            }
            break; 
        case PSP_SET_MOTOR_TEST_VALUE:
            /*
            // data_buffer should contain 2 bytes (byte 0 = motor number, byte 1 = value)
            if (data_buffer[0] < MOTORS) { // Check if motor number is within our setup
                MotorOut[data_buffer[0]] = 1000 + (data_buffer[1] * 10);
                updateMotors(); // Update ESCs
            } else { // Motor number is not in our setup
                REFUSED();
            }
            */
            break;
        case PSP_SET_REBOOT:
            // In the future we might also utilize the true flag an allow a standard reboot.
            systemReset(true); // reboot to bootloader
            break;
        default: // Unrecognized code
            REFUSED();
    }
    
    // send over crc
    protocol_tail();
}

void Configurator::protocol_head(uint8_t code, uint16_t length) {
    uartWrite(PSP_SYNC1);
    uartWrite(PSP_SYNC2);

    crc = 0; // reset crc
    
    serialize_uint8(code);
    serialize_uint16(length);
}

void Configurator::protocol_tail() {
    uartWrite(crc);
}

void Configurator::serialize_uint8(uint8_t data) {
    uartWrite(data);
    crc ^= data;
}

void Configurator::serialize_uint16(uint16_t data) {
    serialize_uint8(lowByte(data));
    serialize_uint8(highByte(data));
}

void Configurator::serialize_uint32(uint32_t data) {
    for (uint8_t i = 0; i < 4; i++) {
        serialize_uint8((uint8_t) (data >> (i * 8)));
    }
}

void Configurator::serialize_uint64(uint64_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        serialize_uint8((uint8_t) (data >> (i * 8)));
    }
}

void Configurator::serialize_float32(float f) {
    uint8_t *b = (uint8_t*) & f;
    
    for (uint8_t i = 0; i < sizeof(f); i++) {
        serialize_uint8(b[i]);
    }
}

void Configurator::ACK() {
    protocol_head(PSP_INF_ACK, 1);
    
    serialize_uint8(0x01);
}

void Configurator::REFUSED() {
    protocol_head(PSP_INF_REFUSED, 1);
    
    serialize_uint8(0x00);
}

void Configurator::CRC_FAILED(uint8_t code, uint8_t failed_crc) {
    protocol_head(PSP_INF_CRC_FAIL, 2);
    
    serialize_uint8(code);
    serialize_uint8(failed_crc);
}

void Configurator::send_UNION() {
    protocol_head(PSP_REQ_CONFIGURATION, sizeof(CONFIG));

    for (uint16_t i = 0; i < sizeof(CONFIG); i++) {
        serialize_uint8(CONFIG.raw[i]);
    }
}

Configurator configurator;

void readSerial() {
    configurator.read_packet();
}