#pragma once

#define PSP_SYNC1 0xB5
#define PSP_SYNC2 0x62

#define PSP_REQ_CONFIGURATION 1
#define PSP_REQ_GYRO_ACC      2
#define PSP_REQ_MAG           3
#define PSP_REQ_BARO          4
#define PSP_REQ_GPS           5
#define PSP_REQ_RC            6
#define PSP_REQ_KINEMATICS    7
#define PSP_REQ_MOTORS_OUTPUT 8
#define PSP_REQ_MOTORS_COUNT  9
#define PSP_REQ_SENSORS_ALIVE 10
#define PSP_REQ_AUX_TRIGGERED 11


#define PSP_SET_CONFIGURATION     101
#define PSP_SET_EEPROM_REINIT     102
#define PSP_SET_ACCEL_CALIBRATION 103
#define PSP_SET_MAG_CALIBRATION   104
#define PSP_SET_MOTOR_TEST_VALUE  105
#define PSP_SET_REBOOT            106


#define PSP_INF_ACK      201
#define PSP_INF_REFUSED  202
#define PSP_INF_CRC_FAIL 203

class Configurator {
    public:
        Configurator();
        void read_packet();
        void process_data();
        void protocol_head(uint8_t code, uint16_t length);
        void protocol_tail();
        void serialize_uint8(uint8_t data);
        void serialize_uint16(uint16_t data);
        void serialize_uint32(uint32_t data);
        void serialize_uint64(uint64_t data);
        void serialize_float32(float f);
        void ACK();
        void REFUSED();
        void CRC_FAILED(uint8_t code, uint8_t failed_crc);
        void send_UNION();
        
    private:
        uint8_t data; // variable used to store a single byte from serial
        
        uint8_t state;
        uint8_t code;
        uint8_t message_crc;
        uint8_t crc;
        
        uint16_t payload_length_expected;
        uint16_t payload_length_received;
        
        uint8_t data_buffer[200];
};

void readSerial();