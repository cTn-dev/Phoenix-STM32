#pragma once

// USART1
void uartInit(uint32_t speed);
uint16_t uartAvailable(void);
bool uartTransmitEmpty(void);
uint8_t uartRead(void);
uint8_t uartReadPoll(void);
void uartWrite(uint8_t ch);
void uartPrint(char *str);

