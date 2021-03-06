#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define RADX10 (M_PI / 1800.0f) // 0.001745329252f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// +- PI normalization macro
#define NORMALIZE(x) do { if ((x) < -PI) (x) += 2 * PI; else if ((x) > PI) (x) -= 2 * PI; } while (0);

#define radians(deg) ((deg)*DEG_TO_RAD)
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

#define LEDG_GPIO GPIOB
#define LEDG_PIN  GPIO_Pin_3 // PB3 (LED) GREEN
#define LEDG_ON   digitalHi(LEDG_GPIO,LEDG_PIN)
#define LEDG_OFF  digitalLo(LEDG_GPIO,LEDG_PIN)

#define LEDR_GPIO GPIOB
#define LEDR_PIN  GPIO_Pin_4 // PB4 (LED) RED
#define LEDR_ON   digitalHi(LEDR_GPIO,LEDR_PIN)
#define LEDR_OFF  digitalLo(LEDR_GPIO,LEDR_PIN)

// Flight modes
#define RATE_MODE 0
#define ATTITUDE_MODE 1

// Axis definitions
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

// PID pseudo definitions
#define P  0 // Proportional
#define I  1 // Integral
#define D  2 // Derivative
#define WG 3 // WindupGuard

// Primary channel definitions
#define ROLL        0
#define PITCH       1
#define THROTTLE    2
#define YAW         3

#define GYROSCOPE_DETECTED      0x01
#define ACCELEROMETER_DETECTED  0x02
#define MAGNETOMETER_DETECTED   0x04
#define BAROMETER_DETECTED      0x08
#define GPS_DETECTED            0x10

#include "drv_system.h" // timers, delays, etc
#include "drv_i2c.h"
#include "drv_uart.h"