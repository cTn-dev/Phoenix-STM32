#include "main.h"

typedef struct gpio_config_t {
    GPIO_TypeDef *gpio;
    uint16_t pin;
    GPIOMode_TypeDef mode;
} gpio_config_t;

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

static void cycleCounterInit(void) {
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void) {
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    
    return (ms * 1000) + (72000 - cycle_cnt) / 72;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void) {
    return sysTickUptime;
}

void systemInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    gpio_config_t gpio_cfg[] = {
      { LEDR_GPIO, LEDR_PIN, GPIO_Mode_Out_PP },
      { LEDG_GPIO, LEDG_PIN, GPIO_Mode_Out_PP },
    };
    uint8_t gpio_count = sizeof(gpio_cfg) / sizeof(gpio_cfg[0]);
    uint32_t i;

    // This is needed because some shit inside Keil startup fucks with SystemCoreClock, setting it back to 72MHz even on HSI.
    SystemCoreClockUpdate();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_I2C2 | RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_ClearFlag();

    // Make all GPIO in by default to save power and reduce noise
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Turn off JTAG port 'cause we're using the GPIO for leds
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    // Configure gpio
    for (i = 0; i < gpio_count; i++) {
        GPIO_InitStructure.GPIO_Pin = gpio_cfg[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode = gpio_cfg[i].mode;
        GPIO_Init(gpio_cfg[i].gpio, &GPIO_InitStructure);
    }

    LEDR_OFF
    LEDG_OFF

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Configure the rest of the stuff
    i2cInit(I2C2);

    // sleep for 100ms
    delay(100);
}

void delayMicroseconds(uint32_t us) {
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay(uint32_t ms) {
    while (ms--) {
        delayMicroseconds(1000);
    }
}

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader) {
    if (toBootloader) {
        *((uint32_t *)0x20004FF0) = 0xDEADBEEF; // 20KB STM32F103
    }
    
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}
