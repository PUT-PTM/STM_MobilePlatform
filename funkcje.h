#ifndef funkcje_h
#define funkcje_h
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"

#define ARRAYSIZE 8
#define ADC_1_ADDRESS_BASE 0x40012000
// ADC_DR = ADC regular Data Register
#define ADC_DR_ADDRESS_OFFSET 0x4C
volatile uint16_t valueFromADC[ARRAYSIZE];


void ButtonInterrupt();
void DMA_initP2M(void);
void ADC_ScanMode_init(void);
void PWM_Engine_init(void);
void Engine_Controll_init(void);
void Engine_Controller(char direction);
void InputCaptureDistanceSensor();

//komunikacja bluetooth
void UART_GPIOC_init(uint32_t baudRate);


#endif


