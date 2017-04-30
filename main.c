#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "funkcje.h"
#include "stm32f4xx_usart.h"

char cmd[5];
int index=0;

char X=0;
char os_Y=0;
char direction=0;//0-przod, 1-tyl, 2-lewo, 3-prawo


//tylko do podgladu
uint16_t value1=0;
uint16_t value2=0;
uint16_t value3=0;
uint16_t value4=0;
uint16_t value5=0;
uint16_t value6=0;
uint16_t value7=0;
uint16_t value8=0;


//sterowanie przez bluetooth
void USART3_IRQHandler(void)
{
	// sprawdzenie flagi zwiazanej z odebraniem danych przez USART
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		/////////////////////////////////////
		//TODO wsadzic kod do osobnej funkcji
		/////////////////////////////////////

		//reset timera wylaczajacego silniki
		TIM3->CNT=0;

		cmd[index]=USART_ReceiveData(USART3);
		if(++index>4)
			index=0;

		if(USART3->DR=='\n')
		{
		switch(cmd[0])
		{
		case 'x':
		{
			if(cmd[1]=='-')
			{
				if(os_Y==0)
					direction=1;

				X=cmd[2]-48;
				index=0;
			}else
			{
				if(os_Y==0)
					direction=0;

				X=cmd[1]-48;
				index=0;
			}

		}break;
		case 'y':
		{
			if(cmd[1]=='-')
			{
				if(X==0)
					direction=2;

				os_Y=cmd[2]-48;
				index=0;
			}else
			{
				if(X==0)
					direction=3;

				os_Y=cmd[1]-48;
				index=0;
			}

		}break;
		case 'o':
		{
			GPIO_ToggleBits(GPIOA,GPIO_Pin_4);

			//sprawdzenie stanu na diodach
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4))
			{
				TIM4->CCR3=TIM4->ARR;
			}else
			{
				TIM4->CCR3=0;
			}
			index=0;


		}break;

		default:
		{
			index=0;
		}break;
		}
		}
		////////////////////////////////////////
		////////////////////////////////////////

	}
}


void EXTI0_IRQHandler(void)
{
        if(EXTI_GetITStatus(EXTI_Line0) != RESET)
        {
         // miejsce na kod wywo³ywany w momencie wyst¹pienia przerwania

         // wyzerowanie flagi wyzwolonego przerwania
         EXTI_ClearITPendingBit(EXTI_Line0);
   	   	}
}

//wylaczenie silnikow
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_4);
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4))
				TIM4->CCR3=TIM4->ARR;
			else
				TIM4->CCR3=0;

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		}

	}

}

int main(void)
{
	SystemInit();
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


	DMA_initP2M();
	ADC_ScanMode_init();
	ADC_SoftwareStartConv(ADC1);
	
	//wlaczenie czujnikow IR
	GPIO_SetBits(GPIOB,GPIO_Pin_2);

	for(;;)
	{
		value1=valueFromADC[0];
		value2=valueFromADC[1];
		value3=valueFromADC[2];
		value4=valueFromADC[3];
		value5=valueFromADC[4];
		value6=valueFromADC[5];
		value7=valueFromADC[6];
		value8=valueFromADC[7];

	}

}


/*uzywane piny
 * pc0-1 sterowanie lewym silnikiem
 * pc2-3 sterowanie prawym silnikiem
 * pc4 	 wylaczenie/wlaczenie silnikow
 *
 * pc10 UART/bluetooth
 * pc11 UART/bluetooth
 *
 * TIM4
 * czestotliwosc 100hz
 * pd12	 PWM lewy silnik
 * pd13  PWM prawy silnik
 *
 * TIM3<zabezpieczenie przed utrata lacznosci>
 * Jest zerowany przy odebraniu znaku przez UART,
 * gdy nie otrzyma nic przez sekunde wylacza silniki
 *
 *
 * PINY ADC
 * pb2   wlaczenie czujnikow
 * pb0-1 glowne czujniki IR
 * pa0-6 boczne czujniki IR
 *
 * todo
 * czujnik odleglosci
 * sterowanie na podstawie czujnikow IR
 * todo PWM na inne piny
 */


