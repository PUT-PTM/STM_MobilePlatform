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
char Y=0;
char direction=0;//0-przod, 1-tyl, 2-lewo, 3-prawo


//tylko do podgladu
uint16_t srodek1=0;
uint16_t srodek2=0;
uint16_t prawo3=0;
uint16_t prawo2=0;
uint16_t prawo1=0;
uint16_t lewo1=0;
uint16_t lewo2=0;
uint16_t lewo3=0;

int PWM_Prawy;
int PWM_Lewy;


int Kp=100;
int Kd=650;
int Ki=100;
int docelowa=0;
int Tp=5000;
int aktualna=0;
int ile_czujnikow=0;
float aktualna_pozycja=0;
float error=0;
float zmiana=0;
float calka=0;
float ostatnia_wartosc=0;
int strona=0;


int16_t waga1=-60;
int16_t waga2=-30;
int16_t waga3=-20;
int16_t waga4=0;
int16_t waga5=0;
int16_t waga6=20;
int16_t waga7=30;
int16_t waga8=60;



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
				if(Y==0)
					direction=1;

				X=cmd[2]-48;
				index=0;
			}else
			{
				if(Y==0)
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

				Y=cmd[2]-48;
				index=0;
			}else
			{
				if(X==0)
					direction=3;

				Y=cmd[1]-48;
				index=0;
			}

		}break;
		case 'o':
		{
			GPIO_ToggleBits(GPIOC,GPIO_Pin_4);

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


void LineFollow()
{
	srodek1=valueFromADC[0];
	srodek2=valueFromADC[1];
	prawo3=valueFromADC[2];
	prawo2=valueFromADC[3];
	prawo1=valueFromADC[4];
	lewo1=valueFromADC[5];
	lewo2=valueFromADC[6];
	lewo3=valueFromADC[7];


		if(lewo3>1000)
			{
			aktualna=aktualna+waga1;
			ile_czujnikow++;
			strona=1;
			}
			if(lewo2>1000)
			{
			aktualna=aktualna+waga2;
			ile_czujnikow++;
			}
			if(lewo1>1000)
			{
			aktualna=aktualna+waga3;
			ile_czujnikow++;
			}
			if(srodek2>1000)
			{
			aktualna=aktualna+waga4;
			ile_czujnikow++;
			}
			if(srodek1>1000)
			{
			aktualna=aktualna+waga5;
			ile_czujnikow++;
			}
			if(prawo1>1000)
			{
			aktualna=aktualna+waga6;
			ile_czujnikow++;
			}
			if(prawo2>1000)
			{
			aktualna=aktualna+waga7;
			ile_czujnikow++;
			}
			if(prawo3>1000)
			{
			aktualna=aktualna+waga8;
			ile_czujnikow++;
			strona=2;
			}

//			if((aktualna==0)&&(srodek1<1000)||(srodek2<1000)){
//				TIM4->CCR1 = 0; // Przekazujemy do silnika prawego now¹ prêdkoœæ
//				TIM4->CCR2 = 0;
//			}

			aktualna_pozycja=(float)(aktualna/ile_czujnikow);
			error=docelowa+aktualna_pozycja;
			float zmiana_P=Kp*error;
			float zmiana_D=Kd*(error-ostatnia_wartosc);
			calka+=error;
			float zmiana_I=Ki*calka;

			zmiana=zmiana_P+zmiana_D+zmiana_I;
			GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_2);
			GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_3);

			ostatnia_wartosc=error;

			if(zmiana==0)
			{
				TIM4->CCR1 = 7500;
				TIM4->CCR2 = 7500;
			}else {
				TIM4->CCR1 = Tp - zmiana; // Przekazujemy do silnika prawego now¹ prêdkoœæ
				TIM4->CCR2 = Tp + zmiana;
			}



			ile_czujnikow=0;
			aktualna=0;
			aktualna_pozycja=0;
			error=0;
			calka=0;
			strona=0;



}

int main(void)
{
	SystemInit();

	UART_GPIOC_init(9600);
	PWM_Engine_init();
	Engine_Controll_Pins_init();
	//Engine_Off_Timer_init();


	DMA_initP2M();
	ADC_ScanMode_init();
	ADC_SoftwareStartConv(ADC1);
	
	//wlaczenie czujnikow IR
	GPIO_SetBits(GPIOB,GPIO_Pin_2);

	//wlaczeni silnikow
	GPIO_SetBits(GPIOC,GPIO_Pin_4);

	for(;;)
	{

		LineFollow();
		PWM_Lewy=TIM4->CCR2;
		PWM_Prawy=TIM4->CCR1;


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
 * todo PWM na inne piny
 *
 *
 * todo
 * calkiiii
 * regulacja
 * pamiec
 * dokumentacja
 */



