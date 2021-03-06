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
#include "stm32f4xx_dac.h"



extern const u8 rawData[54174];//slawomir
extern const u8 rawData2[96078];//pyrpyr

char ActiveMode = 1;//0-LineFollower, 1-Manual
char cmd[5];//bluetooth/uart commands
int index=0;//

char X=0;//forward/backward value from bluetooth
char Y=0;//left/right value from bluetooth
char direction=0;//0-forward, 1-backward, 2-left, 3-right


uint16_t timer2cnt=0;

// more than 1000 means black line
uint16_t sensorThreshold=1000;
//sensors
uint16_t right3=0;
uint16_t right2=0;
uint16_t right1=0;
uint16_t middle1=0;
uint16_t middle2=0;
uint16_t left1=0;

uint16_t left2=0;
uint16_t left3=0;

//values for PD alghoritm
int Kp=100;
int Kd=800;
int Tp=8000;
int currentPosition=0;
int sensorsCounter=0;
int error=0;
int16_t PWM_change=0;
int lastValue=0;
//if all sensors don't see the line
//0 - left, 1 - right
int lastKnownPosition=0;

//values of each sensor
int16_t left3SensorValue=-45;
int16_t left2SensorValue=-30;
int16_t left1SensorValue=-15;
int16_t middle2SensorValue=0;
int16_t middle1SensorValue=0;
int16_t right1SensorValue=15;
int16_t right2SensorValue=30;
int16_t right3SensorValue=45;


//ultrawave sensor values
char ultrawaveSensorEnabled=1;
uint16_t inputCaptureValue=0;
uint16_t inputCaptureValue2=0;
uint16_t ultrawavesensorHightime;
uint16_t distance=0;

//speaker values
u8 dac;
int array_index;
int volume=1;
char soundEnabled=0;
char sound=1;

void moveForward(){
	GPIO_SetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_4);
	GPIO_ResetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_5);
}
void moveBackward(){
	GPIO_SetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_5);
	GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_4);
}
void moveLeft(){
	GPIO_SetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_4);
	GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_5);
}
void moveRight(){
	GPIO_SetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_5);
	GPIO_ResetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_4);
}


//BluetoothControl
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{

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
					direction=0;

				X=cmd[2]-48;
				index=0;
			}else
			{
				if(Y==0)
					direction=1;

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
			GPIO_ToggleBits(GPIOE,GPIO_Pin_6);

			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6))
			{
				TIM4->CCR3=TIM4->ARR;
			}else
			{
				TIM4->CCR3=0;
			}
			index=0;


		}break;
		case 'L'://linefollower
		{
			ActiveMode=0;
			index=0;
			break;
		}
		case 'B'://bluetooth
		{
			ActiveMode=1;
			index=0;
			break;
		}
		case 'D'://distancesensor
		{
			if(ultrawaveSensorEnabled==0)
			{
				ultrawaveSensorEnabled=1;
			}else
			{
				ultrawaveSensorEnabled=0;
			}
			index=0;
			break;
		}
		case 'S':
		{
			TIM2->CNT=0;
			TIM_Cmd(TIM2,ENABLE);
			sound=1;
			array_index=0;
			index=0;
			break;
		}
		case 'T':
		{
			if(soundEnabled==0){
				TIM_Cmd(TIM2,ENABLE);
				soundEnabled=1;
				TIM2->CNT=0;
				sound=0;
				array_index=0;
			}
			else {
				soundEnabled=0;
				TIM_Cmd(TIM2,DISABLE);
				TIM2->CNT=0;
				array_index=0;

			}
			index=0;
			break;
		}
		default:
		{
			index=0;
		}break;
		}
		}


	}
}


void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{

		if(ultrawaveSensorEnabled)
		{
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
			inputCaptureValue=TIM_GetCapture2(TIM3);
		else
			inputCaptureValue2=TIM_GetCapture2(TIM3);

		ultrawavesensorHightime=inputCaptureValue2-inputCaptureValue;
		distance=(ultrawavesensorHightime*34)/1000/2;
		}else
		{
			distance=0;
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}

}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{

		switch(sound)
		{
		case 0:
		{
			dac=rawData2[array_index]*1.3;
			DAC_SetChannel1Data(DAC_Align_12b_R,dac);
			array_index++;
			if(array_index==96078)
			{
				array_index=0;
				TIM2->CNT=0;
				TIM_Cmd(TIM2,DISABLE);
			}
			break;
		}case 1:
		{
			dac=rawData[array_index]*1.3;
			DAC_SetChannel1Data(DAC_Align_12b_R,dac);
			array_index++;
			if(array_index==54068)
			{
				array_index=0;
				TIM2->CNT=0;
				TIM_Cmd(TIM2,DISABLE);
			}
			break;
		}
		default:
		{
			break;
		}
		}


		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}


void FollowLine()
{
	middle1=valueFromADC[0];
	middle2=valueFromADC[1];
	right3=valueFromADC[2];
	right2=valueFromADC[3];
	right1=valueFromADC[4];
	left1=valueFromADC[5];
	left2=valueFromADC[6];
	left3=valueFromADC[7];


		if(left3>1000)
			{
			currentPosition=currentPosition+left3SensorValue;
			sensorsCounter++;
			lastKnownPosition=0;
			}
			if(left2>sensorThreshold)
			{
			currentPosition=currentPosition+left2SensorValue;
			sensorsCounter++;
			lastKnownPosition=0;
			}
			if(left1>sensorThreshold)
			{
			currentPosition=currentPosition+left1SensorValue;
			sensorsCounter++;
			}
			if(middle2>sensorThreshold)
			{
			currentPosition=currentPosition+middle1SensorValue;
			sensorsCounter++;
			}
			if(middle1>sensorThreshold)
			{
			currentPosition=currentPosition+middle2SensorValue;
			sensorsCounter++;
			}
			if(right1>sensorThreshold)
			{
			currentPosition=currentPosition+right1SensorValue;
			sensorsCounter++;
			}
			if(right2>sensorThreshold)
			{
			currentPosition=currentPosition+right2SensorValue;
			sensorsCounter++;
			lastKnownPosition=1;
			}
			if(right3>sensorThreshold)
			{
			currentPosition=currentPosition+right3SensorValue;
			sensorsCounter++;
			lastKnownPosition=1;
			}


			error=(int)(currentPosition/sensorsCounter);
			int P_controller=Kp*error;
			int D_controller=Kd*(error-lastValue);


			PWM_change=P_controller+D_controller;
			lastValue=error;

			if(sensorsCounter==0){
				if(lastKnownPosition==0)
				{
				TIM4->CCR1 = 0;
				TIM4->CCR2 = Tp;
				}else
				{
					TIM4->CCR1 = Tp;
					TIM4->CCR2 = 0;
				}
			}else{


			if(PWM_change>=0)
			{
				if(PWM_change>=Tp)
				{
					TIM4->CCR1 = Tp;
					TIM4->CCR2 = 0;
				}else
				{
					TIM4->CCR1 = Tp;
					TIM4->CCR2 = Tp - PWM_change;
				}
			}else {
				if(PWM_change<=-Tp)
				{
					TIM4->CCR1 = 0;
					TIM4->CCR2 = Tp;
				}else
				{
					TIM4->CCR1 = Tp + PWM_change;
					TIM4->CCR2 = Tp;
				}
			}

			}

			sensorsCounter=0;
			currentPosition=0;



}

void BluetoothControl()
{
	switch(direction){
		case 0:
		{
			moveForward();
			TIM4->CCR1=X*1999;
			TIM4->CCR2=X*1999;
			break;
		}
		case 1:
		{
			moveBackward();
			TIM4->CCR1=X*1999;
			TIM4->CCR2=X*1999;
			break;
		}
		case 2:
		{
			moveLeft();
			TIM4->CCR1=Y*1999;
			TIM4->CCR2=Y*1999;
			break;
		}
		case 3:
		{
			moveRight();
			TIM4->CCR1=Y*1999;
			TIM4->CCR2=Y*1999;
			break;
		}
}
}



int main(void)
{
	SystemInit();

	UART_GPIOC_init(9600);
	PWM_Engine_init();
	Engine_Control_Pins_init();

	DMA_initP2M();
	ADC_ScanMode_init();
	ADC_SoftwareStartConv(ADC1);

	//IR Sensors
	GPIO_SetBits(GPIOB,GPIO_Pin_2);

	//DistanceSensor
	InputCaptureDistanceSensor();
	TIM4->CCR3=9;

	//speaker
	SpeakerInit();

	GPIO_SetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_4);
	GPIO_ResetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_5);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);


	while(1)
	{
		timer2cnt=TIM2->CNT;

		switch(ActiveMode)
		{
		case 0:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_4);
			GPIO_ResetBits(GPIOE,GPIO_Pin_3|GPIO_Pin_5);
			FollowLine();

			break;
		}
		case 1:
		{
			BluetoothControl();
			break;
		}
		default:
		{
			break;
		}
		}



	}


}

/*
 *
 *pe2-3	Engine1 control
 *pe4-5	Engine2 control
 *pe6	turn on/off engines
 *
 * pc10 UART/bluetooth
 * pc11 UART/bluetooth
 *
 * TIM4
 *100hz
 * pd12	 PWM engine1
 * pd13  PWM engine2
 *
 * pc7 - inputCapture -ultrawave sensor echo
 * pd14, TIM4->CCR3, ultrawave sensor trigger
 *
 * ADC pins
 * pb2   turn on IR sensors
 * pb0-1 main sensors middle1 & middle2
 * pa0-5 left & right sensors
 *
 *Bluetooth commands
 *	'x[number]' move forward/backward it uses -5 to 5 values
 *	'y[number]' move left/right it uses -5 to 5 values
 *	'o' on/off engines
 *	'L' linefollower mode
 *	'B' manual mode
 *	'D' distance sensor - not used
 *	'S' play 'Czesc tu slawomir'
 *	'T' play 'pyrpyrpyr'
 */



