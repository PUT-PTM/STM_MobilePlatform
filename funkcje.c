#include "funkcje.h"


void DMA_initP2M(void)
{
   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   	DMA_InitTypeDef DMAinitStruct;
   	// DMA Channel
   	DMAinitStruct.DMA_Channel = DMA_Channel_0;
   	// (memory2memory / peripheral2memory / memory2peripheral)
   	DMAinitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
   	// DMA MODE
   	DMAinitStruct.DMA_Mode = DMA_Mode_Circular;
   	// Priority
   	DMAinitStruct.DMA_Priority = DMA_Priority_High;
   	// Data to send
   	DMAinitStruct.DMA_BufferSize = ARRAYSIZE;
   	// Source address
   	DMAinitStruct.DMA_PeripheralBaseAddr = (uint32_t)(ADC_1_ADDRESS_BASE+ADC_DR_ADDRESS_OFFSET);
   	// Destination address
   	DMAinitStruct.DMA_Memory0BaseAddr = (uint32_t)&valueFromADC;

   	DMAinitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
   	DMAinitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   	// size of data
   	DMAinitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   	DMAinitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

   	DMAinitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   	DMAinitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;

   	DMAinitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
   	DMAinitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

   	DMA_Init(DMA2_Stream4, &DMAinitStruct);
   	DMA_Cmd(DMA2_Stream4, ENABLE);
}


void ADC_ScanMode_init(void)
{
   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

   	GPIO_InitTypeDef GPIO_InitStructure;
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
   			|GPIO_Pin_7|GPIO_Pin_5;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOA, &GPIO_InitStructure);

   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOB, &GPIO_InitStructure);

   	//Pin is used to enable IR sensors
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOB, &GPIO_InitStructure);



   	ADC_CommonInitTypeDef ADC_CommonInitStructure;
   	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
   	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   	ADC_CommonInit(&ADC_CommonInitStructure);

   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   	ADC_InitTypeDef ADC_InitStructure;
   	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
   	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
   	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   	ADC_InitStructure.ADC_NbrOfConversion = 8;
   	ADC_Init(ADC1, &ADC_InitStructure);


   	//Main IR Sensors
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_84Cycles);

   	//Additional IR Sensors
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 5, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 6, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 8, ADC_SampleTime_84Cycles);


   	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

   	ADC_DMACmd(ADC1, ENABLE);
   	ADC_Cmd(ADC1, ENABLE);
}

void PWM_Engine_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		GPIO_InitTypeDef port;

		port.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
		port.GPIO_Mode = GPIO_Mode_AF;
		port.GPIO_OType = GPIO_OType_PP;//PP raczej nie trzeba zmieniac
		port.GPIO_Speed = GPIO_Speed_100MHz;
		port.GPIO_PuPd = GPIO_PuPd_NOPULL;//zmienic np dla przyciskow
		GPIO_Init(GPIOD, &port);


		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIM_TimeBaseInitTypeDef timer;

		//10-1 ==> 0,01ms
		timer.TIM_Period = 10000-1;
		timer.TIM_Prescaler = 84-1;
		timer.TIM_ClockDivision = TIM_CKD_DIV1;
		timer.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &timer);

		TIM_Cmd(TIM4, ENABLE);

		TIM_OCInitTypeDef PWM;
		PWM.TIM_OCMode = TIM_OCMode_PWM1;
		PWM.TIM_OutputState = TIM_OutputState_Enable;
		PWM.TIM_Pulse = 0;
		PWM.TIM_OCPolarity = TIM_OCPolarity_High;

		TIM_OC1Init(TIM4, &PWM);
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_OC2Init(TIM4, &PWM);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_OC3Init(TIM4, &PWM);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_OC4Init(TIM4, &PWM);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

				TIM_Cmd(TIM4, ENABLE);

}

void Engine_Control_Pins_init(void)
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void UART_GPIOC_init(uint32_t baudRate)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//Rx & Tx
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//Tx Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);


	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	USART_Init(USART3, &USART_InitStructure);


	USART_Cmd(USART3, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	NVIC_EnableIRQ(USART3_IRQn);



}


void InputCaptureDistanceSensor()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		GPIO_InitTypeDef port;

		port.GPIO_Pin = GPIO_Pin_7;
		port.GPIO_Mode = GPIO_Mode_AF;
		port.GPIO_OType = GPIO_OType_PP;
		port.GPIO_Speed = GPIO_Speed_100MHz;
		port.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &port);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		TIM_TimeBaseInitTypeDef timer;

		timer.TIM_Period =10000-1;
		timer.TIM_Prescaler = 84-1;
		timer.TIM_ClockDivision = TIM_CKD_DIV1;
		timer.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &timer);


		TIM_ICInitTypeDef inputCapture;
		inputCapture.TIM_Channel=TIM_Channel_2;
		inputCapture.TIM_ICPolarity=TIM_ICPolarity_BothEdge;
		inputCapture.TIM_ICPrescaler=TIM_ICPSC_DIV1;
		inputCapture.TIM_ICSelection=TIM_ICSelection_DirectTI;
		inputCapture.TIM_ICFilter=0;

		TIM_ICInit(TIM3,&inputCapture);


		NVIC_InitTypeDef NVIC_InitStructure;

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

				NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);

				TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

				TIM_Cmd(TIM3, ENABLE);


}

void SpeakerInit(){
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef port;
	
	port.GPIO_Pin = GPIO_Pin_4;
	port.GPIO_Mode = GPIO_Mode_AN;
	port.GPIO_OType = GPIO_OType_PP;
	port.GPIO_Speed = GPIO_Speed_100MHz;
	port.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &port);
	
	
	DAC_InitTypeDef DAC_InitStructure;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_SetChannel1Data(DAC_Align_12b_R, 0x0000);
		
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 250-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 10-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
}


