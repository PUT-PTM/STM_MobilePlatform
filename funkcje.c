#include "funkcje.h"

void ButtonInterrupt()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	// numer przerwania
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	// priorytet g³ówny
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	// subpriorytet
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	// uruchom dany kana³
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// zapisz wype³nion¹ strukturê do rejestrów
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef EXTI_InitStructure;
	// wybór numeru aktualnie konfigurowanej linii przerwañ
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	// wybór trybu - przerwanie b¹dŸ zdarzenie
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// wybór zbocza, na które zareaguje przerwanie
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	// uruchom dan¹ liniê przerwañ
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// zapisz strukturê konfiguracyjn¹ przerwañ zewnêtrznych do rejestrów
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
}


void DMA_initP2M(void)
{
   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   	DMA_InitTypeDef strukturaDoInicjalizacjiDMA;
   	// wybór kana³u DMA
   	strukturaDoInicjalizacjiDMA.DMA_Channel = DMA_Channel_0;
   	// ustalenie rodzaju transferu (memory2memory / peripheral2memory / memory2peripheral)
   	strukturaDoInicjalizacjiDMA.DMA_DIR = DMA_DIR_PeripheralToMemory;
   	// tryb pracy - pojedynczy transfer b¹dŸ powtarzany
   	strukturaDoInicjalizacjiDMA.DMA_Mode = DMA_Mode_Circular;
   	// ustalenie priorytetu danego kana³u DMA
   	strukturaDoInicjalizacjiDMA.DMA_Priority = DMA_Priority_High;
   	// liczba danych do przes³ania
   	strukturaDoInicjalizacjiDMA.DMA_BufferSize = ARRAYSIZE;
   	// adres Ÿród³owy
   	strukturaDoInicjalizacjiDMA.DMA_PeripheralBaseAddr = (uint32_t)(ADC_1_ADDRESS_BASE+ADC_DR_ADDRESS_OFFSET);
   	// adres docelowy
   	strukturaDoInicjalizacjiDMA.DMA_Memory0BaseAddr = (uint32_t)&valueFromADC;
   	// okreslenie, czy adresy maj¹ byæ inkrementowane po ka¿dej przes³anej paczce danych
   	strukturaDoInicjalizacjiDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
   	strukturaDoInicjalizacjiDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   	// ustalenie rozmiaru przesy³anych danych
   	strukturaDoInicjalizacjiDMA.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   	strukturaDoInicjalizacjiDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   	// ustalenie trybu pracy - jednorazwe przes³anie danych
   	strukturaDoInicjalizacjiDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   	strukturaDoInicjalizacjiDMA.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   	// wy³¹czenie kolejki FIFO (nie u¿ywana w tym przykadzie)
   	strukturaDoInicjalizacjiDMA.DMA_FIFOMode = DMA_FIFOMode_Disable;
   	// wype³nianie wszystkich pól struktury jest niezbêdne w celu poprawnego dzia³ania, wpisanie jednej z dozwolonych wartosci
   	strukturaDoInicjalizacjiDMA.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
   	// zapisanie wype³nionej struktury do rejestrów wybranego po³¹czenia DMA
   	DMA_Init(DMA2_Stream4, &strukturaDoInicjalizacjiDMA);
   	// uruchomienie odpowiedniego po³¹czenia DMA
   	DMA_Cmd(DMA2_Stream4, ENABLE);
}


void ADC_ScanMode_init(void)
{
   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE); // wejscie ADC
   	//inicjalizacja wejœcia ADC
   	GPIO_InitTypeDef GPIO_InitStructure;
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
   			|GPIO_Pin_4|GPIO_Pin_5;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOA, &GPIO_InitStructure);

   	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE); // wejscie ADC
   	//inicjalizacja wejœcia ADC
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOB, &GPIO_InitStructure);

   	//pin do wlaczenia czujnikow stanem wysokim
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOB, &GPIO_InitStructure);



   	ADC_CommonInitTypeDef ADC_CommonInitStructure;
   	// niezale¿ny tryb pracy przetworników
   	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   	// zegar g³ówny podzielony przez 2
   	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   	// opcja istotna tylko dla tryby multi ADC
   	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
   	// czas przerwy pomiêdzy kolejnymi konwersjami
   	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
   	ADC_CommonInit(&ADC_CommonInitStructure);

   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ADC
   	ADC_InitTypeDef ADC_InitStructure;
   	//ustawienie rozdzielczoœci przetwornika na maksymaln¹ (12 bitów)
   	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu //wejœciach/kana³ach)
   	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
   	//w³¹czenie ci¹g³ego trybu pracy
   	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   	//wy³¹czenie zewnêtrznego wyzwalania
   	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w //dokumentacji)
   	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
   	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   	//wartoœæ binarna wyniku bêdzie podawana z wyrównaniem do prawej
   	//funkcja do odczytu stanu przetwornika ADC zwraca wartoœæ 16-bitow¹
   	//dla przyk³adu, wartoœæ 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
   	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   	//liczba konwersji równa 8, bo 8 kana³ow
   	ADC_InitStructure.ADC_NbrOfConversion = 8;
   	// zapisz wype³nion¹ strukturê do rejestrów przetwornika numer 1
   	ADC_Init(ADC1, &ADC_InitStructure);
   	// konfiguracja czasu próbkowania sygna³u


   	//podzial na porty A oraz B, poniewaz pin PA6 podawal zle wartosci

   	//Glowne czujniki linii
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_84Cycles);

   	//pomocnicze czujniki
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 3, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 4, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 5, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 6, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 7, ADC_SampleTime_84Cycles);
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 8, ADC_SampleTime_84Cycles);


   	// w³¹czenie wyzwalania DMA po ka¿dym zakoñczeniu konwersji
   	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

   	// w³¹czenie DMA dla ADC
   	ADC_DMACmd(ADC1, ENABLE);
   	// uruchomienie modu³y ADC
   	ADC_Cmd(ADC1, ENABLE);
}

//todo zmienic piny
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

		timer.TIM_Period = 10000-1;
		timer.TIM_Prescaler = 84-1;
		timer.TIM_ClockDivision = TIM_CKD_DIV1;
		timer.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &timer);

		TIM_Cmd(TIM4, ENABLE);

		//pamietac o wlaczeniu GPIOD
		TIM_OCInitTypeDef PWM;
			/* PWM1 Mode configuration: */
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
}

void Engine_Controll_Pins_init(void)
{
	//sterowanie kierunkiem jazdy, pin4- wlaczenie/wylaczenie silnikow

	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void UART_GPIOC_init(uint32_t baudRate)
{
	// wlaczenie taktowania wybranego portu
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// wlaczenie taktowania wybranego uk³adu USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// konfiguracja linii Rx i Tx
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//Tx Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// ustawienie funkcji alternatywnej dla pinów (USART)
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);


	USART_InitTypeDef USART_InitStructure;
	// predkosc transmisji (mozliwe standardowe opcje: 9600, 19200, 38400, 57600, 115200, ...)
	USART_InitStructure.USART_BaudRate = baudRate;

	// d³ugoœæ s³owa (USART_WordLength_8b lub USART_WordLength_9b)
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// liczba bitów stopu (USART_StopBits_1, USART_StopBits_0_5, USART_StopBits_2, USART_StopBits_1_5)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// sprawdzanie parzystoœci (USART_Parity_No, USART_Parity_Even, USART_Parity_Odd)
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// sprzêtowa kontrola przep³ywu (USART_HardwareFlowControl_None, USART_HardwareFlowControl_RTS, USART_HardwareFlowControl_CTS, USART_HardwareFlowControl_RTS_CTS)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// tryb nadawania/odbierania (USART_Mode_Rx, USART_Mode_Rx )
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// konfiguracja
	USART_Init(USART3, &USART_InitStructure);

	// wlaczenie ukladu USART
	USART_Cmd(USART3, ENABLE);

	//struktura do konfiguracji kontrolera NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	// wlaczenie przerwania zwi¹zanego z odebraniem danych (pozostale zrodla przerwan zdefiniowane sa w pliku stm32f4xx_usart.h)
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// konfiguracja kontrolera przerwan
	NVIC_Init(&NVIC_InitStructure);
	// wlaczenie przerwan od ukladu USART
	NVIC_EnableIRQ(USART3_IRQn);



}

void Engine_Off_Timer_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInitTypeDef timer;

	timer.TIM_Period = 10000-1;
	timer.TIM_Prescaler = 8400-1;
	timer.TIM_ClockDivision = TIM_CKD_DIV1;
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timer);
}


void Engine_Controller_Bluetooth(char direction,char X, char Y)
{
	switch(direction)
	{
		case 0:
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_3);
			TIM4->CCR1=X*(2000-1);
			TIM4->CCR2=X*(2000-1);

		}break;
		case 1:
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_3);
			GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_2);
			TIM4->CCR1=X*(2000-1);
			TIM4->CCR2=X*(2000-1);

		}break;
		case 2:
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2);
			GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_3);
			TIM4->CCR1=Y*(2000-1);
			TIM4->CCR2=Y*(2000-1);

		}break;
		case 3:
		{
			GPIO_SetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_3);
			GPIO_ResetBits(GPIOC,GPIO_Pin_1|GPIO_Pin_2);
			TIM4->CCR1=Y*(2000-1);
			TIM4->CCR2=Y*(2000-1);
		}break;
		default:
		{

		}break;
		}


}


