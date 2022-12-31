/**
  ******************************************************************************
  * @file    ME586.c
  * @author  Zhou Zeng, Abhimanyu Gupta (2021)
  * @date    Fall, 2021
  * @brief   This file contains all the functions for use in ME 586 Lab
  ******************************************************************************
  */

#include "main.h"
#include "math.h"
#include "stm32f10x_conf.h"
#include "ME586.h"

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)

#define ADC1_DR_Address    ((uint32_t)0x4001244C)

ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint32_t ADC_ConvertedValueTab[6];  //ADC Data

int encoder1Pos, encoder2Pos;
int encoder1Count, encoder2Count;
uint32_t SysTick_CTRL;

void initcom(void) {
	//Create initialization structures
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;

	//turn on peripheral clocks
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );

	// Setup Tx pin.
	GPIO_StructInit (&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	//intitialize USART clock with default values
	USART_ClockStructInit( &USART_ClockInitStructure );
	USART_ClockInit( USART1, &USART_ClockInitStructure  );

	USART_StructInit( &USART_InitStructure );

	USART_InitStructure.USART_BaudRate = 115200;//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}


void initdac(void){
	DAC_InitTypeDef DAC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/*turn on DAC clock*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	connected to the DAC converter. In order to avoid parasitic consumption,
	the GPIO pin should be configured in analog */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5; //PA4=Ch 1, PA5=Ch 2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);


	/* Enable DAC Channels: Once the DAC channel1 is enabled, PA.04 is
	automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	DAC_SetChannel1Data(DAC_Align_12b_R, 2048);
	DAC_SetChannel2Data(DAC_Align_12b_R, 2048);
}

void initadc(void){
	ADC_InitTypeDef		ADC_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;

	//RCC configuration
	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	/* Enable peripheral clocks ------------------------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Enable ADC1,GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PC.00, PC.01, PC.02, PC.03, PC.04, and PC.05 (ADC Channel11, Channel12, Channel14, Channel14 and Channel15)
	as analog input ----------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);


	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 6;
	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	/* Test on DMA1 channel1 transfer complete flag */
	while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
	/* Clear DMA1 channel1 transfer complete flag */
	DMA_ClearFlag(DMA1_FLAG_TC1);
}

void initports(int direction){  //1 = input, 0 = output // PB5-PB12=bits 0-7 PC6-PC9=bits 8-11
	int directionB=0;
	int directionC=0;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	directionB=direction & 0xFF;  //8 LSBs
	directionC=direction & 0XF00; //4 MSBs

	directionB <<= 5; // Fall 2021 changed from 6 to 5
	directionC >>= 2;

	GPIO_InitStructure.GPIO_Pin = directionB;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = directionC;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	directionB=~directionB;
	directionC=~directionC;
	directionB=directionB & 0x1FE0;
	directionC=directionC & 0x3C0;

	GPIO_InitStructure.GPIO_Pin = directionB;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = directionC;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void inittime(float sample_time){  //sample_time < 700 ms
	/* Setup SysTick Timer for sample_time (in msec) interrupts  */
  if (SysTick_Config((int)((sample_time*(float)SystemCoreClock) / 1000.0)))
  {
    /* Capture error */
    while (1);
  }
}

void initint(void){
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA.01 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI1 Line to PA.01 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	/* Configure EXTI1 line */

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/* Config PC9 as PWM, PC8 as ground shield, PC7 as DIR*/
void initPWM(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	uint16_t PrescalerValue = 0;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* Config PC9 as PWM, PC8 as shield, PC7 as DIR*/
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//F21
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 4095; //about 0.11 ms
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* TIM4 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

/* 12-bit PWM channel, [-4095, 4095]*/
void outputPWM(int value){
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	unsigned int OutVal=0;

	if(value<0){
		OutVal=value*(-1);
		GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);//F21
	}
	else{
		OutVal=value;
		GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);//F21
	}

	TIM3->CCR4 = OutVal; //set duty cycle
}

/* External interrupts are left unaffected*/
void disable_timer_interrupt(void){
	SysTick_CTRL |= SysTick->CTRL;
	SysTick->CTRL  = 0;
	/*SysTick_CTRL_CLKSOURCE_Msk |
      SysTick_CTRL_TICKINT_Msk   |
      SysTick_CTRL_ENABLE_Msk;*/
}

void restore_timer_interrupt(void) {
	SysTick->CTRL = SysTick_CTRL;
}

void d_to_a(int channel, int value){
	switch(channel){
	case 0:
		DAC_SetChannel1Data(DAC_Align_12b_R, (short)value);
		break;
	case 1:
		DAC_SetChannel2Data(DAC_Align_12b_R, (short)value);
	  break;
	}
}

int a_to_d(int channel){
	return(ADC_ConvertedValueTab[channel]);
}

void digout(short data){  // PB5-PB12=bits 0-7 PC6-PC9=bits 8-11
	short dataB;
	short dataC;

	dataB=(short)data;
	dataC=(short)data;

	dataB=data & 0xFF;
	dataC=data & 0xF00;
	dataB=dataB<<5;
	dataC=dataC>>2;

	GPIO_Write(GPIOB,dataB);
	GPIO_Write(GPIOC,dataC);
}

short digin(void){
	short dataB;
	short dataC;

	dataB=GPIO_ReadInputData(GPIOB);
	dataC=GPIO_ReadInputData(GPIOC);

	dataB=dataB & 0x1FE0; //only PB5-12
	dataC=dataC & 0x3C0;  //only C6-9

	dataB=dataB>>5;
	dataC=dataC<<2;

	return((int)dataB + (int)dataC);
}

unsigned char checkcom(void){
	if((USART1->SR & USART_FLAG_RXNE)==0)
		return(0);
	else
		return(0xFF);
}

int getnum(void) {
	int res;
	scanf("%d", &res);
	return res;
}

void shownum(int value){
	printf("%d", value);
}

float getfloat(void) {
	float res;
	scanf("%f", &res);
	return res;
}

void WaitForKeypress(void){
	int status=0;

	do
	{
		status = checkcom();
	} while (status == 0);
	status = getchar();
}

/* Configure PB[8:5](DIO[3:0]) as input pins for two dual channel encoder signals*/
void initEncoder(void){
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	int i;

	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure PB[8:5] pin as input floating */
	for(i = GPIO_Pin_5; i != GPIO_Pin_9; i <<= 1) {
		GPIO_InitStructure.GPIO_Pin = i;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI1 Line to PA[8:5] pin */
	for(i = GPIO_PinSource5; i != GPIO_PinSource9; ++i) {
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, i);
	}

	/* Configure EXTI[5:9] line */
	for(i = EXTI_Line5; i != EXTI_Line9; i <<= 1) {
		EXTI_InitStructure.EXTI_Line = i;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	}

	/* Enable and set EXTI[8:5] Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

#ifdef USE_ABSOLUTE_ENCODER
void EXTI9_5_IRQHandler() {
	if (EXTI->PR & EXTI_Line5) {
		EXTI->PR = EXTI_Line5;
		Encoder1A();
		return;
	}
	else if (EXTI->PR & EXTI_Line6){
		EXTI->PR = EXTI_Line6;
		Encoder1B();
		return;
	}
	else if (EXTI->PR & EXTI_Line7) {
		EXTI->PR = EXTI_Line7;
		Encoder2A();
		return;
	}
	else if (EXTI->PR & EXTI_Line8) {
		EXTI->PR = EXTI_Line8;
		Encoder2B();
		return;
	}
}

void Encoder1A() {
	if (((GPIOB->IDR & GPIO_Pin_5) != 0) ^ ((GPIOB->IDR & GPIO_Pin_6) != 0))
		encoder1Pos++;
	else
		encoder1Pos--;
}

void Encoder1B() {
	if (((GPIOB->IDR & GPIO_Pin_5) != 0) ^ ((GPIOB->IDR & GPIO_Pin_6) != 0))
		encoder1Pos--;
	else
		encoder1Pos++;
}

void Encoder2A() {
	if (((GPIOB->IDR & GPIO_Pin_7) != 0) ^ ((GPIOB->IDR & GPIO_Pin_8) != 0))
		encoder2Pos++;
	else
		encoder2Pos--;
}

void Encoder2B() {
	if (((GPIOB->IDR & GPIO_Pin_7) != 0) ^ ((GPIOB->IDR & GPIO_Pin_8) != 0))
		encoder2Pos--;
	else
		encoder2Pos++;
}

void resetEncoder1Pos() {
	encoder1Pos = 0;
}

void resetEncoder2Pos() {
	encoder2Pos = 0;
}

int getEncoder1Pos() {
	return encoder1Pos;
}

int getEncoder2Pos() {
	return encoder2Pos;
}

void teachEncoder1Pos(int pos) {
	encoder1Pos = pos;
}

void teachEncoder2Pos(int pos) {
	encoder2Pos = pos;
}
#endif

#ifdef USE_INCREMENTAL_ENCODER
void EXTI9_5_IRQHandler() {
	if (EXTI->PR & EXTI_Line5) {
		EXTI->PR = EXTI_Line5;
		Encoder1();
		return;
	}
	else if (EXTI->PR & EXTI_Line6){
		EXTI->PR = EXTI_Line6;
		Encoder1();
		return;
	}
	else if (EXTI->PR & EXTI_Line7) {
		EXTI->PR = EXTI_Line7;
		Encoder2();
		return;
	}
	else if (EXTI->PR & EXTI_Line8) {
		EXTI->PR = EXTI_Line8;
		Encoder2();
		return;
	}
}

void Encoder1() {
	encoder1Count++;
}

void Encoder2() {
	encoder2Count++;
}

void resetEncoder1Count() {
	encoder1Count = 0;
}

void resetEncoder2Count() {
	encoder2Count = 0;
}

int getEncoder1Count() {
	return encoder1Count;
}

int getEncoder2Count() {
	return encoder2Count;
}
#endif

/*******************************************************************************
* Function Name  : PUTCHAR_PROTOTYPE
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
	/* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Fall 2021 this moved before senddata
//
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);
//
  return ch;
	}
//
GETCHAR_PROTOTYPE
{
	while (!(USART1->SR & USART_FLAG_RXNE));
	/* Echoing input character*/
	putchar(USART1->DR & 0xFF);
	return ((int)(USART1->DR & 0xFF));
}
