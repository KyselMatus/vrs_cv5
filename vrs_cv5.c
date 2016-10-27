/*
 * vrs_cv5.c
 *
 *  Created on: 17. 10. 2016
 *      Author: Admin
 */
#include <stddef.h>
#include "stm32l1xx.h"
#include "vrs_cv5.h"

uint32_t adc_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	/* Configure ADCx Channel 2 as analog input */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable the HSI oscillator */
	RCC_HSICmd(ENABLE);

	/* Check that HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Enable ADC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);
	/* ADC1 configuration */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADCx regular channel8 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_16Cycles);
	/* Enable the ADC */
	ADC_Cmd(ADC1, ENABLE);
	/* Wait until the ADC1 is ready */
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
	{
	}
	/* Start ADC Software Conversion */
	ADC_SoftwareStartConv(ADC1);

	AD_value = ADC_GetConversionValue(ADC1);
	return AD_value;
}

void led_init(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitTypeDef gpioInitStruc;

	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType = GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(GPIOA, &gpioInitStruc);
}

void nvic_init(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn; //zoznam prerušení nájdete v súbore stm32l1xx.h
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC | ADC_IT_OVR, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void usart_init(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// inicializacia RX, TX
	GPIO_InitTypeDef RXInitStr;
	RXInitStr.GPIO_Pin = GPIO_Pin_3;
	RXInitStr.GPIO_Mode = GPIO_Mode_AF;
	RXInitStr.GPIO_OType = GPIO_OType_PP;
	RXInitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;
	RXInitStr.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&RXInitStr);

	GPIO_InitTypeDef TXInitStr;
	TXInitStr.GPIO_Pin = GPIO_Pin_2;
	TXInitStr.GPIO_Mode = GPIO_Mode_AF;
	TXInitStr.GPIO_OType = GPIO_OType_PP;
	TXInitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;
	TXInitStr.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&TXInitStr);

	// inicializacia USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

int which_button(uint32_t AD_value)
{
	int button_state;
	if(AD_value>=3940 && AD_value<=3955 )		// ziadne tlacidlo nie je stlacene
		button_state= 0;
	else if (AD_value>=2000 && AD_value<=2020)	// stlacene tlacidlo c.1
		button_state= 1;
	else if (AD_value>=2880 && AD_value<=2930)	// stlacene tlacidlo c.2
		button_state= 2;
	else if (AD_value>=3440 && AD_value<=3465)  // stlacene tlacidlo c.3
		button_state= 3;
	else if (AD_value>=3640 && AD_value<=3670)	// stlacene tlacidlo c.4
		button_state= 4;
	else button_state = -1;

	return button_state;
}

void work(int buttonState)
{
	while (1){
		// blikanie LED v roznych frekvenciach
		if (buttonState == 0)
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
		else if (buttonState == 1){
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			delay(100);
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			delay(100);
		}
		else if (buttonState == 2){
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			delay(500);
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			delay(500);
		}
		else if (buttonState == 3){
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			delay(1000);
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			delay(1000);
		}
		else if (buttonState == 4)
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		else{
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			delay(100);
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			delay(1000);
		}
	}
}

void SendUSART2(char *s){
  while(*s){
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, *s++);
  }
}


