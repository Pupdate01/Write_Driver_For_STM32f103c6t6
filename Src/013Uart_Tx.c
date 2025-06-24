/*
 * 013Uart_Tx.c
 *
 *  Created on: Jun 24, 2025
 *      Author: hiephuu2001
 */

#include<string.h>
#include<stdio.h>
#include<stm32f103xx.h>

USART_Handle_t USART1Handle;

// A9 -> Tx
// A10 -> Rx

char msg[1024] = "USRT Testing....\n\r";

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++);
}

void UART1_Init(void)
{
	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&USART1Handle);
}

void UART1_GPIOInit(void){
	uint32_t USART1rmval = Get_UART1_RM();
	GPIO_Handle_t USART1Pins;

	if(USART1rmval == 0){
		USART1Pins.pGPIOx = GPIOA;
		USART1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_2MHz;
		USART1Pins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_PP;

		//USART1 Tx
		USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		GPIO_Init(&USART1Pins);

		//USART1 Rx
		USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
		GPIO_Init(&USART1Pins);
	} else {
		USART1Pins.pGPIOx = GPIOB;
		USART1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz;
		USART1Pins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_PP;

		//USART1 Tx
		USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&USART1Pins);

		//USART1 Rx
		USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&USART1Pins);
	}
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF = GPIO_IP_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void){

	//Config GPIO button
	GPIO_ButtonInit();

	UART1_GPIOInit();

	UART1_Init();

	USART_PeripheralControl(USART1, ENABLE);
	while(1)
	{
		//wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		USART_SendData(&USART1Handle,(uint8_t*) msg, strlen(msg));
	}
	return 0;
}
