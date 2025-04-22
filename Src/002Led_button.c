/*
 * 002Led_button.c
 *
 *  Created on: Apr 19, 2025
 *      Author: hiephuu2001
 */

#include "stm32f103xx.h"


#define HIGH 			ENABLE
#define LOW				DISABLE
#define BTN_PRESSED		LOW

void delay()
{
	for(uint32_t i = 0 ; i < 500000/2 ;i++);

}

int main()
{
	GPIO_Handle_t GPIOBtn, GPIOLed;


	//Configure GPIOLed PA8 for push-pull mode
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber 	= GPIO_PIN_NO_8;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode   	= GPIO_MODE_OUT_10MHz;
	GPIOLed.GPIO_PinConfig.GPIO_CNF			= GPIO_GP_OP_PP;

	// Enable GPIOA
	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Init GPIOLed
	GPIO_Init(&GPIOLed);

	//Configure GPIOBtn GPIO button PB12 for push-pull mode
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 	= GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF			= GPIO_IP_PUPD;

	//Enable GPIOB
	GPIO_PeriClockControl(GPIOB, ENABLE);

	//Init GPIOBtn
	GPIO_Init(&GPIOBtn);

	while(1)
	{
		if(	GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED ){
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}
	return 0;
}

