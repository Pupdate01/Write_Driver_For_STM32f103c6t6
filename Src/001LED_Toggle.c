/*
 * 001LED_Toggle.c
 *
 *  Created on: Apr 18, 2025
 *      Author: hiephuu2001
 */

#include "stm32f103xx.h"

void delay()
{
	for(uint32_t i = 0 ; i < 500000 ;i++);

}

int main()
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx 							= GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 	= GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode   	= GPIO_MODE_OUT_10MHz;
	GpioLed.GPIO_PinConfig.GPIO_CNF		  	= GPIO_GP_OP_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}
	return 0;
}
