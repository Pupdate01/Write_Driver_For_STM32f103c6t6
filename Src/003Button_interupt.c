/*
 * 004gpio_freq.c
 *
 *  Created on: Apr 20, 2025
 *      Author: hiephuu2001
 */

#include<string.h>
#include<stm32f103xx_gpio_driver.h>

#define HIGH 		1
#define LOW 		0
#define BTN_PRESSED	LOW

void delay(void){
	// set delay 200ms
	for(uint32_t i = 0 ; i < 500000/2 ;i++);
}

int main(void)
{
	GPIO_Handle_t GPIOBtn,GPIOLed;
	memset(&GPIOBtn,0,sizeof(GPIOBtn)); //clear GPIOBtn
	memset(&GPIOLed,0,sizeof(GPIOLed));	//clear GPIOLed

	//Configure GPIO led  PA4 for output push-pull mode
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4 ;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode	  = GPIO_MODE_OUT_10MHz;
	GPIOLed.GPIO_PinConfig.GPIO_CNF		  = GPIO_GP_OP_PP;

	//Configure GPIO button PA6 for input pull up pull down mode
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6 ;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode	  = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF		  = GPIO_IP_PUPD;

	//Enable GPIOA
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//init GPIO led and button
	GPIO_Init(&GPIOBtn);
	GPIO_Init(&GPIOLed);

	//IRQ configure
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

 	while(1);
	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_4);
}
