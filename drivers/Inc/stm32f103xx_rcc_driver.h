/*
 * stm32f103xx_rcc_driver.h
 *
 *  Created on: Jun 11, 2025
 *      Author: BIOS
 */

#ifndef INC_STM32F103XX_RCC_DRIVER_H_
#define INC_STM32F103XX_RCC_DRIVER_H_

#include<stm32f103xx.h>

uint32_t RCC_GetPLLOutputClock(void);

//this return APB1 clock value
uint32_t RCC_GetPCKL1Value(void);

//this return APB2 clock value
uint32_t RCC_GetPCKL2Value(void);


#endif /* INC_STM32F103XX_RCC_DRIVER_H_ */
