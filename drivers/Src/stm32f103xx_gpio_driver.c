/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Apr 14, 2025
 *      Author: hiephuu2001
 */


#include<stm32f103xx_gpio_driver.h>


/*
 * Peripheral Clock setup
 */

/***********************************************************************************
 * @fn							- GPIO_PeriClockControl
 *
 * @brief						- This function enable or disable peripheral clock for given GPIO port
 *
 * param[in]					- Base address of the gpio peripheral
 * param[in]					- ENABLE or DISABLE macros
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi )
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}

	} else {
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}

	}
}

/*
 * Init
 */

/***********************************************************************************
 * @fn							-GPIO_Init
 *
 * @brief						- This function init GPIO
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
	uint8_t index1,index2;
	index1 	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
	index2 	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	//1. Configure the mode of the gpio pin
	if(	pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=  GPIO_MODE_OUT_50MHz){
		//The non-interrupt mode
		temp   	= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * index2);
		pGPIOHandle->pGPIOx->CR[index1] &= ~(0x3<<(4 * index2));					//clear bit
		pGPIOHandle->pGPIOx->CR[index1] |= temp;

	} else {
		//The interrupt mode
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT ){
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT ){
			//1. configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. configure the FTRS and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. Configure the GPIO port selection in AFIO_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		AFIO_PCLK_EN();
		AFIO->EXTICR[temp1] = portcode << (temp2*4);

		//3. Enable the EXTI interrupt delivery IMR

		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;
	//2. configure CNF
	temp   	= pGPIOHandle->GPIO_PinConfig.GPIO_CNF << (4 * index2 + 2);
	pGPIOHandle->pGPIOx->CR[index1] &= ~(0x3<<(4 * index2+2)); 						//clear bit
	pGPIOHandle->pGPIOx->CR[index1] |= temp;

}


/*
 * De-init
 */

/***********************************************************************************
 * @fn							-GPIO_Deinit
 *
 * @brief						- This function delete init GPIO
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOC){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOD){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOE){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOF){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOG){
		GPIOA_REG_RESET();
	}
}

/***********************************************************************************
 * @fn							-GPIO_ReadFromInputPin
 *
 * @brief						- This function read from input pin
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					- Pin number
 * param[in]					-
 *
 * @return						- 0 or 1
 *
 * @Note						- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value ;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}


/***********************************************************************************
 * @fn							- GPIO_ReadFromInputPin
 *
 * @brief						- This function read from input pin
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					- Pin number
 * param[in]					-
 *
 * @return						- 0 or 1
 *
 * @Note						- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/***********************************************************************************
 * @fn							- GPIO_WriteToOutputPin
 *
 * @brief						- This function write a value a output pin
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					- Pin number
 * param[in]					- Value
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_RESET){
		//write 0 to the output data register at the bit field corresponding to the pin
		pGPIOx -> ODR |= ( 1 << PinNumber );
	} else {
		//write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx -> ODR &= ~( 1 << PinNumber );
	}
}

/***********************************************************************************
 * @fn							- GPIO_WriteToOutputPort
 *
 * @brief						- This function write a output port
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					- Value
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***********************************************************************************
 * @fn							- GPIO_ToggleOutputPin
 *
 * @brief						-
 *
 * param[in]					- Base Address of GPIO Pin peripheral
 * param[in]					- Pin number
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,  uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IQR Configuration
 */

/***********************************************************************************
 * @fn							- GPIO_IRQITConfig
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE){
		if(IRQNumber <= 31) //0 to 31
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		} else if (IRQNumber >= 64 && IRQNumber < 68)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1<< (IRQNumber%64));
		}
	} else
	{
		if(IRQNumber <= 31) //0 to 31
		{
			// program ISER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		} else if (IRQNumber >= 64 && IRQNumber < 68)
		{
			//program ISER2 register
			*NVIC_ICER2 |= (1<< (IRQNumber%64));
		}
	}
}

/***********************************************************************************
 * @fn							- GPIO_IRQPriorityConfig
 *
 * @brief						-
 *
 * param[in]					-
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1.first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;


	uint8_t shift_amount = ( 8 * iprx_section)+ ( 8 - NO_PRIOR_BITS_IMPLEMENTED );
	*(NVIC_PRIOR_BASE_ADDR + iprx) |= (IRQPriority << (8 * shift_amount));


}


/***********************************************************************************
 * @fn							- GPIO_IRQHandling
 *
 * @brief						-
 *
 * param[in]					- Number Pin
 * param[in]					-
 * param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber ))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber );
	}
}


