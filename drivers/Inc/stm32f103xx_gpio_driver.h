/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Apr 14, 2025
 *      Author: hiephuu2001
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include<stm32f103xx.h>

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;				/* possible value from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;				/* possible value from @GPIO_PIN_MODE*/
	uint8_t GPIO_CNF;					/* possible value from @GPIO_CONFIGURE_BIT*/
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					/*This holds the base Address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*This holds GPIO pin configuration settings*/
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 * GPIO pin possible modes
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODE
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN	 		0
#define GPIO_MODE_OUT_10MHz		1
#define GPIO_MODE_OUT_2MHz		2
#define GPIO_MODE_OUT_50MHz		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_CONFIGURE_BIT
 * GPIO configure Input or Output mode
 */

#define GPIO_GP_OP_PP			0	/*Output:General purpose output push-pull*/
#define GPIO_GP_OP_OD			1	/*Output:General purpose output Open-drain*/
#define GPIO_ALTFN_OP_PP		2	/*Output:Alternate function output Push-pull*/
#define GPIO_ALTFN_OP_OD		3	/*Output:Alternate function output Open-drain*/
#define GPIO_IP_ANALOG			0	/*Input: Analog mode*/
#define GPIO_IP_FLOAT			1	/*Input: Floating input*/
#define GPIO_IP_PUPD			2	/*Input with pull-up / pull-down*/


/*********************************************************************************************************
 *  					API supported by this driver
 * 			For more information about the APIs check the function definitions
 *********************************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi );

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,  uint8_t PinNumber);

/*
 * IQR Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
