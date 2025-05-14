/*
 * stm32f1xx_afio_driver.h
 *
 *  Created on: May 14, 2025
 *      Author: hiephuu2001
 */

#ifndef INC_STM32F1XX_AFIO_DRIVER_H_
#define INC_STM32F1XX_AFIO_DRIVER_H_

#include<stm32f103xx.h>

/*
 * This Handle configure API for AFIO register
 */

/*
 * These API configure REMAP
 */
void SPI1_ControlRemap(uint8_t EnorDi);
void I2C1_ControlRemap(uint8_t EnorDi);
void UART1_ControlRemap(uint8_t EnorDi);
void UART2_ControlRemap(uint8_t EnorDi);
void UART3_ControlRemap(uint8_t UART3RMVal); 	/*Possible value from @UART3RMVal*/
void TIM1_ControlRemap(uint8_t TIM1RMVal);		/*Possible value from @TIM1RMVal*/
void TIM2_ControlRemap(uint8_t TIM2RMVal);		/*Possible value from @TIM2RMVal*/
void TIM3_ControlRemap(uint8_t TIM3RMVal);		/*Possible value from @TIM3RMVal*/
void TIM4_ControlRemap(uint8_t EnorDi);
void CAN_ControlRemap(uint8_t CANRMVal);		/*Possible value from @CANRMVal*/
void PD01_ControlRemap(uint8_t EnorDi);
void TIM5CH4_ControlRemap(uint8_t EnorDi);
void ADC1_ETRGINJ_ControlRemap(uint8_t EnorDi);
void ADC1_ETRGREG_ControlRemap(uint8_t EnorDi);
void ADC2_ETRGINJ_ControlRemap(uint8_t EnorDi);
void ADC2_ETRGREG_ControlRemap(uint8_t EnorDi);
void SWJ_CFG(uint8_t SWJCFGVal);				/*Possible value from @SWJCFGVal*/

/*
 * Macro for enable SPI1,I2C1, UART1, UART2,TIM4, PD1,TIM5CH4, ADC1, ADC2
 */
#define SPI1_REMAP_EN()					(AFIO->MAPR	|= (1 << 0))
#define I2C1_REMAP_EN()					(AFIO->MAPR	|= (1 << 1))
#define UART1_REMAP_EN()				(AFIO->MAPR	|= (1 << 2))
#define UART2_REMAP_EN()				(AFIO->MAPR	|= (1 << 3))
#define TIM4_REMAP_EN()					(AFIO->MAPR	|= (1 << 12))
#define PD01_REMAP_EN()					(AFIO->MAPR	|= (1 << 15))
#define TIM5CH4_REMAP_EN()				(AFIO->MAPR	|= (1 << 16))
#define ADC1_ETRGINJ_REMAP_EN()			(AFIO->MAPR	|= (1 << 17))
#define ADC1_ETRGREG_REMAP_EN()			(AFIO->MAPR	|= (1 << 18))
#define ADC2_ETRGINJ_REMAP_EN()			(AFIO->MAPR	|= (1 << 19))
#define ADC2_ETRGREG_REMAP_EN()			(AFIO->MAPR	|= (1 << 20))

/*
 * Macro for disable SPI1,I2C1, UART1, UART2,TIM4, PD1,TIM5CH4, ADC1, ADC2
 */
#define SPI1_REMAP_DI()					(AFIO->MAPR	&= ~(1 << 0))
#define I2C1_REMAP_DI()					(AFIO->MAPR	&= ~(1 << 1))
#define UART1_REMAP_DI()				(AFIO->MAPR	&= ~(1 << 2))
#define UART2_REMAP_DI()				(AFIO->MAPR	&= ~(1 << 3))
#define TIM4_REMAP_DI()					(AFIO->MAPR	&= ~(1 << 12))
#define PD01_REMAP_DI()					(AFIO->MAPR	&= ~(1 << 15))
#define TIM5CH4_REMAP_DI()				(AFIO->MAPR	&= ~(1 << 16))
#define ADC1_ETRGINJ_REMAP_DI()			(AFIO->MAPR	&= ~(1 << 17))
#define ADC1_ETRGREG_REMAP_DI()			(AFIO->MAPR	&= ~(1 << 18))
#define ADC2_ETRGINJ_REMAP_DI()			(AFIO->MAPR	&= ~(1 << 19))
#define ADC2_ETRGREG_REMAP_DI()			(AFIO->MAPR	&= ~(1 << 20))

/*
 * @UART3RMVal
 * UART3 Remap has possible values
 */
#define UART3_NO_REMAP					0
#define UART3_PARTIAL_REMAP				1
#define UART3_NOT_USE					2
#define UART3_FULL_REMAP				3

/*
 * @TIM1RMVal
 * TIM1 Remap has possible values
 */
#define TIM1_NO_REMAP					0
#define TIM1_PARTIAL_REMAP				1
#define TIM1_NOT_USE					2
#define TIM1_FULL_REMAP					3

/*
 * @TIM2RMVal
 * TIM2 Remap has possible values
 */
#define TIM2_NO_REMAP					0
#define TIM2_PARTIAL1_REMAP				1
#define TIM2_PARTIAL2_REMAP				2
#define TIM2_FULL_REMAP					3

/*
 * @TIM3RMVal
 * TIM3 Remap has possible values
 */
#define TIM3_NO_REMAP					0
#define TIM3_NOT_USE					1
#define TIM3_PARTIAL_REMAP				2
#define TIM3_FULL_REMAP					3

/*
 * @CANRMVal
 * CAN Remap has possible values
 */
#define CAN_RX_PA11						0  		//CAN_RX = PA11, CAN_TX = PA12
#define CAN_NOT_USE						1
#define CAN_RX_PB8						2		//CAN_RX = PB8, CAN_TX = PB9
#define CAN_RX_PD0						3		//CAN_RX = PD0, CAN_TX = PD1

/*
 * @SWJCFGVal
 * SWJ Configure has possible values
 */
#define SWJ_FULL						0
#define SWJ_WT_NJTRST					1
#define JTAG_DP_EN						2
#define JTAG_DP_DI						4

#endif /* INC_STM32F1XX_AFIO_DRIVER_H_ */
