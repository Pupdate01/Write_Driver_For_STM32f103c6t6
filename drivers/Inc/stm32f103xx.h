/*
 * stm32f103xx.h
 *
 *  Created on: Apr 12, 2025
 *      Author: hiephuu2001
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
#include<stdint.h>

#define __vo 							volatile
#define __weak 							__attribute__((weak))

/*
 * ARM Cortex M3 Processor NVIC ISERx register Address
 */
#define NVIC_ISER0						((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1						((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2						((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3						((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M3 Processor NVIC ICERx register Address
 */
#define NVIC_ICER0						((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1						((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2						((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3						((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M3 Processor Priority Register Address Calculation
 */
#define NVIC_PRIOR_BASE_ADDR 				((__vo uint32_t*)0xE000E400)

#define NO_PRIOR_BITS_IMPLEMENTED			4


/*	Base Address of Flash and SRAM memories	*/

#define FLASH_BASEADDR					0x08000000UL 	//Address of flash memory
#define SRAM							0x20000000UL	//Address of Sram
#define ROM								0x1FFFF000UL	//Address of Rom

/*	APBx and AHB2 Peripheral base address	*/

#define PERIPH_BASE						0x40000000UL	//Address of Peripheral bus
#define APB1PH_BASE						PERIPH_BASE		//Address of PHB1 peripheral bus
#define APB2PH_BASE						0x40010000UL	//Address of APB2 peripheral bus
#define AHBPH_BASE						0x40018000UL	//Address of AHB peripheral bus

/*Base addresses of peripherals which are hanging on AHB bus */
#define SDIO_BASEADDR					(AHBPH_BASE	+ 0x0000)		//Address of SDIO register
#define DMA1_BASEADDR					(AHBPH_BASE	+ 0x8000)		//Address of DMA1 register
#define DMA2_BASEADDR					(AHBPH_BASE	+ 0x8400)		//Address of DMA2 register
#define RCC_BASEADDR					(AHBPH_BASE	+ 0x9000)		//Address of RCC register
#define FLASHMEM_BASEADDR				(AHBPH_BASE	+ 0xA000)		//Address of Flash memory interface register
#define CRC_BASEADDR					(AHBPH_BASE	+ 0xB000)		//Address of CRC register
#define ETHERNET_BASEADDR				(AHBPH_BASE	+ 0x10000)		//Address of Ethernet register
#define USBOTG_BASEADDR					(AHBPH_BASE	+ 0xFFE8000)	//Address of USB OTG FS register
#define FSMC_BASEADDR					(AHBPH_BASE	+ 0x5FFE8000)	//Address of FSMC register

/*Base addresses of peripherals which are hanging on APB1 bus */
#define TIM2_BASEADDR					(APB1PH_BASE + 0x0000)	//Address of Timer 2 register
#define TIM3_BASEADDR					(APB1PH_BASE + 0x0400)	//Address of Timer 3 register
#define TIM4_BASEADDR					(APB1PH_BASE + 0x0800)	//Address of Timer 4 register
#define TIM5_BASEADDR					(APB1PH_BASE + 0x0C00)	//Address of Timer 5 register
#define TIM6_BASEADDR					(APB1PH_BASE + 0x1000)	//Address of Timer 6 register
#define TIM7_BASEADDR					(APB1PH_BASE + 0x1400)	//Address of Timer 7 register
#define TIM12_BASEADDR					(APB1PH_BASE + 0x1800)	//Address of Timer 12 register
#define TIM13_BASEADDR					(APB1PH_BASE + 0x1C00)	//Address of Timer 13 register
#define TIM14_BASEADDR					(APB1PH_BASE + 0x2000)	//Address of Timer 14 register
#define RTC_BASEADDR					(APB1PH_BASE + 0x2800)	//Address of RTC register
#define WWDG_BASEADDR					(APB1PH_BASE + 0x2C00)	//Address of Window watchdog register
#define IWDG_BASEADDR					(APB1PH_BASE + 0x3000)	//Address of Independent watchdog register
#define SPI2_I2S_BASEADDR				(APB1PH_BASE + 0x3800)	//Address of SPI2 register
#define SPI3_I2S_BASEADDR				(APB1PH_BASE + 0x3C00)	//Address of SPI3 register
#define USART2_BASEADDR					(APB1PH_BASE + 0x4400)	//Address of UART2 register
#define USART3_BASEADDR					(APB1PH_BASE + 0x4800)	//Address of UART3 register
#define UART4_BASEADDR					(APB1PH_BASE + 0x4C00)	//Address of UART4 register
#define UART5_BASEADDR					(APB1PH_BASE + 0x5000)	//Address of UART5 register
#define I2C1_BASEADDR					(APB1PH_BASE + 0x5400)	//Address of I2C1 register
#define I2C2_BASEADDR					(APB1PH_BASE + 0x5800)	//Address of I2C2 register
#define USB_BASEADDR					(APB1PH_BASE + 0x5C00)	//Address of USB device FS register
#define BXCAN1_BASEADDR					(APB1PH_BASE + 0x6400)	//Address of bxCAN1 register
#define BXCAN2_BASEADDR					(APB1PH_BASE + 0x6800)	//Address of bxCAN2 register
#define BKP_BASEADDR					(APB1PH_BASE + 0x6C00)	//Address of Backup register
#define PWR_BASEADDR					(APB1PH_BASE + 0x7000)	//Address of power control PWR register
#define DAC_BASEADDR					(APB1PH_BASE + 0x7400)	//Address of DAC register


/*Base addresses of peripherals which are hanging on APB2 bus */

#define AFIO_BASEADDR					(APB2PH_BASE + 0x0000)	//Address of AFIO register
#define EXTI_BASEADDR					(APB2PH_BASE + 0x0400)	//Address of EXTI register
#define GPIOA_BASEADDR					(APB2PH_BASE + 0x0800)	//Address of GPIO port A register
#define GPIOB_BASEADDR					(APB2PH_BASE + 0x0C00)	//Address of GPIO port B register
#define GPIOC_BASEADDR					(APB2PH_BASE + 0x1000)	//Address of GPIO port C register
#define GPIOD_BASEADDR					(APB2PH_BASE + 0x1400)	//Address of GPIO port D register
#define GPIOE_BASEADDR					(APB2PH_BASE + 0x1800)	//Address of GPIO port E register
#define GPIOF_BASEADDR					(APB2PH_BASE + 0x1C00)	//Address of GPIO port F register
#define GPIOG_BASEADDR					(APB2PH_BASE + 0x2000)	//Address of GPIO port G register
#define ADC1_BASEADDR					(APB2PH_BASE + 0x2400)	//Address of ADC 1 register
#define ADC2_BASEADDR					(APB2PH_BASE + 0x2800)	//Address of ADC 2 register
#define TIM1_BASEADDR					(APB2PH_BASE + 0x2C00)	//Address of Timer 1 register
#define SPI1_BASEADDR					(APB2PH_BASE + 0x3000)	//Address of SPI 1 register
#define TIM8_BASEADDR					(APB2PH_BASE + 0x3400)	//Address of Timer 8 register
#define UART1_BASEADDR					(APB2PH_BASE + 0x3800)	//Address of UART1 register
#define ADC3_BASEADDR					(APB2PH_BASE + 0x3C00)	//Address of ADC3 register
#define TIM9_BASEADDR					(APB2PH_BASE + 0x4C00)	//Address of Timer 9 register
#define TIM10_BASEADDR					(APB2PH_BASE + 0x5000)	//Address of Timer 10 register
#define TIM11_BASEADDR					(APB2PH_BASE + 0x5400)	//Address of Timer 11 register

/************************************Peripheral register definition structures ****************/
/*
 * Note: Register of a peripheral are specific to MCU
 * e.g: Number of Register of SPI of STM32F103xx family of MCUs may be different (more or less)
 */

typedef struct
{
	__vo uint32_t CR[2];	//Port configuration register											Address offset: 0x00 and 0x04
	__vo uint32_t IDR;		//Port input data register												Address offset: 0x08
	__vo uint32_t ODR;		//Port output data register												Address offset: 0x0C
	__vo uint32_t BSRR;		//Port bit set/reset register											Address offset: 0x10
	__vo uint32_t BRR;		//Port bit reset register												Address offset: 0x14
	__vo uint32_t LCKR;		//Port configuration lock register										Address offset: 0x18

}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct{
	__vo uint32_t CR;			/*Clock control register						Address offset: 0x00*/
	__vo uint32_t CFGR;			/*Clock configuration register					Address offset: 0x04*/
	__vo uint32_t CIR;			/*Clock interrupt register						Address offset: 0x08*/
	__vo uint32_t APB2RSTR;		/*APB2 peripheral reset register				Address offset: 0x0C*/
	__vo uint32_t APB1RSTR;		/*APB1 peripheral reset register				Address offset: 0x10*/
	__vo uint32_t AHBENR;		/*AHB peripheral clock enable register			Address offset: 0x14*/
	__vo uint32_t APB2ENR;		/*APB2 peripheral clock enable register			Address offset: 0x18*/
	__vo uint32_t APB1ENR;		/*APB1 peripheral clock enable register			Address offset: 0x1C*/
	__vo uint32_t BDCR;			/*Backup domain control register				Address offset: 0x20*/
	__vo uint32_t CSR;			/*Control/status register						Address offset: 0x24*/
}RCC_RegDef_t;

/*
 * Peripheral register definition structure for AFIO
 */

typedef struct {
	__vo uint32_t EVCR;			//Event control register
	__vo uint32_t MAPR;			//AF remap and debug I/O configuration register
	__vo uint32_t EXTICR[4];	//External interrupt configuration register
}AFIO_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;			/*Interrupt mask register						Address offset: 0x00*/
	__vo uint32_t EMR;			/*Event mask register							Address offset: 0x04*/
	__vo uint32_t RTSR;			/*Rising trigger selection register				Address offset: 0x08*/
	__vo uint32_t FTSR;			/*Falling trigger selection register			Address offset: 0x0C*/
	__vo uint32_t SWIER;		/*Software interrupt event register				Address offset: 0x10*/
	__vo uint32_t PR;			/*Pending register								Address offset: 0x14*/

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */

typedef struct {
	__vo uint32_t CR1;			//SPI control register 1
	__vo uint32_t CR2;			//SPI control register 2
	__vo uint32_t SR;			//SPI status register
	__vo uint32_t DR;			//SPI data register
	__vo uint32_t CRCPR;		//SPI CRC polynomial register
	__vo uint32_t RXCRCR;		//SPI RX CRC register
	__vo uint32_t TXCRCR;		//SPI TX CRC register
	__vo uint32_t I2FCFGR;		//SPI_I2S configuration register
	__vo uint32_t I2SPR;		//SPI_I2S prescaler register

}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */

typedef struct {
	__vo uint32_t CR1;			//I2C Control register 1
	__vo uint32_t CR2;			//I2C Control register 2
	__vo uint32_t OAR1;			//I2C Own address register 1
	__vo uint32_t OAR2;			//I2C Own address register 2
	__vo uint32_t DR;			//I2C Data register
	__vo uint32_t SR1;			//I2C Status register 1
	__vo uint32_t SR2;			//I2C Status register 2
	__vo uint32_t CCR;			//I2C Clock control register
	__vo uint32_t TRISE;		//I2C TRISE register

}I2C_RegDef_t;

/*********************************
 * Peripheral definitions
 * ***********************************************/

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define AFIO			((AFIO_RegDef_t*)AFIO_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_I2S_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_I2S_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 2)) 	//IO port A clock enable
#define GPIOB_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 3)) 	//IO port B clock enable
#define GPIOC_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 4)) 	//IO port C clock enable
#define GPIOD_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 5)) 	//IO port D clock enable
#define GPIOE_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 6)) 	//IO port E clock enable
#define GPIOF_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 7)) 	//IO port F clock enable
#define GPIOG_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 8)) 	//IO port G clock enable

/*
 * Clock Enable Macros for AFIO peripherals
 */
#define AFIO_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 0)) 	//ASFIO clock enable

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 21)) 	// I2C1 clock enable
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 22)) 	// I2C2 clock enable

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 12)) 	// SPI1 clock enable
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 14)) 	// SPI2 clock enable
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 15)) 	// SPI3 clock enable

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART1_PCLK_EN()			(RCC->APB2ENR |= ( 1 << 14)) 	// UART1 clock enable
#define UART2_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 17)) 	// UART2 clock enable
#define UART3_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 18)) 	// UART3 clock enable
#define UART4_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 19)) 	// UART4 clock enable
#define UART5_PCLK_EN()			(RCC->APB1ENR |= ( 1 << 20)) 	// UART5 clock enable

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 2)) 	//IO port A clock disable
#define GPIOB_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 3)) 	//IO port B clock enable
#define GPIOC_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 4)) 	//IO port C clock enable
#define GPIOD_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 5)) 	//IO port D clock enable
#define GPIOE_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 6)) 	//IO port E clock enable
#define GPIOF_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 7)) 	//IO port F clock enable
#define GPIOG_PCLK_DI()			(RCC->APB2ENR &= ~( 1 << 8)) 	//IO port G clock enable

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 21)) 	// I2C1 clock disable
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~( 1 << 22)) 	// I2C2 clock disable

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ( 1 << 12)) 	// SPI1 clock disable
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 14)) 	// SPI2 clock disable
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 15)) 	// SPI3 clock disable

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART1_PCLK_DI()			(RCC->APB2ENR &= ( 1 << 14)) 	// UART1 clock disable
#define UART2_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 17)) 	// UART2 clock disable
#define UART3_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 18)) 	// UART3 clock disable
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 19)) 	// UART4 clock disable
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ( 1 << 20)) 	// UART5 clock disable

/*
 * Macro to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 2)); (RCC->APB2ENR &= ~( 1 << 2)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 3)); (RCC->APB2ENR &= ~( 1 << 3)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 4)); (RCC->APB2ENR &= ~( 1 << 4)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 5)); (RCC->APB2ENR &= ~( 1 << 5)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 6)); (RCC->APB2ENR &= ~( 1 << 6)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 7)); (RCC->APB2ENR &= ~( 1 << 7)); } while(0)
#define GPIOG_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 8)); (RCC->APB2ENR &= ~( 1 << 8)); } while(0)

/*
 * Macro to reset SPIx peripherals
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2ENR |= ( 1 << 12 )); (RCC->APB2ENR &= ~(1 << 12 ));}while(0)
#define SPI2_REG_RESET()		do{	(RCC->APB1ENR |= ( 1 << 14 )); (RCC->APB1ENR &= ~(1 << 14 ));}while(0)
#define SPI3_REG_RESET()		do{	(RCC->APB1ENR |= ( 1 << 15 )); (RCC->APB1ENR &= ~(1 << 15 ));}while(0)

/*
 * return port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA ) ? 0 :\
									(x == GPIOB ) ? 1 :\
									(x == GPIOC ) ? 2 :\
									(x == GPIOD ) ? 3 :\
									(x == GPIOE ) ? 4 :\
									(x == GPIOF ) ? 5 :\
									(x == GPIOG ) ? 6 :0)

/*
 * IRQ number of STM32f103 MCU
 * Note: update these macro with valid values according to the MCU
 * Todo:
 */
#define IRQ_NO_EXTI0 					6
#define IRQ_NO_EXTI1 					7
#define IRQ_NO_EXTI2 					8
#define IRQ_NO_EXTI3 					9
#define IRQ_NO_EXTI4 					10
#define IRQ_NO_EXTI9_5 					23
#define IRQ_NO_EXTI15_10 				40
#define IQR_NO_SPI1						35
#define IQR_NO_SPI2						36

/*
 * Macros for all IRQ Priority levels
 */

#define NVIC_IRQ_PRIO0 					0
#define NVIC_IRQ_PRIO1 					1
#define NVIC_IRQ_PRIO2 					2
#define NVIC_IRQ_PRIO3 					3
#define NVIC_IRQ_PRIO4 					4
#define NVIC_IRQ_PRIO5 					5
#define NVIC_IRQ_PRIO6 					6
#define NVIC_IRQ_PRIO7 					7
#define NVIC_IRQ_PRIO8 					8
#define NVIC_IRQ_PRIO9 					9
#define NVIC_IRQ_PRIO10					10
#define NVIC_IRQ_PRIO11					11
#define NVIC_IRQ_PRIO12					12
#define NVIC_IRQ_PRIO13					13
#define NVIC_IRQ_PRIO14					14
#define NVIC_IRQ_PRIO15					15


//Generic macros
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET


/*
 * Bit position definitions of SPI peripheral
 **********************************************************
 */

/*
 * Bit position definition SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSPFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definition SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definition SPI_SR
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7


/*
 * Bit position definitions of I2C peripheral
 **********************************************************
 */

/*
 * Bit position definition I2C_CR1
 */
#define	I2C_CR1_PE				0
#define	I2C_CR1_SMBUS			1
#define	I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/*
 * Bit position definition I2C_CR2
 */
#define	I2C_CR2_FREQ			0
#define	I2C_CR2_ITERREN			8
#define	I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * Bit position definition I2C_OAR1
 */
#define	I2C_OAR1_ADD0			0
#define	I2C_OAR1_ADD1			1
#define	I2C_OAR1_ADD2			2
#define	I2C_OAR1_ADDMODE		15

/*
 * Bit position definition I2C_OAR2
 */
#define	I2C_OAR2_ENDUAL			0
#define	I2C_OAR2_ADD2			1

/*
 * Bit position definition I2C_DR
 */
#define	I2C_DR_DR				0

/*
 * Bit position definition I2C_SR1
 */
#define	I2C_SR1_SB				0
#define	I2C_SR1_ADDR			1
#define	I2C_SR1_BTF				2
#define	I2C_SR1_ADD10			3
#define	I2C_SR1_STOPF			4
#define	I2C_SR1_RXNE			6
#define	I2C_SR1_TXE				7
#define	I2C_SR1_BERR			8
#define	I2C_SR1_ARLO			9
#define	I2C_SR1_AF				10
#define	I2C_SR1_OVR				11
#define	I2C_SR1_PECERR			12
#define	I2C_SR1_TIMEOUT			14
#define	I2C_SR1_SMBALERT		15

/*
 * Bit position definition I2C_SR1
 */
#define	I2C_SR2_MSL				0
#define	I2C_SR2_BUSY			1
#define	I2C_SR2_TRA				2
#define	I2C_SR2_GENCALL			4
#define	I2C_SR2_SMBDEFAULT		5
#define	I2C_SR2_SMBHOST			6
#define	I2C_SR2_DUALF			7
#define	I2C_SR2_PEC				8

/*
 * Bit position definition I2C_CCR
 */
#define	I2C_CCR_CCR				0
#define	I2C_CCR_DUTY			14
#define	I2C_CCR_FS				15

/*
 * Bit position definition I2C_TRISE
 */
#define	I2C_CCR_TRISE				0

#include"stm32f103xx_gpio_driver.h"
#include"stm32f103xx_spi_driver.h"
#include"stm32f1xx_afio_driver.h"
#include"stm32f103xx_i2c_driver.h"
#include"stm32f103xx_rcc_driver.h"

#endif /* INC_STM32F103XX_H_ */
