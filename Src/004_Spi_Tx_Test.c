/*
 * 004_Spi_Tx_Test.c
 *
 *  Created on: May 15, 2025
 *      Author: hiephuu2001
 */

#include"stm32f103xx.h"
#include<string.h>

/*
 * SPI1_ReMap = 0
 * 	PA4 -> SPI1_NSS
 * 	PA5 -> SPI1_SCK
 * 	PA6 -> SPI1_MISO
 * 	PA7 -> SPI1_MOSI
 * SPI1_ReMap = 1
 * 	PA15 -> SPI1_NSS
 * 	PB3  -> SPI1_SCk
 * 	PB4	 -> SPI1_MISO
 * 	PB5	 -> SPI1_MOSI
 */

void SPI1_GPIOs(void){
	uint32_t Spi1rmval = Get_SPI1_RM();
	if( Spi1rmval == 0){
		GPIO_Handle_t SPIPins;

		//Config GPIOA
		SPIPins.pGPIOx = GPIOA;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_GP_OP_PP;

		//CLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 5;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 6;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 7;
		GPIO_Init(&SPIPins);

		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
		GPIO_Init(&SPIPins);
	} else
	{
		GPIO_Handle_t SPIPins;
		GPIO_Handle_t SPIPinsA;

		//Config GPIOA and GPIOB
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_GP_OP_PP;

		SPIPinsA.pGPIOx = GPIOA;
		SPIPinsA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz ;
		SPIPinsA.GPIO_PinConfig.GPIO_CNF		= GPIO_GP_OP_PP;

		//CLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 3;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPinsA.GPIO_PinConfig.GPIO_PinNumber = 5;
		GPIO_Init(&SPIPinsA);

		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
		GPIO_Init(&SPIPins);
	}
}


void SPI1_Init(void){
	SPI_Handle_t SPI1hanle;

	SPI1hanle.SPIConfig.SPI_BusConfig 	= SPI_BUS_CONFIG_FD;
	SPI1hanle.SPIConfig.SPI_DeviceMode 	= SPI_DEVICE_MODE_MASTER;
	SPI1hanle.SPIConfig.SPI_SclkSpeed	= SPI_FPCLK_DIV2;		//generate sclk 8mhz
	SPI1hanle.SPIConfig.SPI_DFF			= SPI_DFF_8BIT;
	SPI1hanle.SPIConfig.SPI_CPOL		= SPI_CPOL_LO;
	SPI1hanle.SPIConfig.SPI_CPHA		= SPI_CPHA_LO;
	SPI1hanle.SPIConfig.SPI_SSM			= SPI_SSM_EN;			//software slave management enable for NSS pin

	SPI_Init(&SPI1hanle);

}

int main(void){

	char user_data[] = "Hello World";
	//This function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOs();

	//This function is used to initialize the SPI1 peripheral parameter
	SPI1_Init();

	//Enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1,ENABLE);

	//to sent data
	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	while(1);

	return 0;
}

