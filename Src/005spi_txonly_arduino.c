/*
 * 005spi_txonly_arduino.c
 *
 *  Created on: May 25, 2025
 *      Author: hiephuu2001
 */


/*
 * 004_Spi_Tx_Test.c
 *
 *  Created on: May 15, 2025
 *      Author: hiephuu2001
 */

#include"stm32f103xx.h"
#include<string.h>

#define HIGH		ENABLE
#define BTN_PRESSED HIGH

void delay(){
	for(uint32_t i =0 ; i < 500000/2; i++);
}

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
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_ALTFN_OP_PP;

		//CLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 5;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 7;
		GPIO_Init(&SPIPins);

		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_IP_PUPD;
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 6;
		GPIO_Init(&SPIPins);

	} else
	{
		GPIO_Handle_t SPIPins;
		GPIO_Handle_t SPIPinsA;

		//Config GPIOA and GPIOB
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_ALTFN_OP_PP;

		SPIPinsA.pGPIOx = GPIOA;
		SPIPinsA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz ;
		SPIPinsA.GPIO_PinConfig.GPIO_CNF		= GPIO_ALTFN_OP_PP;

		//CLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 3;
		GPIO_Init(&SPIPins);

		//MISO
//		SPIPins.GPIO_PinConfig.GPIO_PinNumber = 4;
//		GPIO_Init(&SPIPins);

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

	SPI1hanle.pSPIx 					= SPI1;
	SPI1hanle.SPIConfig.SPI_BusConfig 	= SPI_BUS_CONFIG_FD;
	SPI1hanle.SPIConfig.SPI_DeviceMode 	= SPI_DEVICE_MODE_MASTER;
	SPI1hanle.SPIConfig.SPI_SclkSpeed	= SPI_FPCLK_DIV8;		//generate sclk 2mhz
	SPI1hanle.SPIConfig.SPI_DFF			= SPI_DFF_8BIT;
	SPI1hanle.SPIConfig.SPI_CPOL		= SPI_CPOL_LO;
	SPI1hanle.SPIConfig.SPI_CPHA		= SPI_CPHA_LO;
	SPI1hanle.SPIConfig.SPI_SSM			= SPI_SSM_DI;			//Hardware slave management enable for NSS pin

	SPI_Init(&SPI1hanle);

}

void GPIO_ButtonInit(GPIO_Handle_t *GPIOBtn,GPIO_RegDef_t *pGPIO, uint8_t PinNumber){
	// Set GPIO Button :

	//Configure GPIO Button for push-pull mode
	GPIOBtn->pGPIOx = pGPIO;
	GPIOBtn->GPIO_PinConfig.GPIO_PinNumber 	  = PinNumber;
	GPIOBtn->GPIO_PinConfig.GPIO_PinMode	  = GPIO_MODE_IN;
	GPIOBtn->GPIO_PinConfig.GPIO_CNF		  = GPIO_IP_FLOAT;

	//Enable GPIO
	GPIO_PeriClockControl(pGPIO, ENABLE);

	//Init GPIO Button
	GPIO_Init(GPIOBtn);
}

int main(void){

	char user_data[] = "Hello World";
	//This function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOs();
	//AFIO_PCLK_EN();
	//This function is used to initialize the SPI1 peripheral parameter
	SPI1_Init();

	/*
	 * making SSOE1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high whine SPE = 0
	 */
	SPI_SSOEConfig(SPI1, ENABLE);

	GPIO_Handle_t GPIOBtn;
	GPIO_ButtonInit(&GPIOBtn, GPIOA, GPIO_PIN_NO_12 );
	while(1){
		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12));
		delay();
		//Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1,ENABLE);

		//first send length information
		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI1,&datalen,1);

		//to sent data
		SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

		//lets confỉm SPI not busy

		while( SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG) );
		//Enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1,DISABLE);
	}

	return 0;
}

