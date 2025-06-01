/*
 * 008_spi_message_rcv_it.c
 *
 *  Created on: Jun 1, 2025
 *      Author: hiephuu2001
 */

#include <stdio.h>
#include <string.h>
#include <stm32f103xx.h>

//command codes

#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LED_OFF						0

//arduino analog pins

#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4

//arduino led
#define LED_PIN						9

SPI_Handle_t SPI1Handle;

uint8_t RcvBuff[100];
uint8_t Readbyte;
uint8_t RxContFlag = RESET;

void delay()
{
	for(uint32_t i = 0 ; i < 500000/2;i++);
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
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIO_Init(&SPIPins);

		//NSS
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&SPIPins);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_IP_PUPD;
		GPIO_Init(&SPIPins);

	} else
	{
		GPIO_Handle_t SPIPins;
		GPIO_Handle_t SPIPinsA;

		//Config GPIOA and GPIOB
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode 	= GPIO_MODE_OUT_10MHz ;
		SPIPins.GPIO_PinConfig.GPIO_CNF			= GPIO_ALTFN_OP_PP;

		SPIPinsA.pGPIOx = GPIOA;
		SPIPinsA.GPIO_PinConfig.GPIO_PinMode 	= GPIO_MODE_OUT_10MHz ;
		SPIPinsA.GPIO_PinConfig.GPIO_CNF		= GPIO_ALTFN_OP_PP;

		//CLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
		GPIO_Init(&SPIPins);


		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
		GPIO_Init(&SPIPins);

		//NSS
		SPIPinsA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPinsA);

		//MISO
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
		SPIPins.GPIO_PinConfig.GPIO_CNF		= GPIO_IP_PUPD;
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

	SPI_PeriClockControl(SPI1, ENABLE);
	SPI_Init(&SPI1hanle);

}

void GPIO_Config_Init(){
	// Set GPIO Button :
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber 	  = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode	 	  = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF		  	  = GPIO_IP_PUPD;

	//Enable GPIO
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//Init GPIO Button
	GPIO_Init(&GPIOBtn);

	//this is btn gpio configuration
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 	  = GPIO_PIN_NO_11;
	GpioLed.GPIO_PinConfig.GPIO_PinMode	 	  = GPIO_MODE_OUT_10MHz;
	GpioLed.GPIO_PinConfig.GPIO_CNF		  	  = GPIO_GP_OP_PP;

	//Enable GPIO
	GPIO_PeriClockControl(GPIOB, ENABLE);

	//Init GPIO Button
	GPIO_Init(&GpioLed);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	// Config GPIO pin button and GPIO pin led
	GPIO_Config_Init();

	// This function is used to initialize the GPIO pins to behave as SPI1
	SPI1_GPIOs();
	SPI1_Init();

	/*
	 * making SSOE1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high whine SPE = 0
	 */

	SPI_SSOEConfig(SPI1, ENABLE);

	SPI_IRQInterruptConfig(IQR_NO_SPI1, ENABLE);

	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12));

	// to avoid button de-bouncing related issues 200ms of delay
	delay();

	//enable the SPI1 peripheral
	SPI_PeriClockControl(SPI1, ENABLE);

	RxContFlag = SET;

	while(RxContFlag == SET)
	{
		while(!SPI_ReceiveDataIT(&SPI1Handle, &Readbyte, 1) == SPI_READY);
	}

	//let comfirm SPI is not busy
	while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

	//disable the SPI peripheral
	SPI_PeriClockControl(SPI1, DISABLE);

	return 0;
}


void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1Handle);
}

void SPI_ApplicationEventCallBack(SPI_Handle_t *pHanle,uint8_t AppEv) {
	static uint32_t i = 0;
	static uint8_t rcv_start = 0;

	if(AppEv == SPI_EVENT_RX_CMPLT )
	{
		if(Readbyte == 0XF1)
		{
			rcv_start= 1;
		} else {
			if(rcv_start){
				if(Readbyte == '\r')
				{
					RxContFlag = RESET;
					rcv_start = 0;
					RcvBuff[i++] = Readbyte;
					i =0;
				} else {
					RcvBuff[i++] = Readbyte;
				}
			}
		}
	}
}
