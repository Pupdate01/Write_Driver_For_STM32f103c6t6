/*
 * 009_I2C_Master_tx_testing.c
 *
 *  Created on: Jun 14, 2025
 *      Author: hiephuu2001
 */

#include<stdio.h>
#include<string.h>
#include"stm32f103xx.h"

#define MY_ADDR 	0x61
#define SLAVE_ADDR	0x68

I2C_Handle_t I2C1Handle;
uint8_t rcv_buf[32];

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void) {
	uint32_t I2C1rmval = Get_I2C_RM();
	GPIO_Handle_t I2CPins;
	if (I2C1rmval == 0) {
		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz;
		I2CPins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_OD;

		//Scl
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
		GPIO_Init(&I2CPins);

		//sda
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
		GPIO_Init(&I2CPins);
	} else {
		I2CPins.pGPIOx = GPIOB;
		I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_10MHz;
		I2CPins.GPIO_PinConfig.GPIO_CNF = GPIO_ALTFN_OP_OD;

		//Scl
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
		GPIO_Init(&I2CPins);

		//sda
		I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
		GPIO_Init(&I2CPins);
	}
}

void I2C1_Inits(void) {
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_CNF = GPIO_IP_PUPD;

	//Enable GPIO
	//GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOBtn);
}

int main(void) {

	//Config GPIO pin button
	//GPIO_ButtonInit();

	//enable AFIO
	AFIO_PCLK_EN();

	//This function is used to initialize the GPIO pins to behave as SPI1
	I2C1_GPIOInits();

	//I2C1 peripheral configuration
	I2C1_Inits();

	//enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while (1) {
		//wait till button is pressed
		//while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12));

		//to avoid button de-bouncing related issue 200ms delay
		delay();

		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data),SLAVE_ADDR, I2C_ENABLE_SR);
	}

}
