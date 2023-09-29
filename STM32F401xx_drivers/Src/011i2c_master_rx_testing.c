/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: 1 Eyl 2023
 *      Author: alprn
 */


#include "stm32f401xx.h"
#include <string.h>
#include <stdio.h>

#define MY_ADDR			0x61
#define SLAVE_ADDR 		0x68

extern void initialise_monitor_handles();

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

I2C_Handle_t I2C1Handle ;

//Some data
uint8_t rcv_buf[32]; //Make sure you don't send more then 32 bytes, that is arduino wire lib. limit.

/*
 * PB9 -> SDA
 * PB8 -> SCL
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2C1Pins;

	I2C1Pins.pGPIOx = GPIOB;

	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4 ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD ; //Need for open drain, I2C will need it.
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;
	I2C1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8 ;
	GPIO_Init(&I2C1Pins);

	//SDA
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9 ;
	GPIO_Init(&I2C1Pins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2 ;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM ;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t Gpio_Button ;

	Gpio_Button.pGPIOx = GPIOB ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5 ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;

	GPIO_Init(&Gpio_Button);
}

int main()
{
	initialise_monitor_handles();

	printf("Application is running\n");

	uint8_t commandcode ;

	uint8_t len ;

	//Button Init.
	GPIO_ButtonInit();

	//I2C pin Init.
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the i2c peripheral

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
	//wait for button press
	while(! GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_5) );

	//to avoid button debouncing related issues 200ms of delay.
	delay();

	commandcode = 0x51;

	I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

	I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

	commandcode = 0x52;

	I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

	I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);

	rcv_buf[len+1] = '\0';

	printf("Data : %s", rcv_buf);
	}
}
