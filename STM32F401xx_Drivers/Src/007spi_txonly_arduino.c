/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: 26 Tem 2023
 *      Author: alprn
 */

/*
*PB15 -> SPI2_MOSI
*PB14 -> SPI2_MISO
*PB13 -> SPI2_SCLK
*PB12 -> SPI2_NSS
*
*ALT function mode : AF05
*/

#include "stm32f401xx.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5 ;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ; //No need for open drain, I2C will need it.
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13 ;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12 ;
	GPIO_Init(&SPIPins);
//*********************For this application we don't want to use MISO.
//*********************So, don't configure this pin and this is free for other uses.
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14 ;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15 ;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD ;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER ;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8 ; //2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS ;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW ; //DEFAULT
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW ; //DEFAULT
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI ; //Hardware Slave management for NSS pin.

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t Gpio_Button ;

	Gpio_Button.pGPIOx = GPIOB ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5 ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;

	GPIO_Init(&Gpio_Button);
}

int main()
{
	char user_data[] = "Hello World" ;

		GPIO_ButtonInit();

		//this function is used to initialize the GPIO pins to behave as SPI2 pins.
		SPI2_GPIOInits();

		SPI2_Inits();

	   /*
		* making SSOE 1 does NSS output enable.
		* The NSS pin is automatically managed by the hardware.
		* i.e when SPE=1 , NSS will be pulled to low
		* and NSS pin will be high when SPE=0
		*/

		SPI_SSOEConfig(SPI2, ENABLE);

		while(1)
		{

			while(! GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_5) );

			//to avoid button debouncing related issues 200ms of delay.
			delay();

			//Enable the spı2 peripheral.
			SPI_PeripheralControl(SPI2, ENABLE);

			//First send lenght information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen, 1);

			//to send data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

			//Disable the SPI2 peripheral.
			SPI_PeripheralControl(SPI2, DISABLE);

		}



		return 0 ;
}
