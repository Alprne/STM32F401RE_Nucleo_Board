/*
 * 002LedButton.c
 *
 *  Created on: 12 Tem 2023
 *      Author: alprn
 */
#include "stm32f401xx.h"
#include <stdio.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

int main(void)
{
	GPIO_Handle_t Gpio_Led ;

	Gpio_Led.pGPIOx = GPIOA ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5 ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Gpio_Led);

	GPIO_Handle_t Gpio_Button ;

	Gpio_Button.pGPIOx = GPIOC ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13 ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&Gpio_Button);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC , GPIO_PIN_13) == LOW)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		}
	}
	return 0;
}
