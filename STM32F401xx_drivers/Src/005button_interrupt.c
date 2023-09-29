/*
 * 005button_interrupt.c
 *
 *  Created on: 14 Tem 2023
 *      Author: alprn
 */


#include "stm32f401xx.h"
#include <stdio.h>
#include <string.h>

void delay(void) //This will introduce ~200ms delay when system clock is 16Mhz.
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

int main(void)
{
	GPIO_Handle_t Gpio_Led, Gpio_Button ;
	memset(&Gpio_Led, 0, sizeof(Gpio_Led));
	memset(&Gpio_Button, 0, sizeof(Gpio_Button));

	Gpio_Led.pGPIOx = GPIOA ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6 ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ;
	Gpio_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Gpio_Led);


	Gpio_Button.pGPIOx = GPIOD ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2 ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	Gpio_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU ;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&Gpio_Button);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);

	while(1);
}

void EXTI2_IRQHandler(void) //Interrupt Service Routine (ISR)
{
	delay(); //200ms
	GPIO_IRQHandling(GPIO_PIN_2);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_6);
}




