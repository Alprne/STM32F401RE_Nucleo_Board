/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: 3 Tem 2023
 *      Author: alprn
 */

#include "stm32f401xx_gpio_driver.h"


//Peripheral Clock Setup
/*****************************************************************
* @fn 				- GPIO_PeriClockControl
*
* @brief 			- This function enables or disables peripheral clock for the given GPIO port.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		- Enable or Disable macros
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}
}

//Inıt and De-ınıt
/*****************************************************************
* @fn 				- GPIO_Init
*
* @brief 			- This function initialize the given GPIO.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0 ; //temp register

	//0.Enable the peripheral clock.

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//The non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp ; //setting

	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Just in case clear the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Just in case clear the FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO Port selection in SYSCFG_EXTI
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;

		uint8_t portcode = GPIO_BASE_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PLCK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0 ; //Reset the temp register.

	//2. Configure the speed.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp ;

	temp = 0;

	//3. Configure the pupd settings.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp ;

	temp = 0;

	//4. Configure the optype.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp ;

	temp = 0;


	//5. Configure the alt functionality.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}

/*****************************************************************
* @fn 				- GPIO_Deinit
*
* @brief 			- This function deinitialize the given GPIO.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

//Data read and write
/*****************************************************************
* @fn 				- GPIO_ReadFromInputPin
*
* @brief 			- This function used to read data from the input GPIO Pin.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		- Pin Number
* @param[in] 		-
*
* @return 			- 0 or 1
*
* @Note 			- None
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
 uint8_t value;

 value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0X00000001);

 return value;
}

/*****************************************************************
* @fn 				- GPIO_ReadFromInputPort
*
* @brief 			- This function used to read data from the input GPIO Port.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		-
* @param[in] 		-
*
* @return 			- 16 bit binary input data
*
* @Note 			- None
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR ;

	return value;
}

/*****************************************************************
* @fn 				- GPIO_WriteToOutputPin
*
* @brief 			- This function used to write data from the output GPIO Pin.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		- Pin Number
* @param[in] 		- Value
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//Write 1 to output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber); //Reset
		pGPIOx->ODR |= (1 << PinNumber);  //Set
	}else
	{
		//Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*****************************************************************
* @fn 				- GPIO_WriteToOutputPort
*
* @brief 			- This function used to write data from the output GPIO Port.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		- Value
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= ~(0xFFFF << 16);
	pGPIOx->ODR = Value ;
}

/*****************************************************************
* @fn 				- GPIO_ToggleOutputPin
*
* @brief 			- This function used to toggle the output GPIO Pin.
*
* @param[in] 		- Base address of the GPIO peripheral
* @param[in] 		- Pin Number
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ Configuration and ISR Handling
/*****************************************************************
* @fn 				- GPIO_IRQInterruptConfig
*
* @brief 			- This function is enable or disable the interrupt in the ARM M4 processor
*
* @param[in] 		- IRQ Number
* @param[in] 		- Enable or Disable
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96) //64 to 96
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64) );

		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64) );
		}
	}
}

/*****************************************************************
* @fn 				- GPIO_IRQPriorityConfig
*
* @brief 			- This function is configure the Priority of the Interrupt in other Interrupts
*
* @param[in] 		- IRQPriority
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First Let's find out the irp register.
	uint8_t irpx = IRQNumber / 4 ;
	uint8_t irpx_section = IRQNumber % 4 ;

	uint8_t shift_amount = (8 * irpx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + irpx ) |= (IRQPriority << shift_amount);
}

/*****************************************************************
* @fn 				- GPIO_IRQHandling
*
* @brief 			- ?
*
* @param[in] 		- Pin Number
* @param[in] 		-
* @param[in] 		-
*
* @return 			- None
*
* @Note 			- None
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//CLEAR
		EXTI->PR |= (1 << PinNumber);
	}
}











