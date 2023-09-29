/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: 3 Tem 2023
 *      Author: alprn
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"
#include <stdint.h>

//This is Configuration structure for GPIO pin

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				/*!<Possible values from @GPIO_PIN_MODES> */
	uint8_t GPIO_PinSpeed;				/*!<Possible values from @GPIO_PIN_SPEED> */
	uint8_t GPIO_PinPuPdControl;		/*!<Possible values from @GPIO_PIN_PUPD_CRTL> */
	uint8_t GPIO_PinOPType;				/*!<Possible values from @GPIO_PIN_OP_TYPE> */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


//This is Handle structure for GPIO pin

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERs
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Possible Modes
*/
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT 			1
#define GPIO_MODE_ALTFN 		2
#define GPIO_MODE_ANALOG 		3
#define GPIO_MODE_IT_FT 		4 		//Input falling edge trigger
#define GPIO_MODE_IT_RT 		5 		//Input raising edge trigger
#define GPIO_MODE_IT_RFT 		6 		//Input falling raising edge

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin Possible Output Speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD_CRTL
 * GPIO Pin Pull-Up and Pull-Down Configuration Macros
 */
#define GPIO_NO_PUPD			0
#define	GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO Pin Possible Output Types
 */
#define GPIO_OP_TYPE_PP			0		//Output Type Push-Pull
#define GPIO_OP_TYPE_OD			1		//Output Type Open-Drain

//*******************************************************************************************************
//									APIs Supported For This Driver
//					For more information about the APIs check the function definitions
//*******************************************************************************************************


//Peripheral Clock Setup

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Inıt and De-ınıt

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR Handling

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);









#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
