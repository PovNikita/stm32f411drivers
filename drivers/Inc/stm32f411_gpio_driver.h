/*
 * stm32f411xx_gpio_driver.h
 *
 */

#ifndef INC_STM32F411_GPIO_DRIVER_H_
#define INC_STM32F411_GPIO_DRIVER_H_

#include "stm32f411.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			/* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/* Possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;			/* Possible values from @GPIO_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;     /* Possible values from 0b0000 to 0b1111 */
}GPIO_PinConfig_t;

typedef struct
{
	//pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIO;				/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */
}GPIO_Handle_t;

/*
 * GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0		//This is INPUT_MODE
#define GPIO_MODE_OUT		1		//This is OUTPUT_MODE
#define GPIO_MODE_ALTFN		2		//This is ALTERNATE_FUNCTION mode
#define GPIO_MODE_ANALOG	3		//This is ANALOG_MODE
#define GPIO_MODE_IT_FT		4		//This is INTERRUPT_MODE FALLING_TRIGGERING
#define GPIO_MODE_IT_RT		5		//This is INTERRUPT_MODE RISING_TRIGGERING
#define GPIO_MODE_IT_RFT	6		//This is INTERRUPT_MODE RISING_AND_FALLING_TRIGGERING

/*
 * GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0		//This is push-pull state for output mode
#define GPIO_OP_TYPE_OD		1		//This is open-drain state for output mode

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0		//No pull-up or pull-down resistor
#define GPIO_PU				1		//Pull-up resistor
#define GPIO_PD				2		//Pull-down resistor

/***********************************************************
 * 				APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 ***********************************************************/

/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t  *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQConfiguration and ISR handling
 */

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411_GPIO_DRIVER_H_ */
