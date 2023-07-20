/*
 * stm32f411xx_gpio_driver.c
 *
 */

#include "stm32f411_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
/************************************************************
 * Fn					- GPIO_PeriCloclControl
 *
 * Brief				- This function enables or disables
 * 						peripheral clock for the given GPIO port
 *
 * Param[in]			- Base address of the GPIO peripheral
 * Param[in]			- ENABLE or DISABLE macros
 *
 * Return				- None
 *
 ************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t  *pGPIOx, uint8_t EnorDi)
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
	else if(EnorDi == DISABLE)
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

/*
 * Init and De-Init
 */

/************************************************************
 * Fn					- GPIO_Init
 *
 * Brief				- It configures the given GPIO pin
 *
 * Param[in]			- Address of the GPIO_Handle structure
 *
 * Return				- None
 *
 ************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; //temporary register

	//Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIO, ENABLE);

	//1. Configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIO->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIO->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//2. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//3. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1=( (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4 );
		uint8_t temp2=( ( (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4 )*4 );
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO);

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] |= (portcode << temp2);

		//3. Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed

	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIO->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->OSPEEDR |= temp;

	temp = 0;
	//3. Configure the PU/PD settings

	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIO->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type

	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIO->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER |= temp;

	temp = 0;
	//5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIO->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIO->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

	temp = 0;

}

/************************************************************
 * Fn					- GPIO_DeInit
 *
 * Brief				- It reset configuration of the given port
 *
 * Param[in]			- Base address of the GPIO peripheral
 *
 * Return				- None
 *
 ************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
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

/*
 * Data read and write
 */

/************************************************************
 * Fn					- GPIO_ReadFromInputPin
 *
 * Brief				- Return value from given Pin
 *
 * Param[in]			- The base address of the GPIO port
 * Param[in]			- PinNumber
 * Param[in]			-
 *
 * Return				- 0 or 1
 *
 ************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;

}

/************************************************************
 * Fn					- GPIO_ReadFromInputPort
 *
 * Brief				- Return value from given GPIO port
 *
 * Param[in]			- The base address of the GPIO port
 *
 * Return				- from 0 to 65535
 *
 ************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);

	return value;
}

/************************************************************
 * Fn					- GPIO_WriteToOutputPin
 *
 * Brief				- Write 1 or 0 to the output data register
 * 						at the bit field corresponding to the pin number
 *
 * Param[in]			- The base address of the GPIO port
 * Param[in]			- Pin Number
 * Param[in]			- Value for writing (1 or 0)
 *
 * Return				- None
 *
 ************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/************************************************************
 * Fn					- GPIO_WriteToOutputPort
 *
 * Brief				- Write value to the output data register
 * 						at the bit field corresponding to the GPIO port
 *
 * Param[in]			- The base address of the GPIO port
 * Param[in]			- Value for writing
 *
 * Return				- None
 *
 ************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/************************************************************
 * Fn					- GPIO_ToggleOutputPin
 *
 * Brief				- This function switches output pin
 *
 * Param[in]			- The base address of the GPIO port
 * Param[in]			- Pin Number
 *
 * Return				- None
 *
 *************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQConfiguration and ISR handling
 */

/************************************************************
 * Fn					- GPIO_IRQConfig
 *
 * Brief				- This function configures ISE(interrupt set-enable)
 * 						and ICE(interrupt clear-enable) registers of the processor
 *
 * Param[in]			- IRQ Number
 * Param[in]			- Enable or Disable macros
 *
 * Return				- None
 *
 *************************************************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));

		}
		else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				*NVIC_ICER0 |= (1 << IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64)
			{
				//program ICER1 register
				*NVIC_ICER1 |= (1 << (IRQNumber%32));
			}else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				//program ICER2 register
				*NVIC_ICER2 |= (1 << (IRQNumber%64));
			}
		}
	}
}

/************************************************************
 * Fn					- GPIO_IRQPriorityConfig
 *
 * Brief				- This function configures IRQ priority
 * 						by configuring interrupt priority registers
 * 						of the processor
 *
 * Param[in]			- IRQ Number
 * Param[in]			- IRQ Priority
 *
 * Return				- None
 *
 *************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Lets find out the IPR register
	uint8_t iprx = (IRQNumber/4);
	uint8_t iprx_section = (IRQNumber%4);

	uint8_t shift_amount = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << (shift_amount) );
}

/************************************************************
 * Fn					- GPIO_IRQHandling
 *
 * Brief				- This function handle IRQ from GPIO pin
 * 						by clearing the EXTI Pending Register corresponding
 * 						to the pin number
 *
 * Param[in]			- Pin Number
 *
 * Return				- None
 *
 **************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		// Clear
		EXTI->PR |= (1 << PinNumber);
	}
}

