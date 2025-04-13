/*
 * x_gpio.c
 *
 *  Created on: Apr 1, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_gpio.h"


/*

 * @fn						- GPIO_PeriphClkCtrl
 *
 * @brief					- this function enables or disables the peripheral clock for the given GPIO port
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 * @param[en_di]			- ENABLE or DISABLE macros, ENABLE=1, DISABLE=0
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_PeriphClkCtrl(__R GPIOx_Reg_t *const ptr_GPIOx, uint8_t en_di)
{
	if (ptr_GPIOx == GPIOA) (en_di) ? GPIOA_CLK_EN() : GPIOA_CLK_DI();

	else if (ptr_GPIOx == GPIOB) (en_di) ? GPIOB_CLK_EN() : GPIOB_CLK_DI();

	else if (ptr_GPIOx == GPIOC) (en_di) ? GPIOC_CLK_EN() : GPIOC_CLK_DI();

	else;
}

/*
void func1(uint32_t *p, uint8_t e)
{
	if ( p == (uint32_t *)0x48000000UL ) (e) ? *((uint32_t *)0x4002104CUL) |= 0x1 : *((uint32_t *)0x4002104CUL) &= ~0x1;

	else if ( p == (uint32_t *)0x48000400UL ) (e) ? *((uint32_t *)0x4002104CUL) |= 0x2 : *((uint32_t *)0x4002104CUL) &= ~0x2;

	else if ( p == (uint32_t *)0x48000800UL ) (e) ? *((uint32_t *)0x4002104CUL) |= 0x4 : *((uint32_t *)0x4002104CUL) &= ~0x4;

	else;
}
*/

/*

 * @fn						- GPIO_Init
 *
 * @brief					- this function initializes the given GPIO port/pin i.e. configures it with the provided settings
 *
 * @param[ptr_GPIOHandle]	- pointer to the base address of the GPIO Handle structure
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_Init(__RH GPIO_Handle_t *const ptr_GPIOHandle)
{
	/* Enable the peripheral clock */
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)ptr_GPIOHandle->ptr_GPIOx, ENABLE);

	/* 1. Configure the GPIO Pin Mode */
	const uint8_t pin_mode = ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinMode;

	switch (pin_mode)
	{
		/* Non-interrupt functions */
		case INPUT:
		case OUTPUT:
		case ALTERNATE:
		case ANALOG:
			/* Extract the pin number
			 * Note that each pin in MODER takes 2 bits, so double bit-shift to extracted pin number
			 * Clear the desired location
			 * Bit-shift the desired pin mode by (2 * pin number) positions
			 * Write this to the GPIO's mode register
			 */
			ptr_GPIOHandle->ptr_GPIOx->MODER &= ~( CLEAR_TWO_BITMASK << (2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
			ptr_GPIOHandle->ptr_GPIOx->MODER |= ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinMode ) << ( 2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
			break;

		/* Interrupt functions */
		case INTERRUPT_FT:
		case INTERRUPT_RT:
		case INTERRUPT_RFT:
			/* Pin must be configured as an input pin - '00' so no need to set */
			ptr_GPIOHandle->ptr_GPIOx->MODER &= ~( CLEAR_TWO_BITMASK << (2 * ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

			SYSCFG_CLK_EN();
			// see data sheet, there exists 4 EXTI control registers (EXTICR)
			const uint8_t exticr_port_number = ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			const uint8_t pin_offset = ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			const uint8_t port_code = SYSCFG_EXTICR_helper_func((__R GPIOx_Reg_t *const)ptr_GPIOHandle->ptr_GPIOx);

			// Note that each pin in EXTICR takes 3 bits, so triple bit-shift
			SYSCFG->EXTICR[exticr_port_number] |= (port_code << 4 * pin_offset);

			EXTI->IMR1 |= (SET_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			switch (pin_mode)
			{
				case INTERRUPT_FT:
					// Enable falling edge triggering - FTSR
					EXTI->FTSR1 |= (SET_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					// Disable corresponding rising edge triggering - RTSR (precaution)
					EXTI->RTSR1 &= ~(CLEAR_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					break;

				case INTERRUPT_RT:
					// Enable rising edge triggering - RTSR
					EXTI->RTSR1 |= (SET_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					// Disable corresponding falling edge triggering - FTSR (precaution)
					EXTI->FTSR1 &= ~(CLEAR_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					break;

				case INTERRUPT_RFT:
					// Enable both FTSR and RTSR
					EXTI->RTSR1 |= (SET_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					EXTI->FTSR1 |= (SET_ONE_BITMASK << ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					break;

				default:
					break;
			}
	}
	if (pin_mode == OUTPUT)
	{
		/* 2. Configure the GPIO Pin Output Speed
		 * Extract the pin number
		 * Note that each pin in OSPEEDR takes 2 bits, so double bit-shift to extracted pin number
		 * Clear the desired location
		 * Bit-shift the desired pin speed by (2 * pin number) positions
		 * Write this to the GPIO's output speed register
		 */
		ptr_GPIOHandle->ptr_GPIOx->OSPEEDR &= ~( CLEAR_TWO_BITMASK << (2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
		ptr_GPIOHandle->ptr_GPIOx->OSPEEDR |= ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinOSpeed ) << ( 2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

		/* 3. Configure the GPIO Pin Output Type
		 * Extract the pin number
		 * Note that each pin in OTYPER takes 1 bit, so single bit-shift to extracted pin number
		 * Clear the desired location
		 * Bit-shift the desired pin resistor configuration by (pin number) positions
		 * Write this to the GPIO's output type register
		 */
		ptr_GPIOHandle->ptr_GPIOx->OTYPER &= ~( CLEAR_ONE_BITMASK << (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		ptr_GPIOHandle->ptr_GPIOx->OTYPER |= ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinOType ) << ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	/* 4. Configure the GPIO Pin Internal Resistor Configuration
	 * Extract the pin number
	 * Note that each pin in PUPDR takes 2 bits, so double bit-shift to extracted pin number
	 * Clear the desired location
	 * Bit-shift the desired pin resistor configuration by (2 * pin number) positions
	 * Write this to the GPIO's pull-up/-down register
	 */
	ptr_GPIOHandle->ptr_GPIOx->PUPDR &= ~( CLEAR_TWO_BITMASK << (2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) );
	ptr_GPIOHandle->ptr_GPIOx->PUPDR |= ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl ) << ( 2 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

	/* 5. Configure the GPIO Pin Alternate Functionality, if set to it
	 * Extract the pin number
	 * Note that each pin in AFR takes 4 bits, so quad bit-shift to extracted pin number
	 * Clear the desired location
	 * Bit-shift the desired pin alternate function by (4 * pin number) positions
	 * Write this to the GPIO's alternate function register
	 */
	if (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == ALTERNATE)
	{
		uint8_t correction_factor = ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		volatile uint32_t *const temp = (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber < PIN_EIGHT) ? &(ptr_GPIOHandle->ptr_GPIOx->AFRL) : &(ptr_GPIOHandle->ptr_GPIOx->AFRH);
		*temp &= ~( CLEAR_FOUR_BITMASK << (4 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber - (correction_factor * 8) )) );
		*temp |= ( ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc ) << ( 4 * (ptr_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber - (correction_factor * 8) ) );
	}
}

/*

 * @fn						- GPIO_DeInit
 *
 * @brief					- this function resets all registers of a given GPIO port through the RCC reset register
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_DeInit(__RH GPIOx_Reg_t *const ptr_GPIOx)
{
	if (ptr_GPIOx == GPIOA) GPIOA_REG_RESET();

	else if (ptr_GPIOx == GPIOB) GPIOB_REG_RESET();

	else if (ptr_GPIOx == GPIOC) GPIOC_REG_RESET();

	else;
}

/*

 * @fn						- GPIO_ReadPin
 *
 * @brief					- this function reads and returns the contents of a GPIO pin's input data register
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 * @param[pin_number]		- GPIO pin number
 *
 * @return					- input GPIO pin's state/contents
 *
 * @Note					- none

 */
uint8_t GPIO_ReadPin(__R GPIOx_Reg_t *const ptr_GPIOx, uint8_t pin_number)
{
//	return (uint8_t)( ptr_GPIOx->IDR & (CHECK_ONE_BITMASK << pin_number) );
	return get_flag_status(ptr_GPIOx->IDR, 1, pin_number);
}

/*

 * @fn						- GPIO_ReadPort
 *
 * @brief					- this function reads and returns the contents of a GPIO port's input data register
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 *
 * @return					- GPIO port's IDR contents
 *
 * @Note					- none

 */
uint16_t GPIO_ReadPort(__R GPIOx_Reg_t *const ptr_GPIOx)
{
	return ( (uint16_t)ptr_GPIOx->IDR );
}

/*

 * @fn						- GPIO_WritePin
 *
 * @brief					- this function writes a value @value to the given GPIO pin @pin_number
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 * @param[pin_number]		- pin to write to
 * @param[value]			- value to write
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_WritePin(__W GPIOx_Reg_t *const ptr_GPIOx, uint8_t pin_number, uint8_t value)
{
	if (value == SET) ptr_GPIOx->ODR |= (SET_ONE_BITMASK << pin_number);

	else if (value == RESET) ptr_GPIOx->ODR &= ~(CLEAR_ONE_BITMASK << pin_number);

	else;
}

/*

 * @fn						- GPIO_WritePort
 *
 * @brief					- this function writes a value @value to the given GPIO port
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 * @param[value]			- value to write to the GPIO port's IDR
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_WritePort(__W GPIOx_Reg_t *const ptr_GPIOx, uint16_t value)
{
	ptr_GPIOx->ODR = value;
}

/*

 * @fn						- GPIO_TogglePin
 *
 * @brief					- this function toggles the desired pin's output value
 *
 * @param[ptr_GPIOx]		- pointer to the base address of the GPIO peripheral
 * @param[pin_number]		- GPIO pin to toggle
 *
 * @return					- none
 *
 * @Note					- none

 */
void GPIO_TogglePin(__RW GPIOx_Reg_t *const ptr_GPIOx, uint8_t pin_number)
{
	ptr_GPIOx->ODR ^= (SET_ONE_BITMASK << pin_number);
}

/* GPIO Interrupt Configuration/Handling */
void GPIO_IRQNumberConfig(uint8_t IRQNumber, uint8_t en_di)
{
	/* Each bit in an ISER register represents an IRQ number,
	 * Check data sheet for number of available/pre-configured IRQ numbers */
	const uint8_t iser_reg_x = IRQNumber / 32;
	const uint8_t iser_irq_offset = IRQNumber % 32;

	const uint32_t irq_bitmask = (SET_ONE_BITMASK << iser_irq_offset);

	switch (en_di)
	{
		case ENABLE:
			if (iser_reg_x == 0) *NVIC_ISER0 |= irq_bitmask;

			else if (iser_reg_x == 1) *NVIC_ISER1 |= irq_bitmask;

			else if (iser_reg_x == 2) *NVIC_ISER2 |= irq_bitmask;

			else if (iser_reg_x == 3) *NVIC_ISER3 |= irq_bitmask;
			break;

		case DISABLE:
			if (iser_reg_x == 0) *NVIC_ICER0 |= irq_bitmask;

			else if (iser_reg_x == 1) *NVIC_ICER1 |= irq_bitmask;

			else if (iser_reg_x == 2) *NVIC_ICER2 |= irq_bitmask;

			else if (iser_reg_x == 3) *NVIC_ICER3 |= irq_bitmask;
			break;

		default:
			break;
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint16_t IRQPriority)
{
	// 1. Find priority register
	const uint8_t priority_reg_x = IRQNumber / 4;
	const uint8_t priority_irq_offset = IRQNumber % 4;

	const uint8_t shift_amount = (priority_irq_offset * 8) + NO_PR_BITS_IMPLEMENTED;

	/* Remember underlying working of pointer arithmetic */
	*(NVIC + priority_reg_x) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandler(uint8_t pin_number)
{
	// clear the Pending Interrupt register in the EXTI peripheral
	if ( get_flag_status(EXTI->PR1, 1, pin_number) ) ( EXTI->PR1 |= (SET_ONE_BITMASK << pin_number) );
}

uint8_t SYSCFG_EXTICR_helper_func(__R GPIOx_Reg_t *const port)
{
	if (port == GPIOA) return (uint8_t) 0;

	else if (port == GPIOB) return (uint8_t) 1;

	else if (port == GPIOC) return (uint8_t) 2;

	else return (-1);
}

