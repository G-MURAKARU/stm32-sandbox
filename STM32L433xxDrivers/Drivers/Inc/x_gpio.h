/*
 * x_gpio.h
 *
 *  Created on: Apr 1, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_GPIO_H_
#define INC_X_GPIO_H_


#include "x_gpio_defs.h"


/* RCC - GPIOA mapping structure, for clock control */
static const RCC_Periph_t RCC_MAP_GPIOA = { .RCC_BitPos = RCC_GPIOA, .RCC_Bus = RCC_AHB2 };

/* RCC - GPIOB mapping structure, for clock control */
static const RCC_Periph_t RCC_MAP_GPIOB = { .RCC_BitPos = RCC_GPIOB, .RCC_Bus = RCC_AHB2 };

/* RCC - GPIOC mapping structure, for clock control */
static const RCC_Periph_t RCC_MAP_GPIOC = { .RCC_BitPos = RCC_GPIOC, .RCC_Bus = RCC_AHB2 };

/*
 * @GPIO_PERIPHERAL_REGISTERS
 * Initializer for a GPIO peripheral instance, containing all memory-mapped registers
 */
typedef struct GPIO_PeripheralRegisters
{
	__RW uint32_t MODER;								/* GPIO Port Mode Register */
	__RW uint32_t OTYPER;								/* GPIO Port Output Type Register */
	__RW uint32_t OSPEEDR;								/* GPIO Port Output Speed Register */
	__RW uint32_t PUPDR;								/* GPIO Port Pull-Up/Pull-Down Register */
	__RH uint32_t IDR;									/* GPIO Port Input Data Register */
	__RW uint32_t ODR;									/* GPIO Port Output Data Register */
	__W  uint32_t BSRR;									/* GPIO Port Bit Set-Reset Register */
	__RW uint32_t LCKR;									/* GPIO Port Configuration Lock Register */
	__RW uint32_t AFRL;									/* GPIO Port Alternate Function Low Register */
	__RW uint32_t AFRH;									/* GPIO Port Alternate Function High Register */
	__W  uint32_t BRR;									/* GPIO Port Bit Reset Register */
} GPIOx_Reg_t;

/*
 * @GPIO_PIN_CONFIGURATION
 * Initializer for a GPIO pin configuration structure, containing all pin settings
 */
typedef struct GPIOPinConfiguration
{
	uint8_t GPIO_PinNumber;								/* !< Possible values from @GPIO_PIN_NUMBERS > */
	uint8_t GPIO_PinMode;								/* !< Possible values from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinOutSpeed;							/* !< Possible values from @GPIO_PIN_OUTPUT_SPEEDS > */
	uint8_t GPIO_PinPUPDControl;						/* !< Possible values from @GPIO_PIN_RESISTOR_CONFIGS > */
	uint8_t GPIO_PinOutType;							/* !< Possible values from @GPIO_PIN_OUTPUT_TYPES > */
	uint8_t GPIO_PinAltFunc;							/* !< Possible values from @GPIO_PIN_ALTERNATE_FUNCTIONS > */
	uint8_t GPIO_PinIRQPriority;						/* !< Possible values from @NVIC_IRQ_PRIORITY_NUMBERS > */
} GPIO_PinConfig_t;

/*
 * @GPIO_HANDLER
 * Initializer for a GPIO handler structure, that manages a GPIO instance's configuration and properties
 */
typedef struct GPIOHandleStructure
{
	GPIOx_Reg_t 	  *ptr_GPIOx;						/* This holds a pointer to the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;					/* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO possible pin numbers
 */
typedef enum GPIOPinNumbers
{
	GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5,
	GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
	GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15,
} GPIO_Pin_e;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
typedef enum GPIOModes
{
	GPIO_INPUT, GPIO_OUTPUT, GPIO_ALTERNATE, GPIO_ANALOG,
	GPIO_INTERRUPT_FT, GPIO_INTERRUPT_RT, GPIO_INTERRUPT_RFT,
} GPIO_Mode_e;

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
typedef enum GPIOOutputTypes
{
	GPIO_PUSH_PULL, GPIO_OPEN_DRAIN,
} GPIO_OType_e;

/*
 * @GPIO_PIN_OUTPUT_SPEEDS
 * GPIO pin possible output speeds
 */
typedef enum GPIOOutputSpeeds
{
	GPIO_LOW_SPEED, GPIO_MEDIUM_SPEED, GPIO_HIGH_SPEED, GPIO_VERY_HIGH_SPEED,
} GPIO_OSpeed_e;

/*
 * @GPIO_PIN_RESISTOR_CONFIGS
 * GPIO pin possible internal resistor configurations
 */
typedef enum GPIOPullUpPullDownResistors
{
	GPIO_NO_PUPDN, GPIO_PULL_UP, GPIO_PULL_DOWN,
} GPIO_PUpDn_e;

/*
 * @GPIO_PIN_ALTERNATE_FUNCTIONS
 * GPIO pin possible alternate function configurations
 */
typedef enum GPIOAlternateFunctions
{
	GPIO_ALT_0, GPIO_ALT_1, GPIO_ALT_2, GPIO_ALT_3, GPIO_ALT_4, GPIO_ALT_5,
	GPIO_ALT_6, GPIO_ALT_7, GPIO_ALT_8, GPIO_ALT_9, GPIO_ALT_10, GPIO_ALT_11,
	GPIO_ALT_12, GPIO_ALT_13, GPIO_ALT_14, GPIO_ALT_15,
} GPIO_Alt_e;


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/* GPIO Clock Configuration */
void GPIO_PeriphClkCtrl(__R GPIOx_Reg_t *const, bool);

/* GPIO Initialization */
void GPIO_Init(__R GPIO_Handle_t *const);
void GPIO_DeInit(__RH GPIOx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* GPIO Data Read and Write, static inline functions */

/*

 * @fn						- GPIO_ReadPin
 *
 * @brief					- this function reads and returns the contents of a GPIO pin's input data register
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 * @param pin_number		- GPIO pin number
 *
 * @return					- input GPIO pin's state/contents
 *
 * @Note					- none

 */
static __always_inline uint8_t GPIO_ReadPin(__R GPIOx_Reg_t *const ptr_GPIOx, uint8_t pin_number)
{
 	return ( (ptr_GPIOx->IDR >> pin_number) & CHECK_ONE_BITMASK ) ? 1 : 0;
}

/*

 * @fn						- GPIO_ReadPort
 *
 * @brief					- this function reads and returns the contents of a GPIO port's input data register
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 *
 * @return					- GPIO port's IDR contents
 *
 * @Note					- none

 */
static __always_inline uint16_t GPIO_ReadPort(__R GPIOx_Reg_t *const ptr_GPIOx)
{
	return ( (uint16_t)ptr_GPIOx->IDR );
}

/*

 * @fn						- GPIO_WritePort
 *
 * @brief					- this function writes a value @value to the given GPIO port
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 * @param value				- value to write to the GPIO port's IDR
 *
 * @return					- none
 *
 * @Note					- none

 */
static __always_inline void GPIO_WritePort(GPIOx_Reg_t *const ptr_GPIOx, uint16_t value)
{
	ptr_GPIOx->ODR = value;
}

/*

 * @fn						- GPIO_SetPin
 *
 * @brief					- this function sets the output value of given GPIO pins
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 * @param pin_mask			- mask containing the pin(s) to set
 *
 * @return					- none
 *
 * @Note					- this is inline function definition

 */
static __always_inline void GPIO_SetPin(GPIOx_Reg_t *const ptr_GPIOx, uint32_t pin_mask)
{
	ptr_GPIOx->BSRR = pin_mask;
}

/*

 * @fn						- GPIO_ResetPin
 *
 * @brief					- this function resets the output value of given GPIO pins
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 * @param pin_mask			- mask containing the pin(s) to reset
 *
 * @return					- none
 *
 * @Note					- this is inline function definition

 */
static __always_inline void GPIO_ResetPin(GPIOx_Reg_t *const ptr_GPIOx, uint32_t pin_mask)
{
	ptr_GPIOx->BSRR = (pin_mask << 16);
}

/*

 * @fn						- GPIO_TogglePin
 *
 * @brief					- this function toggles the desired pin(s)'s output value
 *
 * @param ptr_GPIOx			- pointer to the base address of the GPIO peripheral
 * @param pin_mask			- mask containing the pin(s) to toggle
 *
 * @return					- none
 *
 * @Note					- none

 */
static __always_inline void GPIO_TogglePin(GPIOx_Reg_t *const ptr_GPIOx, uint32_t pin_mask)
{
	const uint32_t odr_contents = ptr_GPIOx->ODR;

	// First part will toggle from HIGH to LOW, second part will toggle from LOW to HIGH
	// Remember, writing '0' to any part of BSRR does nothing,
	// So one of below operations will do nothing, hence no conflict

	// Below OR operation essentially 'concatenates' two 16-bit masks into one 32-bit mask, no overwrites
	// This allows for multiple pin manipulations at once
	ptr_GPIOx->BSRR = ( ((odr_contents & pin_mask) << 16) | (~odr_contents & pin_mask) );
}

/* GPIO Interrupt Configuration/Handling */
void GPIO_IRQHandlerFunc(uint8_t);


#endif /* INC_X_GPIO_H_ */
