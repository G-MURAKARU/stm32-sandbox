/*
 * x_gpio.h
 *
 *  Created on: Apr 1, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_GPIO_H_
#define INC_X_GPIO_H_


#include "stm32l433xx.h"


/* Structure forward declarations */
typedef struct GPIO_PeripheralRegisters GPIOx_Reg_t;

/* GPIO pointers to base registers */
#define GPIOA 											( (__RW GPIOx_Reg_t *const)GPIOA_BASE_ADDR )
#define GPIOB 											( (__RW GPIOx_Reg_t *const)GPIOB_BASE_ADDR )
#define GPIOC 											( (__RW GPIOx_Reg_t *const)GPIOC_BASE_ADDR )

/* Clock Enable for GPIOx */
#define GPIOA_CLK_EN() 								( (RCC->AHB2ENR) |= (SET_ONE_BITMASK << RCC_GPIOAEN) )
#define GPIOB_CLK_EN() 								( (RCC->AHB2ENR) |= (SET_ONE_BITMASK << RCC_GPIOBEN) )
#define GPIOC_CLK_EN() 								( (RCC->AHB2ENR) |= (SET_ONE_BITMASK << RCC_GPIOCEN) )

/* Clock Disable for GPIOx */
#define GPIOA_CLK_DI() 								( (RCC->AHB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_GPIOAEN) )
#define GPIOB_CLK_DI() 								( (RCC->AHB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_GPIOBEN) )
#define GPIOC_CLK_DI() 								( (RCC->AHB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_GPIOCEN) )

/* GPIO Register Reset Macro Definitions */
#define GPIOA_REG_RESET()								do { RCC->AHB2RSTR |= (SET_ONE_BITMASK << RCC_GPIOARST); RCC->AHB2RSTR &= ~(CLEAR_ONE_BITMASK << RCC_GPIOARST); } while (0)
#define GPIOB_REG_RESET()								do { RCC->AHB2RSTR |= (SET_ONE_BITMASK << RCC_GPIOBRST); RCC->AHB2RSTR &= ~(CLEAR_ONE_BITMASK << RCC_GPIOBRST); } while (0)
#define GPIOC_REG_RESET()								do { RCC->AHB2RSTR |= (SET_ONE_BITMASK << RCC_GPIOCRST); RCC->AHB2RSTR &= ~(CLEAR_ONE_BITMASK << RCC_GPIOCRST); } while (0)

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
	uint8_t GPIO_PinOSpeed;								/* !< Possible values from @GPIO_PIN_OUTPUT_SPEEDS > */
	uint8_t GPIO_PinPUPDControl;						/* !< Possible values from @GPIO_PIN_RESISTOR_CONFIGS > */
	uint8_t GPIO_PinOType;								/* !< Possible values from @GPIO_PIN_OUTPUT_TYPES > */
	uint8_t GPIO_PinAltFunc;							/* !< Possible values from @GPIO_PIN_ALTERNATE_FUNCTIONS > */
} GPIO_PinConfig_t;

/*
 * @GPIO_HANDLER
 * Initializer for a GPIO handler structure, that manages a GPIO instance's configuration and properties
 */
typedef struct GPIOHandleStructure
{
	__RW GPIOx_Reg_t *ptr_GPIOx;						/* This holds a pointer to the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;					/* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO possible pin numbers
 */
typedef enum GPIOPinNumbers
{
	PIN_ZERO, PIN_ONE, PIN_TWO, PIN_THREE, PIN_FOUR, PIN_FIVE, PIN_SIX, PIN_SEVEN,
	PIN_EIGHT, PIN_NINE, PIN_TEN, PIN_ELEVEN, PIN_TWELVE, PIN_THIRTEEN, PIN_FOURTEEN, PIN_FIFTEEN,
} GPIO_Pin_e;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
typedef enum GPIOModes
{
	INPUT, OUTPUT, ALTERNATE, ANALOG, INTERRUPT_FT, INTERRUPT_RT, INTERRUPT_RFT,
} GPIO_Mode_e;

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
 */
typedef enum GPIOOutputTypes
{
	PUSH_PULL, OPEN_DRAIN,
} GPIO_OType_e;

/*
 * @GPIO_PIN_OUTPUT_SPEEDS
 * GPIO pin possible output speeds
 */
typedef enum GPIOOutputSpeeds
{
	GPIO_LOW, GPIO_MEDIUM, GPIO_HIGH, GPIO_VERY_HIGH,
} GPIO_OSpeed_e;

/*
 * @GPIO_PIN_RESISTOR_CONFIGS
 * GPIO pin possible internal resistor configurations
 */
typedef enum GPIOPullUpPullDown
{
	GPIO_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN,
} GPIO_PUpDn_e;

/*
 * @GPIO_PIN_ALTERNATE_FUNCTIONS
 * GPIO pin possible alternate function configurations
 */
typedef enum GPIOAlternateFunctions
{
	ALT_ZERO, ALT_ONE, ALT_TWO, ALT_THREE, ALT_FOUR, ALT_FIVE, ALT_SIX, ALT_SEVEN,
	ALT_EIGHT, ALT_NINE, ALT_TEN, ALT_ELEVEN, ALT_TWELVE, ALT_THIRTEEN, ALT_FOURTEEN, ALT_FIFTEEN,
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

/* GPIO Data Read and Write */
uint8_t GPIO_ReadPin(__R GPIOx_Reg_t *const, uint8_t);
uint16_t GPIO_ReadPort(__R GPIOx_Reg_t *const);
void GPIO_WritePin(__W GPIOx_Reg_t *const, uint8_t, uint8_t);
void GPIO_WritePort(__W GPIOx_Reg_t *const, uint16_t);
void GPIO_TogglePin(__RW GPIOx_Reg_t *const, uint8_t);

/* GPIO Interrupt Configuration/Handling */
void GPIO_IRQHandlerFunc(uint8_t);


#endif /* INC_X_GPIO_H_ */
