/*
 * x_gpio.h
 *
 *  Created on: Apr 1, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_GPIO_H_
#define INC_X_GPIO_H_

#include "stm32l433xx.h"

/* Structure forward declaration */
typedef struct GPIOHandleStructure GPIO_Handle_t;
typedef struct GPIOPinConfiguration GPIO_PinConfig_t;

/* Configuration structure for a GPIO pin */
typedef struct GPIOPinConfiguration
{
	uint8_t GPIO_PinNumber;								/* !< Possible values from @GPIO_PIN_NUMBERS > */
	uint8_t GPIO_PinMode;								/* !< Possible values from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinOSpeed;								/* !< Possible values from @GPIO_PIN_OUTPUT_SPEEDS > */
	uint8_t GPIO_PinPUPDControl;						/* !< Possible values from @GPIO_PIN_RESISTOR_CONFIGS > */
	uint8_t GPIO_PinOType;								/* !< Possible values from @GPIO_PIN_OUTPUT_TYPES > */
	uint8_t GPIO_PinAltFunc;
} GPIO_PinConfig_t;

/* Handle structure for a GPIO pin */
typedef struct GPIOHandleStructure
{
	__RW GPIOx_Reg_t *ptr_GPIOx;					/* This holds the base address of the GPIO port to which the pin belongs */
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
	LOW, MEDIUM, HIGH, VERY_HIGH,
} GPIO_OSpeed_e;

/*
 * @GPIO_PIN_RESISTOR_CONFIGS
 * GPIO pin possible internal resistor configurations
 */
typedef enum GPIOPullUpPullDown
{
	NONE, PULL_UP, PULL_DOWN,
} GPIO_PUpDn_e;

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
void GPIO_PeriphClkCtrl(__R GPIOx_Reg_t *const, uint8_t);

/* GPIO Initialization */
void GPIO_Init(__W GPIO_Handle_t *const);
void GPIO_DeInit(__R GPIOx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* GPIO Data Read and Write */
uint8_t GPIO_ReadPin(volatile __R GPIOx_Reg_t *const, uint8_t);
uint16_t GPIO_ReadPort(volatile __R GPIOx_Reg_t *const);
void GPIO_WritePin(__W GPIOx_Reg_t *const, uint8_t, uint8_t);
void GPIO_WritePort(__W GPIOx_Reg_t *const, uint16_t);
void GPIO_TogglePin(__RW GPIOx_Reg_t *const, uint8_t);

/* GPIO Interrupt Configuration/Handling */
void GPIO_IRQNumberConfig(uint8_t, uint8_t);
void GPIO_IRQPriorityConfig(uint8_t, uint16_t);
void GPIO_IRQHandler(uint8_t);





#endif /* INC_X_GPIO_H_ */
