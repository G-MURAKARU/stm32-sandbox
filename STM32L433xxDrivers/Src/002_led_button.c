/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2025
 *      Author: OMEN 16
 */

#include "stm32l433xx.h"
#include "x_gpio.h"

void delay(void)
{
	for (uint16_t i = 0; i < 30000; ++i);
}

int main(void)
{
	// 1. Create a GPIO Handle structure for the LED and for the push-button
	GPIO_Handle_t gpio_LED, gpio_BUTTON;

	// 2. Output configurations : On-board LED
	gpio_LED.ptr_GPIOx = GPIOC;

	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = PIN_THIRTEEN;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOSpeed = HIGH;
	gpio_LED.GPIO_PinConfig.GPIO_PinOType = PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = NONE;

	// 3. Input configurations : External push button
	gpio_BUTTON.ptr_GPIOx = GPIOB;

	gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = PIN_TWELVE;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = INPUT;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinPUPDControl = PULL_UP;

	// 4. GPIO Clock Initialization
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_LED.ptr_GPIOx, ENABLE);
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_BUTTON.ptr_GPIOx, ENABLE);

	// 5. GPIO Configs. Initialization
	GPIO_Init(&(gpio_LED));
	GPIO_Init(&(gpio_BUTTON));

	for (;;)
	{
		if ( !(GPIO_ReadPin((volatile __R GPIOx_Reg_t *const)gpio_BUTTON.ptr_GPIOx, PIN_TWELVE)) )
		{
			delay();
			GPIO_TogglePin(gpio_LED.ptr_GPIOx, PIN_THIRTEEN);
		}
	}

	return (0);
}
