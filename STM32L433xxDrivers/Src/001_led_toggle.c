/*
 * 001_led_toggle.c
 *
 *  Created on: Apr 3, 2025
 *      Author: OMEN 16
 */

#include "stm32l433xx.h"
#include "x_gpio.h"

void delay(void)
{
	for (uint16_t i = 0; i < 65000; ++i);
}

int main(void)
{
	GPIO_Handle_t gpio_LED;

	gpio_LED.ptr_GPIOx = GPIOC;
	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = PIN_THIRTEEN;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOSpeed = HIGH;
	gpio_LED.GPIO_PinConfig.GPIO_PinOType = PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = NONE;

	GPIO_PeriphClkCtrl(GPIOC, ENABLE);
	GPIO_Init(&(gpio_LED));

	for (;;)
	{
		GPIO_TogglePin(GPIOC, PIN_THIRTEEN);
		delay();
	}

	return (0);
}
