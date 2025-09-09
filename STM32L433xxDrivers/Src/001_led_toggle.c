/*
 * 001_led_toggle.c
 *
 *  Created on: Apr 3, 2025
 *      Author: MURAKARU
 */


#include "x_gpio.h"

void delay(void)
{
	for (uint16_t i = 0; i < 65000; ++i);
}

int main(void)
{
	GPIO_Handle_t gpio_LED;

	gpio_LED.ptr_GPIOx = GPIOC;
	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOutSpeed = GPIO_HIGH_SPEED;
	gpio_LED.GPIO_PinConfig.GPIO_PinOutType = GPIO_PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPDN;

	GPIO_Init(&(gpio_LED));

	for (;;)
	{
		GPIO_TogglePin(GPIOC, (1U << GPIO_PIN_13));
		delay();
	}
}
