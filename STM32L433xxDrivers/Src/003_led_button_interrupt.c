/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2025
 *      Author: MURAKARU
 */

#include "x_gpio.h"
#include "x_gpio_irq.h"
#include <stdlib.h>
#include <string.h>

void delay(void);
void button_handler(void);

static volatile bool button_pressed = false;


int main(void)
{
	// 1. Create a GPIO Handle structure for the LED and for the push-button
	GPIO_Handle_t gpio_LED, gpio_BUTTON;

	// Ensure to clear the above structures' memory locations
	memset(&(gpio_LED), 0, sizeof(GPIO_Handle_t));
	memset(&(gpio_BUTTON), 0, sizeof(GPIO_Handle_t));

	// 2. Output configurations : On-board LED
	gpio_LED.ptr_GPIOx = GPIOC;

	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOutSpeed = GPIO_HIGH_SPEED;
	gpio_LED.GPIO_PinConfig.GPIO_PinOutType = GPIO_PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPDN;

	// 3. Input configurations : External push button
	gpio_BUTTON.ptr_GPIOx = GPIOB;

	gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_INTERRUPT_FT;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PULL_UP;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinIRQPriority = 15;


	// 4. GPIO Configs. Initialization
	GPIO_Init(&(gpio_LED));
	GPIO_Init(&(gpio_BUTTON));

	// 5. IRQ Configurations
	GPIO_RegisterCallback(gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber, button_handler);
	EnableIRQ(gpio_BUTTON.ptr_GPIOx, gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber, gpio_BUTTON.GPIO_PinConfig.GPIO_PinIRQPriority);

	for (;;)
	{
		if (button_pressed)
			{
				GPIO_TogglePin(GPIOC, (1U << GPIO_PIN_13));
				button_pressed = false;
			}
		// do other work();
	}
}

void delay(void)
{
	for (uint32_t i = 0; i < 200000; ++i);
}

void button_handler(void)
{
	delay();
	button_pressed = true;
	delay();
}
