/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2025
 *      Author: MURAKARU
 */

#include "x_gpio.h"
#include "x_nvic.h"
#include <stdlib.h>
#include <string.h>

void delay(void);


int main(void)
{
	// 1. Create a GPIO Handle structure for the LED and for the push-button
	GPIO_Handle_t gpio_LED, gpio_BUTTON;

	// Ensure to clear the above structures' memory locations
	memset(&(gpio_LED), 0, sizeof(gpio_LED));
	memset(&(gpio_BUTTON), 0, sizeof(gpio_BUTTON));

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

	// 4. GPIO Clock Initialization
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_LED.ptr_GPIOx, ENABLE);
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_BUTTON.ptr_GPIOx, ENABLE);

	// 5. GPIO Configs. Initialization
	GPIO_Init(&(gpio_LED));
	GPIO_Init(&(gpio_BUTTON));

	// 6. IRQ Configurations
	EnableIRQ(gpio_BUTTON.ptr_GPIOx, gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber, 15);

	for (;;);
}

void delay(void)
{
	for (uint32_t i = 0; i < 200000; ++i);
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandlerFunc(GPIO_PIN_12);
	GPIO_TogglePin(GPIOC, (1U << GPIO_PIN_13));
}
