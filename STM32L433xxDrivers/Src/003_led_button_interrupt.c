/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2025
 *      Author: MURAKARU
 */

#include "stm32l433xx.h"
#include "x_gpio.h"
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

	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = PIN_THIRTEEN;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_HIGH;
	gpio_LED.GPIO_PinConfig.GPIO_PinOType = PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NONE;

	// 3. Input configurations : External push button
	gpio_BUTTON.ptr_GPIOx = GPIOB;

	gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = PIN_TWELVE;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = INTERRUPT_FT;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PULL_UP;

	// 4. GPIO Clock Initialization
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_LED.ptr_GPIOx, ENABLE);
	GPIO_PeriphClkCtrl((__R GPIOx_Reg_t *const)gpio_BUTTON.ptr_GPIOx, ENABLE);

	// 5. GPIO Configs. Initialization
	GPIO_Init(&(gpio_LED));
	GPIO_Init(&(gpio_BUTTON));

	// 6. IRQ Configurations
	NVIC_IRQNumberConfig(IRQ_EXTI15_10, ENABLE);
	NVIC_IRQPriorityConfig(IRQ_EXTI15_10, 15);

	for (;;);
}

void delay(void)
{
	for (uint32_t i = 0; i < 200000; ++i);
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandler(PIN_TWELVE);
	GPIO_TogglePin(GPIOC, PIN_THIRTEEN);
}



