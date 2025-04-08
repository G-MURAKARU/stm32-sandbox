/*
 * stm32l433xx.c
 *
 *  Created on: Apr 5, 2025
 *      Author: OMEN 16
 */

#include "stm32l433xx.h"
#include "x_gpio.h"


uint8_t SYSCFG_EXTICR_helper_func(__R GPIOx_Reg_t *const port)
{
	if (port == GPIOA) return (uint8_t) 0;

	else if (port == GPIOB) return (uint8_t) 1;

	else if (port == GPIOC) return (uint8_t) 2;

	else return (-1);
}
