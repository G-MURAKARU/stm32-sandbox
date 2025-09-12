/*
 * x_gpio_defs.h
 *
 *  Created on: Sep 12, 2025
 *      Author: OMEN 16
 */

#ifndef INC_X_GPIO_DEFS_H_
#define INC_X_GPIO_DEFS_H_


#include "stm32l433xx.h"

/* Structure forward declarations */
typedef struct GPIO_PeripheralRegisters GPIOx_Reg_t;


/* GPIO pointers to base registers */

/* GPIOA */
#define GPIOA 											( (GPIOx_Reg_t *const)GPIOA_BASE_ADDR )

/* GPIOB */
#define GPIOB 											( (GPIOx_Reg_t *const)GPIOB_BASE_ADDR )

/* GPIOC */
#define GPIOC 											( (GPIOx_Reg_t *const)GPIOC_BASE_ADDR )


#endif /* INC_X_GPIO_DEFS_H_ */
