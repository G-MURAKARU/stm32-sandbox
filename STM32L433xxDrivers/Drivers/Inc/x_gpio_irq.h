/*
 * x_gpio_irq.h
 *
 *  Created on: Sep 13, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_GPIO_IRQ_H_
#define INC_X_GPIO_IRQ_H_

#include "x_nvic.h"

void GPIO_RegisterCallback(uint8_t, void (*)(void));
//void gpio_poll(void);


#endif /* INC_X_GPIO_IRQ_H_ */
