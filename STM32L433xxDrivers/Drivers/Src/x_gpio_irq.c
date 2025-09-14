/*
 * x_gpio_irq.c
 *
 *  Created on: Sep 13, 2025
 *      Author: MURAKARU
 */

#include "x_gpio_irq.h"

static void (*gpio_callbacks[16])(void) = { NULL };
//static volatile uint8_t gpio_irq_flags[16] = {0}; // <-- flags per EXTI line

void GPIO_RegisterCallback(uint8_t line, void (*callback)(void))
{
    if (line < 16) {
        gpio_callbacks[line] = callback;
    }
}

static void gpio_irq_handler(uint8_t line)
{
//	if (((EXTI->PR1 >> line) & 1U) && (line < 16)) gpio_irq_flags[line] = 1;
	if (((EXTI->PR1 >> line) & 1U) && gpio_callbacks[line]) gpio_callbacks[line]();

	EXTI->PR1 |= (1U << line);
	(void)EXTI->PR1;
}

//void gpio_poll(void)
//{
//    for (uint8_t line = 0; line < 16; ++line)
//    {
//        if (gpio_irq_flags[line])
//        {
//            if (gpio_callbacks[line])
//            {
//                gpio_callbacks[line](); // call user callback here
//            }
//            gpio_irq_flags[line] = 0; // clear flag
//        }
//    }
//}

// Individual ISRs
void EXTI0_IRQHandler(void)  { gpio_irq_handler(0); }
void EXTI1_IRQHandler(void)  { gpio_irq_handler(1); }
void EXTI2_IRQHandler(void)  { gpio_irq_handler(2); }
void EXTI3_IRQHandler(void)  { gpio_irq_handler(3); }
void EXTI4_IRQHandler(void)  { gpio_irq_handler(4); }

void EXTI9_5_IRQHandler(void)
{
//    for (uint8_t line = 5; line <= 9; ++line) {
//        gpio_irq_handler(line);
//    }

    if ((EXTI->PR1 >> 5) & 1U) gpio_irq_handler(5);
    if ((EXTI->PR1 >> 6) & 1U) gpio_irq_handler(6);
    if ((EXTI->PR1 >> 7) & 1U) gpio_irq_handler(7);
    if ((EXTI->PR1 >> 8) & 1U) gpio_irq_handler(8);
    if ((EXTI->PR1 >> 9) & 1U) gpio_irq_handler(9);
}

void EXTI15_10_IRQHandler(void)
{
//    for (uint8_t line = 10; line <= 15; ++line) {
//        gpio_irq_handler(line);
//    }

    if ((EXTI->PR1 >> 10) & 1U) gpio_irq_handler(10);
	if ((EXTI->PR1 >> 11) & 1U) gpio_irq_handler(11);
	if ((EXTI->PR1 >> 12) & 1U) gpio_irq_handler(12);
	if ((EXTI->PR1 >> 13) & 1U) gpio_irq_handler(13);
	if ((EXTI->PR1 >> 14) & 1U) gpio_irq_handler(14);
	if ((EXTI->PR1 >> 15) & 1U) gpio_irq_handler(15);
}

