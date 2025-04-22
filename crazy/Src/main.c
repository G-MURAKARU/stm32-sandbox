
int main(void)
{
	// RCC
	*((uint32_t *)0x4002104CUL) |= (0x2 | 0x4);
	// GPIOC
	*((uint32_t *)0x48000800UL) &= ~0xB000000;
	*((uint32_t *)0x48000800UL) |= 0x4000000;
	*((uint32_t *)0x48000804UL) &= ~0x2000;
	*((uint32_t *)0x48000808UL) &= ~0xB000000;
	*((uint32_t *)0x48000808UL) |= 0x8000000;
	*((uint32_t *)0x4800080CUL) &= ~0xB000000;
	// GPIOB
	*((uint32_t *)0x48000400UL) &= ~0xB000000;
	*((uint32_t *)0x4800040CUL) &= ~0xB000000;
	*((uint32_t *)0x4800040CUL) |= ~0x1000000;
	// RCC
	*((uint32_t *)0x40021060UL) |= 0x1;
	// SYSCFG
	*((uint32_t *)0x40010014UL) |= 0x1;
	// EXTI
	*((uint32_t *)0x40010400UL) |= 0x1000;
	*((uint32_t *)0x4001040CUL) |= 0x1000;
	// NVIC
	*((uint32_t *)0xE000E104UL) |= 0x100;
	*((uint32_t *)0xE000E428UL) |= 0xD0;

	for(;;);
}

void EXTI15_10_IRQHandler(void)
{
	for (uint32_t i = 0; i < 200000; ++i);
	// EXTI
	if ( *((uint32_t *)0x40010400UL) & 0x1000 ) ( *((uint32_t *)0x40010400UL) |= 0x1000 );
	// GPIOC
	*((uint32_t *)0x48000814UL) ^= 0x2000;
}
