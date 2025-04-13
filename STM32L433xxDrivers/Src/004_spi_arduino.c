/*
 * 004_spi_arduino.c
 *
 *  Created on: Apr 12, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_gpio.h"
#include "x_spi.h"
#include <string.h>


/*
 * SPI 2 pinout information:
 * Port: B
 * MISO: P14
 * MOSI: P15
 * SCK: P13
 * NSS: P12
 */

void spi2_gpio_inits(__RW GPIO_Handle_t *const);
void spi2_inits(__RW SPI_Handle_t *const);

/* 1. Create a GPIO Handle structure for the SPI interface */
GPIO_Handle_t gpio_SPI, gpio_BUTTON, gpio_LED;
SPI_Handle_t spi_handle;
char user_data[] = "Hello, World!";

int main(void)
{
	memset(&(gpio_SPI), 0, sizeof(GPIO_Handle_t));
	memset(&(gpio_BUTTON), 0, sizeof(GPIO_Handle_t));
	memset(&(gpio_LED), 0, sizeof(gpio_LED));
	memset(&(spi_handle), 0, sizeof(SPI_Handle_t));

	spi2_gpio_inits(&(gpio_SPI));
	spi2_inits(&(spi_handle));

	// Input configurations : External push button
	gpio_BUTTON.ptr_GPIOx = GPIOB;

	gpio_BUTTON.GPIO_PinConfig.GPIO_PinNumber = PIN_ELEVEN;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinMode = INTERRUPT_FT;
	gpio_BUTTON.GPIO_PinConfig.GPIO_PinPUPDControl = PULL_UP;

	gpio_LED.ptr_GPIOx = GPIOC;
	gpio_LED.GPIO_PinConfig.GPIO_PinNumber = PIN_THIRTEEN;
	gpio_LED.GPIO_PinConfig.GPIO_PinMode = OUTPUT;
	gpio_LED.GPIO_PinConfig.GPIO_PinOSpeed = HIGH;
	gpio_LED.GPIO_PinConfig.GPIO_PinOType = PUSH_PULL;
	gpio_LED.GPIO_PinConfig.GPIO_PinPUPDControl = NONE;

	GPIO_Init(&(gpio_LED));
	GPIO_Init(&(gpio_BUTTON));

//	SPI_Control(spi_handle.ptr_SPIx, ENABLE);
//	uint32_t data_length = strlen(user_data) * 8; // return data length in bits
//	SPI_DataSend(spi_handle.ptr_SPIx, (uint8_t *)&user_data, data_length, spi_handle.SPI_Config.SPI_DataFrameFormat);
//	SPI_Control(spi_handle.ptr_SPIx, DISABLE);

	// 6. IRQ Configurations
	GPIO_IRQNumberConfig(EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10, 15);

	for (;;);
}

void spi2_gpio_inits(__RW GPIO_Handle_t *const ptr_gpio_handle)
{
	ptr_gpio_handle->ptr_GPIOx = GPIOB;

	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinMode = ALTERNATE;
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinOType = PUSH_PULL;
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinOSpeed = LOW;
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinPUPDControl = NONE;
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinAltFunc = ALT_FIVE;

	/* MISO */
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinNumber = PIN_FOURTEEN;
	GPIO_Init((__R GPIO_Handle_t *const)ptr_gpio_handle);

	/* MOSI */
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinNumber = PIN_FIFTEEN;
	GPIO_Init((__R GPIO_Handle_t *const)ptr_gpio_handle);

	/* SCK */
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinNumber = PIN_THIRTEEN;
	GPIO_Init((__R GPIO_Handle_t *const)ptr_gpio_handle);

	/* NSS */
	ptr_gpio_handle->GPIO_PinConfig.GPIO_PinNumber = PIN_TWELVE;
	GPIO_Init((__R GPIO_Handle_t *const)ptr_gpio_handle);
}

void spi2_inits(__RW SPI_Handle_t *const ptr_spi_handle)
{
	ptr_spi_handle->ptr_SPIx = SPI2;

	ptr_spi_handle->SPI_Config.SPI_DeviceMode = MASTER;
	ptr_spi_handle->SPI_Config.SPI_BusCommsConfig = FULL_DUPLEX;
	ptr_spi_handle->SPI_Config.SPI_ClockConfig = PRE_2;
	ptr_spi_handle->SPI_Config.SPI_DataFrameFormat = FR_8BIT;
	ptr_spi_handle->SPI_Config.SPI_ClockPolarity = CK_LOW;
	ptr_spi_handle->SPI_Config.SPI_ClockPhase = CP_LOW;
	ptr_spi_handle->SPI_Config.SPI_SSM = SSM_DISABLE;
	ptr_spi_handle->SPI_Config.SPI_SSOE = SSOE_ENABLE;
	ptr_spi_handle->SPI_Config.SPI_NSSP = NSSP_ENABLE;

	SPI_Init((__R SPI_Handle_t *const)ptr_spi_handle);
}

void EXTI15_10_IRQHandler(void)
{
	for (uint32_t i = 0; i < 400000; ++i);

	GPIO_IRQHandler(PIN_ELEVEN);
	GPIO_TogglePin(GPIOC, PIN_THIRTEEN);

	SPI_Control(spi_handle.ptr_SPIx, ENABLE);

	int32_t data_length = strlen(user_data);

	SPI_DataSend(spi_handle.ptr_SPIx, (const uint8_t *)data_length, data_length, spi_handle.SPI_Config.SPI_DataFrameFormat);

	data_length *= 8; // return data length in bits

	SPI_DataSend(spi_handle.ptr_SPIx, (const uint8_t *)user_data, data_length, spi_handle.SPI_Config.SPI_DataFrameFormat);

	while( get_flag_status(spi_handle.ptr_SPIx->SR, 1, BSY) && get_flag_status(spi_handle.ptr_SPIx->SR, 2, FTLVL) );

	SPI_Control(spi_handle.ptr_SPIx, DISABLE);
}
