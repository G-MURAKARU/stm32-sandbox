/*
 * x_spi.c
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_spi.h"


/*

 * @fn						- SPI_PeriphClkCtrl
 *
 * @brief					- This function enables or disables the peripheral clock for the given SPI port
 *
 * @param[ptr_SPIx]			- base address of the SPI peripheral
 * @param[en_di]			- ENABLE or DISABLE macros, ENABLE=1, DISABLE=0
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const ptr_SPIx, uint8_t en_di)
{
	if (ptr_SPIx == SPI1) (en_di) ? SPI1_CLK_EN() : SPI1_CLK_DI();
	else if (ptr_SPIx == SPI2) (en_di) ? SPI2_CLK_EN() : SPI2_CLK_DI();
	else if (ptr_SPIx == SPI3) (en_di) ? SPI3_CLK_EN() : SPI3_CLK_DI();
	else;
}


/*

 * @fn						- SPI_Init
 *
 * @brief					- this function initializes the given SPI port
 *
 * @param[ptr_SPIHandle]	- pointer to the base address of the SPI Handle structure
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_Init(__RH SPI_Handle_t *const ptr_SPIHandle)
{
	/* Enable the peripheral clock */
	SPI_PeriphClkCtrl((__R SPIx_Reg_t *const)ptr_SPIHandle->ptr_SPIx, ENABLE);

	/* Set up for CR1 register contents */
	volatile uint32_t temp_cr1_register = 0;

	/* Set up for CR1 register contents */
	volatile uint32_t temp_cr2_register = 0;

	/* 1. Configure the device mode */
	/* Set the MSTR bit in SPI CR1 if mode = master						!< Already cleared if slave > */
	if (ptr_SPIHandle->SPI_Config.SPI_DeviceMode) temp_cr1_register |= (SET_ONE_BITMASK << MSTR);

	/* 2. Configure the SPI Bus Mode */
	switch (ptr_SPIHandle->SPI_Config.SPI_BusCommsConfig)
	{
		case FULL_DUPLEX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << BIDIMODE);		!< Already cleared > */

			// RXONLY should be cleared (FD enabled)
			/* temp_register &= ~(CLEAR_ONE_BITMASK << RXONLY);			!< Already cleared > */

			break;
		case HALF_DUPLEX:
			// BIDI Mode should be set
			temp_cr1_register |= (SET_ONE_BITMASK << BIDIMODE);

			// BIDI Output should be enabled
			temp_cr1_register |= (SET_ONE_BITMASK << BIDIOE);

			break;
		case SIMPLEX_RX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << BIDIMODE);		!< Already cleared > */

			// RXONLY should be set
			temp_cr1_register |= (SET_ONE_BITMASK << RXONLY);

			break;

		default:
			break;
	}

	/* 3. Configure (pre-scale) the SPI bus baud rate */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockConfig << BAUD);

	/* 4. Configure the SPI data frame format */
	/* Found in SPI CR2 */
	temp_cr2_register |= (ptr_SPIHandle->SPI_Config.SPI_DataFrameFormat << DS);

	/* 5. Configure the SPI clock polarity */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPolarity << CPOL);

	/* 6. Configure the SPI clock phase */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPhase << CPHA);

	/* 7. SPI CRC configurations */
	if (ptr_SPIHandle->SPI_Config.SPI_CRCEnable)
	{
		/* Set CRC enabled bit */
		temp_cr1_register |= (SET_ONE_BITMASK << CRCEN);
		/* Select CRC length, set if 16-bit, leave cleared if 8-bit */
		if (ptr_SPIHandle->SPI_Config.SPI_CRCLength) temp_cr1_register |= (SET_ONE_BITMASK << CRCL);
	}

	/* 8. Configure SPI software slave select management */
	if (ptr_SPIHandle->SPI_Config.SPI_SSM) temp_cr1_register |= (SET_ONE_BITMASK << SSM);

	/* 9. Configure SPI internal slave selection */
	if (ptr_SPIHandle->SPI_Config.SPI_SSI) temp_cr1_register |= (SET_ONE_BITMASK << SSI);

	/* 10. Configure slave select output enable - (!)multi-master */
	if (ptr_SPIHandle->SPI_Config.SPI_SSOE) temp_cr2_register |= (SET_ONE_BITMASK << SSOE);

	/* 11. Configure SPI NSS pulse management */
	if (ptr_SPIHandle->SPI_Config.SPI_NSSP) temp_cr2_register |= (SET_ONE_BITMASK << NSSP);

	/* 12. Write configurations to SPI CR1 and CR2 (overwrite the whole register) */
	ptr_SPIHandle->ptr_SPIx->CR1 = temp_cr1_register;
	ptr_SPIHandle->ptr_SPIx->CR2 = temp_cr2_register;
}


/*

 * @fn						- SPI_DeInit
 *
 * @brief					- this function resets all registers of a given SPI port through the RCC reset register
 *
 * @param[ptr_SPIx]			- pointer to the base address of the SPI peripheral
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_DeInit(__RH SPIx_Reg_t *const ptr_SPIx)
{
	if (ptr_SPIx == SPI1) SPI1_REG_RESET();

	else if (ptr_SPIx == SPI2) SPI2_REG_RESET();

	else if (ptr_SPIx == SPI3) SPI3_REG_RESET();

	else;
}


/* SPI Data Read and Write */

/*

 * @fn						- SPI_DataSend
 *
 * @brief					- this function initiates the sending of data via the SPI bus
 *
 * @param[ptr_SPIx]			- pointer to the base address of the SPI peripheral
 * @param[ptr_tx_data]		- pointer to the base address of the buffer holding data to be transmitted
 * @param[data_length]		- length of the data to be transmitted, in bits
 * @param[data_tx_size]		- configured data transmission frame size, in bits (SPI DS)
 *
 * @return					- none
 *
 * @Note					- this function operates in a blocking manner

 */
void SPI_DataSend(__RW SPIx_Reg_t *const ptr_SPIx, const uint8_t *ptr_tx_data, int32_t data_length, uint8_t data_tx_size)
{
	// Check if there (still) exists data that needs to be transmitted
	while (data_length > 0)
	{
		// Wait until the TXE flag in SPI status register is set
		while  ( !(get_flag_status(ptr_SPIx->SR, 1, TXE)) );

		// Load transmit data into DR (tx FIFO) according to defined tx_size
		// Note: data is right-aligned on 8-bit or 16-bit boundaries
		if (data_tx_size < 8)
			ptr_SPIx->DR = *(ptr_tx_data);
		else
			ptr_SPIx->DR = *( (uint16_t *)ptr_tx_data );

		data_length -= data_tx_size;
		ptr_tx_data += (data_tx_size / 8) + 1; // pointer arithmetic
	}
}

void SPI_DataReceive(__RW SPIx_Reg_t *const, const uint8_t *, uint32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQNumberConfig(uint8_t, uint8_t);
void SPI_IRQPriorityConfig(uint8_t, uint16_t);
void SPI_IRQHandler(__R SPI_Handle_t *const);

void SPI_Control(__W SPIx_Reg_t *const ptr_SPIx, uint8_t en_di)
{
	(en_di) ? ( ptr_SPIx->CR1 |= (SET_ONE_BITMASK << SPE) ) : ( ptr_SPIx->CR1 &= ~(CLEAR_ONE_BITMASK << SPE) );
}
