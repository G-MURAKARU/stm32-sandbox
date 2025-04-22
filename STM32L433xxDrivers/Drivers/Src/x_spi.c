/*
 * x_spi.c
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */


#include "stm32l433xx.h"
#include "x_spi.h"


static void SPI_TXE_Interrupt_HelperFunc(__RW SPI_Handle_t *const, uint8_t);
static void SPI_RXNE_Interrupt_HelperFunc(__RW SPI_Handle_t *const, uint8_t);
static void SPI_OVR_Interrupt_HelperFunc(__R SPI_Handle_t *const);
static void SPI_ClearOVRFlag(__R SPIx_Reg_t *const);

/*

 * @fn						- SPI_PeriphClkCtrl
 *
 * @brief					- This function enables or disables the peripheral clock for the given SPI port, through the RCC register
 *
 * @param[ptr_SPIx]			- base address of the SPI peripheral
 * @param[en_di]			- action to take: ENABLE(1) or DISABLE(0)
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const ptr_SPIx, bool en_di)
{
	if (ptr_SPIx == SPI1) (en_di) ? SPI1_CLK_EN() : SPI1_CLK_DI();

	else if (ptr_SPIx == SPI2) (en_di) ? SPI2_CLK_EN() : SPI2_CLK_DI();

	else if (ptr_SPIx == SPI3) (en_di) ? SPI3_CLK_EN() : SPI3_CLK_DI();

	else {};
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
	if (ptr_SPIHandle->SPI_Config.SPI_DeviceMode) temp_cr1_register |= (SET_ONE_BITMASK << SPI_MSTR);

	/* 2. Configure the SPI Bus Mode */
	switch (ptr_SPIHandle->SPI_Config.SPI_BusCommsConfig)
	{
		case SPI_FULL_DUPLEX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << SPI_BIDIMODE);		!< Already cleared > */

			// RXONLY should be cleared (Full Duplex enabled)
			/* temp_register &= ~(CLEAR_ONE_BITMASK << SPI_RXONLY);			!< Already cleared > */

			break;
		case SPI_HALF_DUPLEX:
			// BIDI Mode should be set
			temp_cr1_register |= (SET_ONE_BITMASK << SPI_BIDIMODE);

			// BIDI Output should be enabled
			temp_cr1_register |= (SET_ONE_BITMASK << SPI_BIDIOE);

			break;
		case SPI_SIMPLEX_RX:
			// BIDI Mode should be cleared
			/* temp_register &= ~(CLEAR_ONE_BITMASK << BIDIMODE);		!< Already cleared > */

			// RXONLY should be set
			temp_cr1_register |= (SET_ONE_BITMASK << SPI_RXONLY);

			break;

		default:
			break;
	}

	/* 3. Configure (pre-scale) the SPI bus baud rate */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockConfig << SPI_BAUD);

	/* 4. Configure the SPI data frame format */
	/* Found in SPI CR2 */
	temp_cr2_register |= (ptr_SPIHandle->SPI_Config.SPI_DataFrameFormat << SPI_DATASIZE);

	/* 5. Configure the SPI clock polarity */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPolarity << CPOL);

	/* 6. Configure the SPI clock phase */
	temp_cr1_register |= (ptr_SPIHandle->SPI_Config.SPI_ClockPhase << CPHA);

	/* 7. SPI CRC configurations */
	if (ptr_SPIHandle->SPI_Config.SPI_CRCEnable)
	{
		/* Set CRC enabled bit */
		temp_cr1_register |= (SET_ONE_BITMASK << SPI_CRCEN);
		/* Select CRC length, set if 16-bit, leave cleared if 8-bit */
		if (ptr_SPIHandle->SPI_Config.SPI_CRCLength) temp_cr1_register |= (SET_ONE_BITMASK << SPI_CRCL);
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

	else {};
}

/* SPI Data Read and Write, assuming data being sent can only be 8-bit or 16-bit */

/*

 * @fn						- SPI_DataSend
 *
 * @brief					- this function initiates the sending of data via the SPI bus
 *
 * @param[ptr_SPIx]			- pointer to the base address of the SPI peripheral
 * @param[ptr_tx_data]		- pointer to the base address of the buffer holding data to be transmitted
 * @param[data_length]		- length of the data to be transmitted, in bytes
 * @param[data_comm_size]		- configured data transmission frame size, in bits (SPI DS)
 *
 * @return					- none
 *
 * @Note					- this function operates in a blocking manner

 */
void SPI_DataSend(__RW SPIx_Reg_t *const ptr_SPIx, __R uint8_t *volatile ptr_tx_data, int32_t data_length, uint8_t data_comm_size)
{
	// Decrement cursor variable to track the moving pointer in TX buffer, defaults to a single byte increment
	uint8_t decrement = 1;

	// Check if there (still) exists data that needs to be transmitted
	while (data_length > 0)
	{
		// Wait until the TXE (transmit empty) flag in SPI status register is set
		while  ( !(MCU_GetFlagStatus(ptr_SPIx->SR, 1, SPI_TXE)) );

		/* 1. Read data from transmit buffer according to defined tx_size
		 * Note: data is right-aligned on 8-bit or 16-bit boundaries
		 */

		/* 2. Write data to SPI data register == TX FIFO */
		if (data_comm_size == 16)
		{
			// Decrement by 2 bytes if data comms size is configured to 16 bits
			decrement = 2;

			// 2.1.1 If 16-bit data, cast the TX buffer pointer to point to 16-bit data, then write data into SPI DR (TX FIFO)
			ptr_SPIx->DR = *(__R uint16_t *volatile)ptr_tx_data;
			// 2.2.1 Cast to point to 16-bit data for correct pointer arithmetic
			// 2.3.1 Move TX buffer pointer to next location to be written, depending on data comm length
			(__R uint16_t *volatile)ptr_tx_data++;
		}
		else if (data_comm_size == 8)
		{
			// 2.1.2 8-bit data, direct write to SPI DR (RX FIFO) and direct pointer increment
			ptr_SPIx->DR = *(ptr_tx_data);
			// 2.2.2
			// 2.3.2
			ptr_tx_data++;
		}

		// 3. Decrement the data length to find the remaining bytes to transmit
		data_length -= decrement;
	}
}

/*

 * @fn						- SPI_DataReceive
 *
 * @brief					- this function initiates the reception of data via the SPI bus
 *
 * @param[ptr_SPIx]			- pointer to the base address of the SPI peripheral
 * @param[ptr_rx_data]		- pointer to the base address of the buffer holding received data
 * @param[data_length]		- length of the data to be received, in bytes
 * @param[data_comm_size]	- configured data transmission frame size, in bits (SPI DS)
 *
 * @return					- none
 *
 * @Note					- this function operates in a blocking manner

 */
void SPI_DataReceive(__RW SPIx_Reg_t *const ptr_SPIx, __RW uint8_t *volatile ptr_rx_data, int32_t data_length, uint8_t data_comm_size)
{
	// Decrement cursor variable to track the moving pointer in RX buffer, defaults to a single byte increment
	uint8_t decrement = 1;

	// Check if there (still) exists data that needs to be received
	while (data_length > 0)
	{
		// Wait until the RXNE (receive not empty) flag in SPI status register is set
		while  ( !(MCU_GetFlagStatus(ptr_SPIx->SR, 1, SPI_RXNE)) );

		/* 1. Read transmitted data from DR (RX FIFO) according to defined rx_size
		 * Note: data is right-aligned on 8-bit or 16-bit boundaries
		 */

		/* 2. Write data to SPI data register == RX FIFO */
		if (data_comm_size == 16)
		{
			// Decrement by 2 bytes if data comms size is configured to 16 bits
			decrement = 2;

			// 2.1.1 If 16-bit data, cast the RX buffer pointer to point to 16-bit data, then read from SPI DR (RX FIFO)
			// and write to RX buffer
			*(__RW uint16_t *volatile)ptr_rx_data = ptr_SPIx->DR;
			// 2.2.1 cast to point to 16-bit data for correct pointer arithmetic
			// 2.3.1 Move RX buffer pointer to next location to be written, depending on data comm length
			(__RW uint16_t *volatile)ptr_rx_data++;
		}
		else if (data_comm_size == 8)
		{
			// 2.1.2 8-bit data, direct write to SPI DR (RX FIFO) and direct pointer increment
			*(ptr_rx_data) = ptr_SPIx->DR;
			// 2.2.2
			// 2.3.2
			ptr_rx_data++;
		}

		// 3. Decrement the data length to find the remaining bytes to receive
		data_length -= decrement;
	}
}

// Non-Blocking APIs

/*

 * @fn						- SPI_DataSend_IT
 *
 * @brief					- this function initiates the sending of data via the SPI bus
 *
 * @param[ptr_SPIHandle]	- pointer to the application's SPI Handle structure instance
 * @param[ptr_tx_data]		- pointer to the base address of the buffer holding data to be transmitted
 * @param[data_length]		- length of the data to be transmitted, in bytes
 *
 * @return					- SPI transmission state (0 = ready, 1 = busy)
 *
 * @Note					- this function is interrupt-driven - operates in a non-blocking manner

 */
bool SPI_DataSend_IT(__RW SPI_Handle_t *const ptr_SPIHandle, __R uint8_t *const ptr_tx_data, int32_t data_length)
{
	/* Check that the device SPI peripheral is not already in transmission */
	bool state = ptr_SPIHandle->TX_State;

	if ( !(state) )
	{
		/* 1. Save the TX buffer address and data length info in some global variables - SPI Handle structure */
		ptr_SPIHandle->ptr_TX_Data = (__RW uint8_t *)ptr_tx_data;
		ptr_SPIHandle->TX_DataLength = data_length;

		/*
		 * 2. Mark the SPI state as BUSY in transmission so that
		 * no other code can take over the same SPI peripheral until transmission is over
		 */
		ptr_SPIHandle->TX_State = SET;

		/* 3. Enable the TXEIE control bit in SPI CR2 to get the interrupt whenever TXE flag is set in SR */
		ptr_SPIHandle->ptr_SPIx->CR2 |= (SET_ONE_BITMASK << SPI_TXEIE);
	}

	/* 4. Data transmission will be handled by the ISR code - IRQ Handler */
	return state;
}

/*

 * @fn						- SPI_DataReceive_IT
 *
 * @brief					- this function initiates the receiving of data via the SPI bus
 *
 * @param[ptr_SPIHandle]	- pointer to the application's SPI Handle structure instance
 * @param[ptr_rx_data]		- pointer to the base address of the buffer that will hold received data
 * @param[data_length]		- length of the data to be transmitted, in bytes
 *
 * @return					- SPI reception state (0 = ready, 1 = busy)
 *
 * @Note					- this function is interrupt-driven - operates in a non-blocking manner

 */
bool SPI_DataReceive_IT(__RW SPI_Handle_t *const ptr_SPIHandle, __R uint8_t *const ptr_rx_data, int32_t data_length)
{
	/* Check that the device SPI peripheral is not already in reception */
	bool state = ptr_SPIHandle->RX_State;

	if ( !(state) )
	{
		/* 1. Save the RX buffer address and data length info in some global variables - SPI Handle structure */
		ptr_SPIHandle->ptr_RX_Data = (__RW uint8_t *)ptr_rx_data;
		ptr_SPIHandle->RX_DataLength = data_length;

		/*
		 * 2. Mark the SPI state as BUSY in reception so that
		 * no other code can take over the same SPI peripheral until reception is over
		 */
		ptr_SPIHandle->RX_State = SET;

		/* 3. Enable the RXNEIE control bit in SPI CR2 to get the interrupt whenever RXNE flag is set in SR */
		ptr_SPIHandle->ptr_SPIx->CR2 |= (SET_ONE_BITMASK << SPI_RXNEIE);
	}

	/* 4. Data reception will be handled by the ISR code - IRQ Handler */
	return state;
}

/*

 * @fn						- SPI_IRQHandler
 *
 * @brief					- this function handles SPI Interrupt events, handing off to helper functions for each event
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 *
 * @return					- none
 *
 * @Note					- called within an ISR, non-blocking

 */
void SPI_IRQHandlerFunc(__R SPI_Handle_t *const ptr_SPIHandle)
{
	/* 1. Check the source of the interrupt event: TXE, RXNE, etc */

	/* Check for transmit event */
	bool tempvar_1, tempvar_2;
	tempvar_1 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->SR, 1, SPI_TXE);
	tempvar_2 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->CR2, 1, SPI_TXEIE);

	/* If transmit event, call TX helper function to initiate transmission */
	if ( tempvar_1 && tempvar_2 )
	{
		SPI_TXE_Interrupt_HelperFunc((__RW SPI_Handle_t *const)ptr_SPIHandle, ptr_SPIHandle->SPI_Config.SPI_DataFrameFormat);
	}

	/* Check for receive event */
	tempvar_1 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->SR, 1, SPI_RXNE);
	tempvar_2 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->CR2, 1, SPI_RXNEIE);

	/* If receive event, call RX helper function to initiate reception */
	if ( tempvar_1 && tempvar_2 )
	{
		SPI_RXNE_Interrupt_HelperFunc((__RW SPI_Handle_t *const)ptr_SPIHandle, ptr_SPIHandle->SPI_Config.SPI_DataFrameFormat);
	}

	/* Check for overrun event */
	tempvar_1 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->SR, 1, SPI_OVR);
	tempvar_2 = MCU_GetFlagStatus(ptr_SPIHandle->ptr_SPIx->CR2, 1, SPI_ERRIE);

	/* If overrun event, call OVR helper function to handle overrun condition */
	if ( tempvar_1 && tempvar_2 )
	{
		SPI_OVR_Interrupt_HelperFunc(ptr_SPIHandle);
	}

	/* TODO: Check for CRC event */
}

/*

 * @fn						- SPI_TXE_Interrupt_HelperFunc
 *
 * @brief					- this function handles a data transmission interrupt event
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 * @param[data_comm_size]	- configured data transmission frame size, in bits (SPI DS)
 *
 * @return					- none
 *
 * @Note					- called within an ISR, non-blocking

 */
static void SPI_TXE_Interrupt_HelperFunc(__RW SPI_Handle_t *const ptr_SPIHandle, uint8_t data_comm_size)
{
	// Decrement cursor variable to track the moving pointer in TX buffer, defaults to a single byte increment
	uint8_t decrement = 1;

	/* 1. Read data from transmit buffer according to defined tx_size
	 * Note: data is right-aligned on 8-bit or 16-bit boundaries
	 */

	/* 2. Write data to SPI data register == TX FIFO */
	if (data_comm_size == 16)
	{
		// Decrement by 2 bytes if data comms size is configured to 16 bits
		decrement = 2;

		// 2.1.1 If 16-bit data, cast the TX buffer pointer to point to 16-bit data, then write data into SPI DR (TX FIFO)
		ptr_SPIHandle->ptr_SPIx->DR = *(__R uint16_t *)ptr_SPIHandle->ptr_TX_Data;
		// 2.2.1 Cast to point to 16-bit data for correct pointer arithmetic
		// 2.3.1 Move TX buffer pointer to next location to be written, depending on data comm length
		(__R uint16_t *)ptr_SPIHandle->ptr_TX_Data++;
	}
	else if (data_comm_size == 8)
	{
		// 2.1.2 8-bit data, direct write to SPI DR (RX FIFO) and direct pointer increment
		ptr_SPIHandle->ptr_SPIx->DR = *(ptr_SPIHandle->ptr_TX_Data);
		// 2.2.2
		// 2.3.2
		ptr_SPIHandle->ptr_TX_Data++;
	}

	// 3. Decrement the data length to find the remaining bytes to transmit
	ptr_SPIHandle->TX_DataLength -= decrement;

	// 4. Check if all data has been received and read, to allow closing of SPI communication
	if (ptr_SPIHandle->TX_DataLength <= 0)
	{
		/* Close data transmission */
		SPI_CloseTransmission(ptr_SPIHandle);

		/* Hand over back to the application */
		SPI_ApplicationEventCallback(ptr_SPIHandle, SPI_TX_DONE);
	}
}

/*

 * @fn						- SPI_RXNE_Interrupt_HelperFunc
 *
 * @brief					- this function handles a data reception interrupt event
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 * @param[data_comm_size]	- configured data transmission frame size, in bits (SPI DS)
 *
 * @return					- none
 *
 * @Note					- called within an ISR, non-blocking

 */
static void SPI_RXNE_Interrupt_HelperFunc(__RW SPI_Handle_t *const ptr_SPIHandle, uint8_t data_comm_size)
{
	// Decrement cursor variable to track the moving pointer in RX buffer, defaults to a single byte increment
	uint8_t decrement = 1;

	/* 1. Read transmitted data from DR (RX FIFO) according to defined rx_size
	 * Note: data is right-aligned on 8-bit or 16-bit boundaries
	 */

	/* 2. Write data to SPI data register == RX FIFO */
	if (data_comm_size == 16)
	{
		// Decrement by 2 bytes if data comms size is configured to 16 bits
		decrement = 2;

		// 2.1.1 If 16-bit data, cast the RX buffer pointer to point to 16-bit data, then read from SPI DR (RX FIFO)
		// and write to RX buffer
		*(uint16_t *)ptr_SPIHandle->ptr_RX_Data = ptr_SPIHandle->ptr_SPIx->DR;
		// 2.2.1 cast to point to 16-bit data for correct pointer arithmetic
		// 2.3.1 Move RX buffer pointer to next location to be written, depending on data comm length
		(uint16_t *)ptr_SPIHandle->ptr_RX_Data++;
	}
	else if (data_comm_size == 8)
	{
		// 2.1.2 8-bit data, direct write to SPI DR (RX FIFO) and direct pointer increment
		*(ptr_SPIHandle->ptr_RX_Data) = ptr_SPIHandle->ptr_SPIx->DR;
		// 2.2.2
		// 2.3.2
		ptr_SPIHandle->ptr_RX_Data++;
	}

	// 3. Decrement the data length to find the remaining bytes to receive
	ptr_SPIHandle->RX_DataLength -= decrement;

	// 4. Check if all data has been received and read, to allow closing of SPI communication
	if (ptr_SPIHandle->RX_DataLength <= 0)
	{
		/* Close data reception */
		SPI_CloseReception(ptr_SPIHandle);

		/* Hand over back to the application */
		SPI_ApplicationEventCallback(ptr_SPIHandle, SPI_RX_DONE);
	}
}

/*

 * @fn						- SPI_OVR_Interrupt_HelperFunc
 *
 * @brief					- this function handles a data overrun interrupt event
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 *
 * @return					- none
 *
 * @Note					- called within an ISR, non-blocking

 */
static void SPI_OVR_Interrupt_HelperFunc(__R SPI_Handle_t *const ptr_SPIHandle)
{
	// Confirm that application is not in transmit mode, as overrun occurs only during data reception
	if ( !(ptr_SPIHandle->TX_State) )
	{
		// 1. Clear the OVR flag
		SPI_ClearOVRFlag((__R SPIx_Reg_t *const)ptr_SPIHandle->ptr_SPIx);
	}

	/* 2. Hand over to application */
	SPI_ApplicationEventCallback((__RW SPI_Handle_t *const)ptr_SPIHandle, SPI_OVR_ERR);
}

/*

 * @fn						- SPI_CloseTransmission
 *
 * @brief					- this function closes SPI transmission by resetting all related flags/states
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_CloseTransmission(__RW SPI_Handle_t *const ptr_SPIHandle)
{
	/* 1. Clear transmission event interrupt flag */
	ptr_SPIHandle->ptr_SPIx->CR2 &= ~(CLEAR_ONE_BITMASK << SPI_TXEIE);

	/* 2. Clear data transmission values held in SPI Handle instance */
	ptr_SPIHandle->ptr_TX_Data = NULL;
	ptr_SPIHandle->TX_DataLength = 0;
	ptr_SPIHandle->TX_State = 0;
}

/*

 * @fn						- SPI_CloseReception
 *
 * @brief					- this function closes SPI reception by resetting all related flags/states
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_CloseReception(__RW SPI_Handle_t *const ptr_SPIHandle)
{
	/* 1. Clear reception event interrupt flag */
	ptr_SPIHandle->ptr_SPIx->CR2 &= ~(CLEAR_ONE_BITMASK << SPI_RXNEIE);

	/* 2. Clear data reception values held in SPI Handle instance */
	ptr_SPIHandle->ptr_RX_Data = NULL;
	ptr_SPIHandle->RX_DataLength = 0;
	ptr_SPIHandle->RX_State = 0;
}

/*

 * @fn						- SPI_ClearOVRFlag
 *
 * @brief					- this function clears the OVR flag in SPI status register
 *
 * @param[ptr_SPIx]			- pointer to the base address of the SPI peripheral
 *
 * @return					- none
 *
 * @Note					- none

 */
static void SPI_ClearOVRFlag(__R SPIx_Reg_t *const ptr_SPIx)
{
	/* Clear the OVR flag
	 * A read from SPI DR and a read from SPI SR will clear the OVR flag
	 */

	// Variable to facilitate read access
	uint32_t *temp = NULL;

	// Read from DR and SR
	*temp = ptr_SPIx->DR;
	*temp = ptr_SPIx->SR;
}

/*

 * @fn						- SPI_ApplicationEventCallback
 *
 * @brief					- this function carries out user logic based on an application event
 *
 * @param[ptr_SPIHandle]	- pointer to the SPI Handle structure instance
 * @param[app_event]		- application event to take action on e.g. TX_DONE
 *
 * @return					- none
 *
 * @Note					- Weak implementation, user application should overwrite this

 */
__weak void SPI_ApplicationEventCallback(__unused __RW SPI_Handle_t *const ptr_SPIHandle, __unused uint8_t app_event)
{
	//
}

/*

 * @fn						- SPI_Control
 *
 * @brief					- this function enables or disables SPIx communication, through configuring SPE flag
 *
 * @param[ptr_SPIx]			- pointer to the SPIx port to enable/disable
 * @param[en_di]			- action to take: ENABLE(1) or DISABLE(0)
 *
 * @return					- none
 *
 * @Note					- none

 */
void SPI_Control(__W SPIx_Reg_t *const ptr_SPIx, bool en_di)
{
	(en_di) ? ( ptr_SPIx->CR1 |= (SET_ONE_BITMASK << SPI_ENABLE) ) : ( ptr_SPIx->CR1 &= ~(CLEAR_ONE_BITMASK << SPI_ENABLE) );
}
