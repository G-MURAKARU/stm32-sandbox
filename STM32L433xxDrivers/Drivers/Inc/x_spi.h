/*
 * x_spi.h
 *
 *  Created on: Apr 9, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_SPI_H_
#define INC_X_SPI_H_


#include "stm32l433xx.h"


/* Structure forward declarations */
typedef struct SPI_PeripheralRegisters SPIx_Reg_t;

/* Pointers to SPI base addresses */
#define SPI1											( (__RW SPIx_Reg_t *const)SPI1_BASE_ADDR )
#define SPI2											( (__RW SPIx_Reg_t *const)SPI2_BASE_ADDR )
#define SPI3											( (__RW SPIx_Reg_t *const)SPI3_BASE_ADDR )

/* Clock Enable for SPIx */
#define SPI1_CLK_EN()									( (RCC->APB2ENR) |= (SET_ONE_BITMASK << RCC_SPI1EN) )
#define SPI2_CLK_EN()									( (RCC->APB1ENR1) |= (SET_ONE_BITMASK << RCC_SPI2EN) )
#define SPI3_CLK_EN()									( (RCC->APB1ENR1) |= (SET_ONE_BITMASK << RCC_SPI3EN) )

/* Clock Disable for SPIx */
#define SPI1_CLK_DI()									( (RCC->APB2ENR) &= ~(CLEAR_ONE_BITMASK << RCC_SPI1EN) )
#define SPI2_CLK_DI()									( (RCC->APB1ENR1) &= ~(CLEAR_ONE_BITMASK << RCC_SPI2EN) )
#define SPI3_CLK_DI()									( (RCC->APB1ENR1) &= ~(CLEAR_ONE_BITMASK << RCC_SPI3EN) )

/* SPI Register Reset Macro Definitions */
#define SPI1_REG_RESET()								do { RCC->APB2RSTR |= (SET_ONE_BITMASK << RCC_SPI1RST); RCC->APB2RSTR &= ~(CLEAR_ONE_BITMASK << RCC_SPI1RST); } while (0)
#define SPI2_REG_RESET()								do { RCC->APB1RSTR1 |= (SET_ONE_BITMASK << RCC_SPI2RST); RCC->APB1RSTR1 &= ~(CLEAR_ONE_BITMASK << RCC_SPI2RST); } while (0)
#define SPI3_REG_RESET()								do { RCC->APB1RSTR1 |= (SET_ONE_BITMASK << RCC_SPI3RST); RCC->APB1RSTR1 &= ~(CLEAR_ONE_BITMASK << RCC_SPI3RST); } while (0)

/*
 * @SPI_PERIPHERAL_REGISTERS
 * Initializer for an SPI peripheral instance, containing all memory-mapped registers
 */
typedef struct SPI_PeripheralRegisters
{
	__RW uint32_t CR1;									/* SPI control register 1 */
	__RW uint32_t CR2;									/* SPI control register 2 */
	__RH uint32_t SR;									/* SPI status register, see CRCERR flag */
	__RW uint32_t DR;									/* SPI data register */
	__RW uint32_t CRCPR;								/* SPI CRC polynomial register */
	__RH uint32_t RXCRCR;								/* SPI Rx CRC register */
	__RH uint32_t TXCRCR;								/* SPI Tx CRC register */
} SPIx_Reg_t;

/*
 * @SPI_PORT_CONFIGURATION
 * Initializer for a SPI port's configuration structure, containing all port settings
 */
typedef struct SPIPeripheralConfiguration
{
	uint8_t SPI_DeviceMode;								/* Master or slave mode */
	uint8_t SPI_BusCommsConfig;							/* Full-duplex, half-dduplex or simplex */
	uint8_t SPI_ClockConfig;							/* Baud rate/Clock speed (Hz) */
	uint8_t SPI_DataFrameFormat;						/* 8-bit or 16-bit frames */
	uint8_t SPI_ClockPolarity;							/* SPI bus idle state: LOW or HIGH */
	uint8_t SPI_ClockPhase;								/* Clock transition on first data capture edge */
	uint8_t SPI_SSM;									/* SSM flag configuration */
	uint8_t SPI_CRCEnable;								/* CRC flag configuration */
	uint8_t SPI_CRCLength;								/* CRC polynomial length */
	uint8_t SPI_SSI;									/* SSI flag configuration */
	uint8_t SPI_SSOE;									/* SSOE flag configuration, CR2 */
	uint8_t SPI_NSSP;									/* NSSP flag configuration, CR2 */
} SPI_Config_t;

/*
 * @SPI_HANDLER
 * Initializer for a SPI handler structure, that manages a SPI instance's configuration and properties
 */
typedef struct SPIHandleStructure
{
	__RW SPIx_Reg_t *ptr_SPIx;							/* Holds the base address of the SPI (1/2/3) peripheral */
	SPI_Config_t 	  SPI_Config;						/*  */
	__RW uint8_t    *ptr_TX_Data;						/* Points to the transmit data buffer's storage location */
	__RW uint8_t    *ptr_RX_Data;						/* Points to the receive data buffer's storage location */
	int32_t          TX_DataLength;						/* Stores the length of the data to transmit, in bytes */
	int32_t          RX_DataLength;						/* Stores the length of the data being received, in bytes */
	bool			  TX_State;							/* Stores current SPI data transmission state */
	bool              RX_State;							/* Stores current SPI data reception state */
} SPI_Handle_t;

/*
 * @SPI_CR1_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister1
{
	CPHA, CPOL, SPI_MSTR, SPI_BAUD, SPI_ENABLE = 6, SPI_LSB_FIRST, SSI, SSM, SPI_RXONLY, SPI_CRCL, SPI_CRCNEXT, SPI_CRCEN, SPI_BIDIOE, SPI_BIDIMODE,
} SPI_CR1_e;

/*
 * @SPI_CR2_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister2
{
	SPI_RXDMAEN, SPI_TXDMAEN, SSOE, NSSP, SPI_FRF, SPI_ERRIE, SPI_RXNEIE, SPI_TXEIE, SPI_DATASIZE, SPI_FRXTH = 12, SPI_LDMA_RX, SPI_LDMA_TX,
} SPI_CR2_e;

typedef enum SPIStatusRegister
{
	SPI_RXNE, SPI_TXE, SPI_CRCERR = 4, SPI_MODF, SPI_OVR, SPI_BSY, SPI_FRE, SPI_FRLVL, SPI_FTLVL = 11,
} SPI_SR_e;

/*
 * @SPI_DEVICE_MODES
 * SPI port possible device modes
 */
typedef enum SPIDeviceModes
{
	SPI_SLAVE, SPI_MASTER,
} SPI_Mode_e;

/*
 * @SPI_BUS_MODES
 * SPI port possible bus communication configuration modes
 */
typedef enum SPIBusCommsConfigModes
{
	SPI_FULL_DUPLEX, SPI_HALF_DUPLEX, SPI_SIMPLEX_RX,
} SPI_Bus_e;

/*
 * @SPI_CLOCK_MODES
 * SPI port possible serial clock speed modes (prescalers)
 */
typedef enum SPIClockPrescalers
{
	PRE_2, PRE_4, PRE_8, PRE_16, PRE_32, PRE_64, PRE_128, PRE_256,
} SPI_Clock_e;

/*
 * @SPI_DATA_FRAME_FORMATS
 * SPI port possible data frame formats/sizes
 */
typedef enum SPIDataFrameFormats
{
	FR_4BIT = 3, FR_5BIT, FR_6BIT, FR_7BIT, FR_8BIT, FR_9BIT, FR_10BIT,
	FR_11BIT, FR_12BIT, FR_13BIT, FR_14BIT, FR_15BIT, FR_16BIT,
} SPI_Frame_e;

/*
 * @SPI_CLOCK_POLARITY
 * SPI port possible clock polarity states (idle)
 */
typedef enum SPIClockPolarity
{
	CPOL_LOW, CPOL_HIGH
} SPI_CPOL_e;

/*
 * @SPI_CLOCK_PHASE
 * SPI port possible clock phases
 */
typedef enum SPIClockPhase
{
	CPHA_LOW, CPHA_HIGH
} SPI_CPHA_e;

/*
 * @SPI_CRC_LENGTH
 * SPI port possible CRC polynomial length
 */
typedef enum SPICRCLength
{
	SPI_CRC_8BIT, SPI_CRC_16BIT,
} SPI_CRCLen_e;

/*
 * @SPI_APP_EVENTS
 * SPI port possible application events
 */
typedef enum SPIApplicationEvents
{
	SPI_TX_DONE, SPI_RX_DONE, SPI_OVR_ERR, SPI_CRC_ERR,
} SPI_AppEvent_e;

/* Macros for disable/enable flags */

// Software slave management mode
#define SSM_DISABLE									DISABLE
#define SSM_ENABLE										ENABLE

// Internal slave select mode
#define SSI_DISABLE									DISABLE
#define SSI_ENABLE										ENABLE

// Cyclic redundancy check mode
#define SPI_CRC_DISABLE								DISABLE
#define SPI_CRC_ENABLE								ENABLE

// NSS pulse management mode
#define NSSP_DISABLE									DISABLE
#define NSSP_ENABLE									ENABLE

// Slave select output enable mode
#define SSOE_DISABLE									DISABLE
#define SSOE_ENABLE									ENABLE

/*

 *****************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 *****************************************************************************************

*/

/* SPI Clock Configuration */
void SPI_PeriphClkCtrl(__R SPIx_Reg_t *const, bool);

/* SPI Initialization */
void SPI_Init(__RH SPI_Handle_t *const);
void SPI_DeInit(__RH SPIx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* SPI Data Read and Write */

// Blocking APIs
void SPI_DataSend(__RW SPIx_Reg_t *const, __R uint8_t *volatile, int32_t, uint8_t);
void SPI_DataReceive(__RW SPIx_Reg_t *const, __RW uint8_t *volatile, int32_t, uint8_t);

// Non-Blocking APIs
bool SPI_DataSend_IT(__RW SPI_Handle_t *const, __R uint8_t *const, int32_t);
bool SPI_DataReceive_IT(__RW SPI_Handle_t *const, __R uint8_t *const, int32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQHandlerFunc(__R SPI_Handle_t *const);

/* Other SPI Control Peripherals */
void SPI_Control(__W SPIx_Reg_t *const, bool);
void SPI_CloseTransmission(__RW SPI_Handle_t *const);
void SPI_CloseReception(__RW SPI_Handle_t *const);

/* Application functions */
void SPI_ApplicationEventCallback(__RW SPI_Handle_t *const, uint8_t);


#endif /* INC_X_SPI_H_ */
