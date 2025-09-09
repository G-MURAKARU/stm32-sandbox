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
#define SPI1											( (SPIx_Reg_t *const)SPI1_BASE_ADDR )
#define SPI2											( (SPIx_Reg_t *const)SPI2_BASE_ADDR )
#define SPI3											( (SPIx_Reg_t *const)SPI3_BASE_ADDR )

/* RCC Mapping structs for SPIx */

static const RCC_Periph_t RCC_MAP_SPI1 = { .RCC_BitPos = RCC_SPI1, .RCC_Bus = RCC_APB2 };
static const RCC_Periph_t RCC_MAP_SPI2 = { .RCC_BitPos = RCC_SPI2, .RCC_Bus = RCC_APB1_R1 };
static const RCC_Periph_t RCC_MAP_SPI3 = { .RCC_BitPos = RCC_SPI3, .RCC_Bus = RCC_APB1_R1 };

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
	SPIx_Reg_t 	 *ptr_SPIx;							/* Holds the base address of the SPI (1/2/3) peripheral */
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
	SPI_CPHA, SPI_CPOL, SPI_MSTR, SPI_BAUD, SPI_ENABLE = 6, SPI_LSB_FIRST, SPI_SSI,
	SPI_SSM, SPI_RXONLY, SPI_CRCL, SPI_CRCNEXT, SPI_CRCEN, SPI_BIDIOE, SPI_BIDIMODE,
} SPI_CR1_e;

/*
 * @SPI_CR2_CONTENTS
 * SPI port control register bits, for shifting
 */
typedef enum SPIControlRegister2
{
	SPI_RXDMAEN, SPI_TXDMAEN, SPI_SSOE, SPI_NSSP, SPI_FRF, SPI_ERRIE, SPI_RXNEIE,
	SPI_TXEIE, SPI_DATASIZE, SPI_FRXTH = 12, SPI_LDMA_RX, SPI_LDMA_TX,
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
	SPI_PRE_2, SPI_PRE_4, SPI_PRE_8, SPI_PRE_16, SPI_PRE_32, SPI_PRE_64, SPI_PRE_128, SPI_PRE_256,
} SPI_Clock_e;

/*
 * @SPI_DATA_FRAME_FORMATS
 * SPI port possible data frame formats/sizes
 */
typedef enum SPIDataFrameFormats
{
	SPI_FR_4BIT = 3, SPI_FR_5BIT, SPI_FR_6BIT, SPI_FR_7BIT, SPI_FR_8BIT, SPI_FR_9BIT, SPI_FR_10BIT,
	SPI_FR_11BIT, SPI_FR_12BIT, SPI_FR_13BIT, SPI_FR_14BIT, SPI_FR_15BIT, SPI_FR_16BIT,
} SPI_Frame_e;

/*
 * @SPI_CLOCK_POLARITY
 * SPI port possible clock polarity states (idle)
 */
typedef enum SPIClockPolarity
{
	SPI_CPOL_LOW, SPI_CPOL_HIGH
} SPI_CPOL_e;

/*
 * @SPI_CLOCK_PHASE
 * SPI port possible clock phases
 */
typedef enum SPIClockPhase
{
	SPI_CPHA_LOW, SPI_CPHA_HIGH
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
#define SPI_SSM_DISABLE								DISABLE
#define SPI_SSM_ENABLE								ENABLE

// Internal slave select mode
#define SPI_SSI_DISABLE								DISABLE
#define SPI_SSI_ENABLE								ENABLE

// Cyclic redundancy check mode
#define SPI_CRC_DISABLE								DISABLE
#define SPI_CRC_ENABLE								ENABLE

// NSS pulse management mode
#define SPI_NSSP_DISABLE								DISABLE
#define SPI_NSSP_ENABLE								ENABLE

// Slave select output enable mode
#define SPI_SSOE_DISABLE								DISABLE
#define SPI_SSOE_ENABLE								ENABLE

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
void SPI_DataSend(SPIx_Reg_t *const, __R uint8_t *volatile, int32_t, uint8_t);
void SPI_DataReceive(SPIx_Reg_t *const, __RW uint8_t *volatile, int32_t, uint8_t);

// Non-Blocking APIs
bool SPI_DataSend_IT(SPI_Handle_t *const, __R uint8_t *const, int32_t);
bool SPI_DataReceive_IT(SPI_Handle_t *const, __R uint8_t *const, int32_t);

/* SPI Interrupt Configuration/Handling */
void SPI_IRQHandlerFunc(__R SPI_Handle_t *const);

/* Other SPI Control Peripherals */
void SPI_Control(SPIx_Reg_t *const, bool);
void SPI_CloseTransmission(SPI_Handle_t *const);
void SPI_CloseReception(SPI_Handle_t *const);

/* Application functions */
void SPI_ApplicationEventCallback(SPI_Handle_t *const, uint8_t);


#endif /* INC_X_SPI_H_ */
