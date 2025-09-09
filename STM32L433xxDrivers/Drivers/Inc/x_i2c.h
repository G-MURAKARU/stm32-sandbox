/*
 * x_i2c.h
 *
 *  Created on: Apr 21, 2025
 *      Author: MURAKARU
 */

#ifndef INC_X_I2C_H_
#define INC_X_I2C_H_


#include "stm32l433xx.h"

/* Structure forward declarations */
typedef struct I2C_PeripheralRegisters I2Cx_Reg_t;

/* Pointers to I2C base addresses */
#define I2C1											( (I2Cx_Reg_t *const)I2C1_BASE_ADDR )
#define I2C2											( (I2Cx_Reg_t *const)I2C2_BASE_ADDR )
#define I2C3											( (I2Cx_Reg_t *const)I2C3_BASE_ADDR )

/* RCC Mapping structs for I2Cx */

static const RCC_Periph_t RCC_MAP_I2C1 = { .RCC_BitPos = RCC_I2C1, .RCC_Bus = RCC_APB1_R1 };
static const RCC_Periph_t RCC_MAP_I2C2 = { .RCC_BitPos = RCC_I2C2, .RCC_Bus = RCC_APB1_R1 };
static const RCC_Periph_t RCC_MAP_I2C3 = { .RCC_BitPos = RCC_I2C3, .RCC_Bus = RCC_APB1_R1 };


/*
 * @I2C_PERIPHERAL_REGISTERS
 * Initializer for an I2C peripheral instance, containing all memory-mapped registers
 */
typedef struct I2C_PeripheralRegisters
{
	__RW uint32_t CR1;									/* I2C control register 1 */
	__RW uint32_t CR2;									/* I2C control register 2, see START, STOP, NACK, PECBYTE */
	__RW uint32_t OAR1;									/* I2C own address 1 */
	__RW uint32_t OAR2;									/* I2C own address 2 */
	__RW uint32_t TIMINGR;								/* I2C timing register */
	__RW uint32_t TIMEOUTR;								/* I2C timeout register */
	__RW uint32_t ISR;									/* I2C interrupt and status register */
	__RW uint32_t ICR;									/* I2C interrupt clear register */
	__RW uint32_t PECR;									/* I2C packet error checking register */
	__RW uint32_t RXDR;									/* I2C receive data register */
	__RW uint32_t TXDR;									/* I2C transmit data register */
} I2Cx_Reg_t;

/*
 * @I2C_PORT_CONFIGURATION
 * Initializer for a I2C port's configuration structure, containing all port settings
 */
typedef struct I2CPeripheralConfiguration
{
	uint32_t I2C_SCLSpeed;								/* Master serial clock speed (~100kHz, ~400kHz, ~1MHz) */
	uint8_t  I2C_DeviceAddress;							/* Device's reachable address (in slave mode) */
	uint8_t  I2C_ACKControl;							/*  */
} I2C_Config_t;

/*
 * @I2C_HANDLER
 * Initializer for a I2C handler structure, that manages a I2C instance's configuration and properties
 */
typedef struct I2CHandleStructure
{
	I2Cx_Reg_t 	  *ptr_I2Cx;						/* Holds the base address of the I2C (1/2/3) peripheral */
	I2C_Config_t 	  I2C_Config;						/*  */
} I2C_Handle_t;

/*
 * @I2C_CR1_CONTENTS
 * I2C port control register bits, for shifting
 */
typedef enum I2CControlRegister1
{
	I2C_ENABLE, I2C_TXIE, I2C_RXIE, I2C_ADDRIE, I2C_NACKIE, I2C_STOPIE, I2C_TCIE, I2C_ERRIE, I2C_DNF, I2C_ANFOFF = 12,
	I2C_TXDMAEN = 14, I2C_RXDMAEN, I2C_SBC, I2C_NOSTRETCH, I2C_WUPEN, I2C_GCEN, I2C_SMBHEN, I2C_SMBDEN,
	I2C_ALERTEN, I2C_PECEN,
} I2C_CR1_e;

/*
 * @I2C_CR2_CONTENTS
 * I2C port control register bits, for shifting
 */
typedef enum I2CControlRegister2
{
	I2C_SADD, I2C_RD_nWR = 10, I2C_ADDRSIZE, I2C_HEAD10R, I2C_START, I2C_STOP,
	I2C_NACK, I2C_NBYTES, I2C_RELOAD = 24, I2C_AUTOEND, I2C_PECBYTE,
} I2C_CR2_e;

/*
 * @I2C_CR2_CONTENTS
 * I2C port control register bits, for shifting
 */
typedef enum I2CInterruptStatusRegister
{
	I2C_TXE, I2C_TXIS, I2C_RXNE, I2C_ADDR, I2C_NACKF, I2C_STOPF, I2C_TC, I2C_TCR, I2C_BERR,
	I2C_ARLO, I2C_OVR, I2C_PECERR, I2C_TIMEOUT, I2C_ALERT, I2C_BUSY = 15, I2C_DIR, I2C_ADDCODE,
} I2C_ISR_e;

/*
 * @I2C_SCL_SPEEDS
 * I2C port possible serial clock speed/frequency
 */
typedef enum I2CSCLSpeeds
{
	I2C_STANDARD = 100000, I2C_FAST = 400000, I2C_FAST_PLUS = 1000000,
} I2C_SCL_e;

/* Macros for disable/enable flags */

// ACK
#define I2C_NACK_DISABLE									DISABLE
#define I2C_NACK_ENABLE									ENABLE

/*

 *****************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 *****************************************************************************************

*/

/* I2C Clock Configuration */
void I2C_PeriphClkCtrl(__R I2Cx_Reg_t *const, bool);

/* I2C Initialization */
void I2C_Init(__RH I2C_Handle_t *const);
void I2C_DeInit(__RH I2Cx_Reg_t *const);					/* See RCC Peripheral Reset Register - Resets all registers */

/* I2C Data Read and Write */

// Blocking APIs
void I2C_DataSend(I2Cx_Reg_t *const, __R uint8_t *volatile, int32_t, uint8_t);
void I2C_DataReceive(I2Cx_Reg_t *const, __RW uint8_t *volatile, int32_t, uint8_t);

// Non-Blocking APIs
bool I2C_DataSend_IT(I2C_Handle_t *const, __R uint8_t *const, int32_t);
bool I2C_DataReceive_IT(I2C_Handle_t *const, __R uint8_t *const, int32_t);

/* I2C Interrupt Configuration/Handling */
void I2C_IRQHandlerFunc(__R I2C_Handle_t *const);

/* Other I2C Control Peripherals */
void I2C_Control(I2Cx_Reg_t *const, bool);

/* Application functions */
void I2C_ApplicationEventCallback(I2C_Handle_t *const, uint8_t);



#endif /* INC_X_I2C_H_ */
