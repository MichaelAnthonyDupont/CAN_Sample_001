/*
 * CAN_Exports.h
 *
 *  Created on: Oct 10, 2018
 *      Author: mdupont
 */

#ifndef CAN_EXPORTS_H_
#define CAN_EXPORTS_H_

#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#define GEN_ERR_UNSUPPORTED_CMD		1
#define PROG_ERR_BAD_RESTART			10
#define PROG_ERR_NOT_IN_LOW_FLASH	11
#define PROG_ERR_NO_PROG_BASE		12
#define PROG_ERR_SEQUENCE_ERR		13
#define PROG_ERR_FAIL_FLASH_WRITE	14
#define PROG_ERR_NO_LOAD				15
#define PROG_ERR_CMD_DURING_LOAD		16


#define CAN_ACK_RESPONSE_BIT			0x400
#define CAN_ERROR_RESPONSE_BIT		0x200

#define CAN_ADDRESS_AUTO_ASSIGNED	0x00
#define CAN_GET_ADDRESS				0x01
#define CAN_ASSIGN_ADDRESS			0x02
#define CAN_REQUEST_NEW_ADDRESS		0x03

#define CAN_LED_FLASH_CONTROL		0x04
#define CAN_LED_STATE_CONTROL		0x05
#define CAN_SWITCH_STATE				0x06

// Paving the way for a boot loader
#define CAN_ERASE_SYS_BLOCK			0xE0
#define CAN_ERASE_PROGRAM_BLOCK		0xE1
#define	CAN_PROGRAM_SET_BASE			0xE2
#define CAN_PROGRAM_CLOSE			0xE4
#define CAN_REPORT_VERSION			0xE5
#define CAN_PROGRAM_BLOCK			0xF0		// LS 3 bits are sequence number 0 - 7
#define CAN_PROGRAM_BLOCK_0			(CAN_PROGRAM_BLOCK + 0)
#define CAN_PROGRAM_BLOCK_1			(CAN_PROGRAM_BLOCK + 1)
#define CAN_PROGRAM_BLOCK_2			(CAN_PROGRAM_BLOCK + 2)
#define CAN_PROGRAM_BLOCK_3			(CAN_PROGRAM_BLOCK + 3)
#define CAN_PROGRAM_BLOCK_4			(CAN_PROGRAM_BLOCK + 4)
#define CAN_PROGRAM_BLOCK_5			(CAN_PROGRAM_BLOCK + 5)
#define CAN_PROGRAM_BLOCK_6			(CAN_PROGRAM_BLOCK + 6)
#define CAN_PROGRAM_BLOCK_7			(CAN_PROGRAM_BLOCK + 7)

#define CAN_RESTART_NODE				0xFE


#define CAN_DEFAULT_ID				0x1FF
#define CAN_TEMPORARY_ID				0x1FE
#define CAN_MASTER_ID				0x001
#define CAN_GLOBAL_ID				0x000

typedef enum _CAN_FILTER_TYPES
{
	CAN_FILTER_GLOBAL,
	CAN_FILTER_MASTER,
	CAN_FILTER_TEMPORARY,	// As yet unassigned -- ID is 0x1FE
	CAN_FILTER_CHILD
} CAN_FILTER_TYPES;

typedef enum _CHILD_PROGRAMMING_STATE
{
	CPS_INIT,
	CPS_START,
	CPS_PROG_BLOCK,
	CPS_CLOSE
} CHILD_PROGRAMMING_STATE;

typedef struct _COMPLETE_CAN_RX_MSG
{
	CAN_RxHeaderTypeDef	RxHeader;
	uint8_t        		RxData[8];
} COMPLETE_CAN_RX_MSG;

#endif /* CAN_EXPORTS_H_ */
