/*
 * CANHandler.c
 *
 *  Created on: Oct 12, 2018
 *      Author: mdupont
 */

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include "can.h"

#include <stdbool.h>
#include <string.h>

#include "FlashSupport.h"
#include "CAN_Exports.h"
#include "CANHandler.h"
#include "UARTHandler.h"

extern osMessageQId CAN_ReceiveHandle;
extern osTimerId CAN_LoadErrorHandle;
extern osTimerId LEDFlashHandle;

static int			errorOnCAN = false;
static int			errorCountCAN = 0;

CAN_FilterTypeDef  	sFilterConfig;
CAN_TxHeaderTypeDef	TxHeader;
CAN_RxHeaderTypeDef	RxHeader_0;
CAN_RxHeaderTypeDef RxHeader_1;
uint8_t            	TxData[8];
uint8_t        		RxData_0[8];
uint8_t				RxData_1[8];
uint32_t          	TxMailbox;

bool 				LEDState_On	= false;
bool 				flashMe		= false;
uint32_t				flashRate = 100;
bool					loadTimerExpired = false;
bool					nodeSentLoadError = false;
static bool 			sendFromSwitch = false;

static uint32_t 		myCANId = CAN_DEFAULT_ID;

uint8_t packetSequenceIndex = 0;	// recycling sequence number to assure packet order
uint8_t packetByteIndex = 0;		// where in the packet does the byte go?
uint8_t payload[8];				// actual payload
uint8_t loadRxSeq = 0;			// received packet sequence
int		loadBlockCount = 0;
bool		childLoadError = false;
bool		masterLoadError = false;


// Organize CAN and filtering for the following:
//
// 	RXFIFO0 is for most run-time message processing
//		Diagnostic commands
//			LED_FLASH_ACK <LED#>
//			LED_ON_ACK
//
//  RXFIFO1 is for administrative message processing
//		Identifier Services:
//			Request
//			Assignment ACK
//			RESET ACK
//		Boot Control
//			Boot Mode ACK
//			Set Base Address ACK
//			Set Offset from Base ACK
//			Store Block ACK
//			CLOSE ACK
//		Reset
//
// Identifier Control:
//		ALWAYS use Extended Identifier (29-bit) messages
//
//		0x000 is Reserved for HIGH PRIORITY purposes
//		0x001 is for the MASTER
//		0x002 - 0x0FF are valid slave identifiers
//		0x1FF is for an unassigned, un-commissioned node - use this when asking for an ID
//
//		| 2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1 0 0 | 0 0 0 0 | 0 0 0 0 |
//		| 8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
//		|   |         |         |         |         |  |      |         |         |
//		|          SRC          |         DEST         |       Command            |
//		|        9-bits         |        9-bits        |       11-bits            |
//
//  9-bit SRC:
//	=========
//  SRC - From MASTER: 0x001
//	SRC - from SLAVE: on RAW POWER-UP: 0x1FF.  Nodes with this power-up initial ID are
//				not permitted to send anything to the bus.  Global messages will be
//              heard by these nodes, but no replies will be generated.
//	SRC - from SLAVE: use 0x1FE during IDENT switch initiation of ID request and at no
//				other time.
//  SRC - from SLAVE: after assignment: 0x002 - 0x0FF -- upper bit is reserved for assignment

//  9-bit DEST:
//  ==========
//  DEST - 0x000 is a GLOBAL command -- ALL NODES to comply.  This is typically reserved to the
//         MASTER.
//  DEST - To MASTER: 0x001
//  DEST - To SLAVE: during assignment ONLY: 0x1FE
//	DEST - To SLAVE: this allows sending a message blindly to nodes that have NEVER been
//			commissioned and are not in IDENT.  Slaves with the ID will NOT respond (as stated
//          above.
//  DEST - To assigned SLAVE: 0x002 to 0x0FF
//
//	11-bit Command:
//	==============
//  Command values are 0x000 to 0x1FF
//		bits 0x600 are reserved for ack/error responses
//			Or-on acknowledge bit - 0x400 - upon success of the command
//			Or-on ERROR bit - 0x200 - upon failure.  Fault code is in payload.
//
//	Commands can have values from 0x001 through 0x01FF
//	Command value of 0x000 is reserved
//
//
//	Commands:
//		Boot Management:
//			BootMode
//			SetBaseAddress
//			SetOffset
//			CodePacket
//			Close
//
//		ID Management
//			SET_ID
//			GET_ID
//
//		Diagnostic:
//
//		Runtime:
//


void cbLEDFlash(void const * argument)
{
	if (flashMe)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void cbCANLoadError(void const * argument)
{
	loadTimerExpired = true;
}

void setTestFilters(void)
{

	if (HAL_CAN_Stop(&hcan) != HAL_OK)
	{
		if (HAL_CAN_ERROR_NOT_STARTED != (hcan.ErrorCode & HAL_CAN_ERROR_NOT_STARTED))
		{
			/* Filter configuration Error */
			Error_Handler();
		}
	}

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan, (CAN_IT_TX_MAILBOX_EMPTY    | CAN_IT_RX_FIFO0_MSG_PENDING      | \
										  CAN_IT_RX_FIFO0_FULL        | CAN_IT_RX_FIFO0_OVERRUN          | \
										  CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL             | \
										  CAN_IT_RX_FIFO1_OVERRUN     | CAN_IT_WAKEUP                    | \
										  CAN_IT_SLEEP_ACK            | CAN_IT_ERROR_WARNING             | \
										  CAN_IT_ERROR_PASSIVE        | CAN_IT_BUSOFF                    | \
										  CAN_IT_LAST_ERROR_CODE      | CAN_IT_ERROR)) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	TxHeader.StdId = 0x3FF;		// Start up with a bogus address to get a legit address from Master
	TxHeader.ExtId = 0x1FFFFFFF;

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;//CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;

}
void setFilters(CAN_FILTER_TYPES filterType)
{
	//setTestFilters();
	//return;

	if (HAL_CAN_Stop(&hcan) != HAL_OK)
	{
		if (HAL_CAN_ERROR_NOT_STARTED != (hcan.ErrorCode & HAL_CAN_ERROR_NOT_STARTED))
		{
			/* Filter configuration Error */
			Error_Handler();
		}
	}

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;


	// Filters for GLOBAL Messages (accept EVERYTHING)
	if (CAN_FILTER_GLOBAL == filterType)
	{
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
	  }
	}
	else if (CAN_FILTER_MASTER == filterType)
	{
		  {
			  //					Source ID		| Dest ID		| CAN Command
			  // Filter for eid CAN_TEMPORARY_ID | CAN_MASTER_ID | CAN_REQUEST_ADDRESS
			  uint32_t filterId = (CAN_TEMPORARY_ID << (32-9)) 	| \
								  (CAN_MASTER_ID << (32-18)) 	| \
								  (CAN_REQUEST_NEW_ADDRESS << 3)	| \
								  CAN_ID_EXT						| \
								  CAN_RTR_DATA;
			  uint32_t filterMask = 0xFFFFFFFE;	// ALL BITS MUST MATCH
			  sFilterConfig.FilterBank = 0;
			  sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			  sFilterConfig.FilterIdLow = (filterId & 0xFFFF);
			  sFilterConfig.FilterMaskIdHigh = ((filterMask >> 16) & 0xFFFF);
			  sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		  }
		  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		  {
		    /* Filter configuration Error */
		    Error_Handler();
		  }

		  {
			  //					Source ID		| Dest ID		| CAN Command with OR'd ACK bit
			  // Filter for eid <anyValidTarget> | CAN_MASTER_ID | <anyValidCommand | AckResponseBit>
			  uint32_t filterId = (0x0FF << (32-9)) 			| \
					  	  	  	  (CAN_MASTER_ID << (32-18))	| \
								  (0x7FF <<  3) 				| \
								  CAN_ID_EXT | \
								  CAN_RTR_DATA;
			  uint32_t filterMask =	(0x100 << (32-9))			| \
									(CAN_MASTER_ID << (32-18))	| \
									(CAN_ACK_RESPONSE_BIT << 3)	| \
									CAN_ID_EXT | \
									CAN_RTR_DATA;
			  sFilterConfig.FilterBank = 1;
			  sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			  sFilterConfig.FilterIdLow = (filterId & 0xFFFF);
			  sFilterConfig.FilterMaskIdHigh = ((filterMask >> 16) & 0xFFFF);
			  sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		  }
		  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		  {
		    /* Filter configuration Error */
		    Error_Handler();
		  }
		  {
			  //					Source ID		| Dest ID		| CAN Command with OR'd Error bit
			  // Filter for eid <anyValidTarget> | CAN_MASTER_ID | <anyValidCommand | ErrorResponseBit>
			  uint32_t filterId = (0x0FF << (32-9))			| \
					  	  	  	  (CAN_MASTER_ID << (32-18))	| \
								  (0x7FF << 3)				| \
								  CAN_ID_EXT 				| \
								  CAN_RTR_DATA;
			  uint32_t filterMask = (0x100 << (32-9))	| \
					  (CAN_MASTER_ID << (32-18))			| \
					  (CAN_ERROR_RESPONSE_BIT << 3)		| \
					  CAN_ID_EXT							| \
					  CAN_RTR_DATA;
			  sFilterConfig.FilterBank = 2;
			  sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			  sFilterConfig.FilterIdLow = (filterId & 0xFFFF);
			  sFilterConfig.FilterMaskIdHigh = ((filterMask >> 16) & 0xFFFF);
			  sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		  }
		  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		  {
		    /* Filter configuration Error */
		    Error_Handler();
		  }

	}
	else if (CAN_FILTER_TEMPORARY == filterType)
	{
		{
			// Filter for eid 0x1ffffffE ONLY
			//	Source ID		| Dest ID		| CAN Command
			//	CAN_MASTER_ID	| CAN_GLOBAL_ID | <anyCommand_NoResponseBits>
			uint32_t filterId = (CAN_MASTER_ID << (32-9))	| \
					(CAN_TEMPORARY_ID << (32-18))			| \
					(CAN_ASSIGN_ADDRESS << 3)				| \
					CAN_ID_EXT								| \
					CAN_RTR_DATA;
			uint32_t filterMask = (0x1FF << (32-9))	| \
					(0x1FF << (32-18))         		| \
					(0x7FF << 3)						| \
					CAN_ID_EXT						| \
					CAN_RTR_DATA;
			sFilterConfig.FilterBank = 0;
			sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			sFilterConfig.FilterIdLow = (filterId  & 0xFFFF);
			sFilterConfig.FilterMaskIdHigh = (filterMask >> 16) & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		}
		if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
	}
	else if (CAN_FILTER_CHILD == filterType)
	{
		{
			//	Source ID		| Dest ID		| CAN Command with OR'd ACK bit
			//	CAN_MASTER_ID	| CAN_GLOBAL_ID	| <anyCommand_NoResponseBits>
			uint32_t filterId = (CAN_MASTER_ID << (32-9))	| \
					(CAN_GLOBAL_ID << (32-18))				| \
					(0x000 << 3)								| \
					CAN_ID_EXT								| \
					CAN_RTR_DATA;
			uint32_t filterMask = (0x1FF << (32-9))	| \
					(0x1FF << (32-18))        		| \
					((CAN_ERROR_RESPONSE_BIT | CAN_ACK_RESPONSE_BIT) << 3)	| \
							CAN_ID_EXT 				| \
							CAN_RTR_DATA;
			sFilterConfig.FilterBank = 0;
			sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			sFilterConfig.FilterIdLow = (filterId  & 0xFFFF);
			sFilterConfig.FilterMaskIdHigh = (filterMask >> 16) & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		}
		if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}

		{
			//	Source ID		| Dest ID				| CAN Command with OR'd ACK bit
			//	CAN_MASTER_ID	| <myCANId>				| <anyCommand_NoResponseBits>
			uint32_t filterId = (CAN_MASTER_ID << (32-9))	| \
					(myCANId << (32-18))						| \
					(0x000 << 3)								| \
					CAN_ID_EXT								| \
					CAN_RTR_DATA;
			uint32_t filterMask = (0x1FF << (32-9))	| \
					(0x1FF << (32-18))				| \
					((CAN_ERROR_RESPONSE_BIT | CAN_ACK_RESPONSE_BIT) << 3)	| \
					CAN_ID_EXT						| \
					CAN_RTR_DATA;
			sFilterConfig.FilterBank = 1;
			sFilterConfig.FilterIdHigh = (filterId >> 16) & 0xFFFF;
			sFilterConfig.FilterIdLow = (filterId  & 0xFFFF);
			sFilterConfig.FilterMaskIdHigh = (filterMask >> 16) & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = (filterMask & 0xFFFF);
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		}
		if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
		{
			/* Filter configuration Error */
			Error_Handler();
		}
	}

	  /*##-3- Start the CAN peripheral ###########################################*/
	  if (HAL_CAN_Start(&hcan) != HAL_OK)
	  {
	    /* Start Error */
	    Error_Handler();
	  }

	  /*##-4- Activate CAN RX notification #######################################*/
	  if (HAL_CAN_ActivateNotification(&hcan, (CAN_IT_TX_MAILBOX_EMPTY    | CAN_IT_RX_FIFO0_MSG_PENDING      | \
	                                  	  	  CAN_IT_RX_FIFO0_FULL        | CAN_IT_RX_FIFO0_OVERRUN          | \
											  CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL             | \
											  CAN_IT_RX_FIFO1_OVERRUN     | CAN_IT_WAKEUP                    | \
											  CAN_IT_SLEEP_ACK            | CAN_IT_ERROR_WARNING             | \
											  CAN_IT_ERROR_PASSIVE        | CAN_IT_BUSOFF                    | \
											  CAN_IT_LAST_ERROR_CODE      | CAN_IT_ERROR)) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }

	  /*##-5- Configure Transmission process #####################################*/
		TxHeader.StdId = 0x3FF;		// Start up with a bogus address to get a legit address from Master
		TxHeader.ExtId = 0x1FFFFFFF;

		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_EXT;//CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;
}

void setFiltersForNodeType(void)
{
	static uint32_t myLastId = CAN_DEFAULT_ID;

	CAN_FILTER_TYPES newFiltering;

	switch(myCANId)
	{
	case CAN_DEFAULT_ID:
		newFiltering = CAN_FILTER_GLOBAL;
		break;
	case CAN_TEMPORARY_ID:
		newFiltering = CAN_FILTER_TEMPORARY;
		break;
	case CAN_MASTER_ID:
		newFiltering = CAN_FILTER_MASTER;
		break;
	default:
		newFiltering = CAN_FILTER_CHILD;
		break;
	}
	if (myLastId != myCANId)
	{
		setFilters(newFiltering);
	}

	myLastId = myCANId;
}

uint32_t formExtendedIdentifier(uint32_t destinationId, uint16_t command)
{
	uint32_t extId = 0;

	extId = myCANId << 20;
	extId |= (destinationId << 11);
	extId |= command & 0x7FF;
	return(extId);
}

void getEIDParts(uint32_t extId, uint16_t *source, uint16_t *destination, uint16_t *command)
{
	*source = 0;
	*destination = 0;
	*command = 0;

	*source = (uint16_t)((extId >> (29-9)) & 0x01FF);
	*destination = (uint16_t)((extId >> (29-18)) & 0x1FF);
	*command = (uint16_t)(extId & 0x7FF);
}

bool reply(uint8_t codeToReply)
{

	TxHeader.DLC = 8;
	TxHeader.ExtId = formExtendedIdentifier(CAN_MASTER_ID, codeToReply | CAN_ACK_RESPONSE_BIT);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		return(false);
	}
	return(true);
}

bool replyWithError(uint8_t codeToReply, uint8_t errorCode)
{
	TxHeader.DLC = 1;
	TxHeader.ExtId = formExtendedIdentifier(CAN_MASTER_ID, codeToReply | CAN_ERROR_RESPONSE_BIT);
	TxData[0] = errorCode;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		return(false);
	}
	return(true);
}

void doLEDFlash(bool flashState)
{
	flashMe = flashState;
	LEDState_On = false;

	switch(flashState)
	{
	case true:
		osTimerStart(LEDFlashHandle, flashRate);
		break;
	case false:
		osTimerStop(LEDFlashHandle);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}
}

void doLEDState(bool state)
{
	LEDState_On = state;
	osTimerStop(LEDFlashHandle);

	switch(LEDState_On)
	{
	case true:
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		break;
	case false:
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		break;
	}
}

void refresh(void)
{
	if (0 != errorCountCAN)
	{
		if (false == errorOnCAN)
		{
			flashMe = true;
			flashRate = 50;
			osTimerStart(LEDFlashHandle, flashRate);
			errorOnCAN = true;
		}
	}
}


uint32_t CAN_MyID(void)
{
	return(myCANId);
}


bool CAN_IAmMaster(void)
{
	return(CAN_MASTER_ID == CAN_MyID());
}

bool CAN_IAmAssignedChild(void)
{
	return((CAN_DEFAULT_ID != myCANId) &&
			(CAN_TEMPORARY_ID != myCANId) &&
			(CAN_MASTER_ID != myCANId));
}

void CAN_ClaimMaster(void)
{
	// Plug ID into FLASH
	ProgramIdIntoFlash(CAN_MASTER_ID);
	myCANId = GetIdFromFlash();

	setFilters(CAN_FILTER_MASTER);
}

bool CAN_GetAddresses(void)
{
	TxHeader.DLC = 1;
	TxHeader.ExtId = formExtendedIdentifier(CAN_GLOBAL_ID, CAN_GET_ADDRESS);

	TxData[0] = 0x00;
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		return(false);
	}
	return(true);
}

bool CAN_AssignAddress(uint32_t addr)
{
	if ((CAN_GLOBAL_ID == addr) ||
		(CAN_MASTER_ID == addr) ||
		(100 <= addr))
	{
		// Invalid address requested
		return(false);
	}

	TxHeader.DLC = 2;
	TxHeader.ExtId = formExtendedIdentifier(CAN_TEMPORARY_ID, CAN_ASSIGN_ADDRESS);

	TxData[0]=(addr >> 8) & 0x01;
	TxData[1]=(addr >> 0) & 0xFF;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      return(false);
    }
	return(true);
}

bool CAN_RequestAddress(void)
{
	TxHeader.DLC = 0;
	TxHeader.ExtId = formExtendedIdentifier(CAN_MASTER_ID, CAN_REQUEST_NEW_ADDRESS);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      return(false);
    }
	return(true);

}

bool CAN_LEDFlashControl(uint32_t addr, bool state)
{
	if (CAN_MASTER_ID == myCANId)
	{
		if (CAN_MASTER_ID == addr)
		{
			doLEDFlash(state);
			return(true);
		}
		if (CAN_GLOBAL_ID == addr)
		{
			doLEDFlash(state);
		}
	}

	TxHeader.DLC = 1;
	TxHeader.ExtId = formExtendedIdentifier(addr, CAN_LED_FLASH_CONTROL);

	TxData[0]=(false == state) ? 0x00 : 0x01;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      return(false);
    }
	return(true);
}

bool CAN_LEDStateControl(uint32_t addr, bool state)
{
	if (CAN_MASTER_ID == myCANId)
	{
		if (CAN_MASTER_ID == addr)
		{
			doLEDState(state);
			return(true);
		}
		if (CAN_GLOBAL_ID == addr)
		{
			doLEDState(state);
		}
	}

	TxHeader.DLC = 1;
	TxHeader.ExtId = formExtendedIdentifier(addr, CAN_LED_STATE_CONTROL);

	TxData[0]=(false == state) ? 0x00 : 0x01;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
      return(false);
    }
	return(true);
}

bool CAN_EraseSysBlock(uint32_t addr)
{
	if ((myCANId == CAN_MASTER_ID) && (addr == CAN_MASTER_ID))
	{
		EraseSystemBlock();
		myCANId = GetIdFromFlash();
		return(true);
	}
	if ((myCANId == CAN_MASTER_ID) && (addr == CAN_GLOBAL_ID))
	{
		// Don't erase the CAN_MASTER_ID just yet...
		EraseSystemBlock();
	}

	TxHeader.DLC = 0;
	TxHeader.ExtId = formExtendedIdentifier(addr, CAN_ERASE_SYS_BLOCK);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}
	myCANId = GetIdFromFlash();	// NOW erase the CAN_MASTER_ID
	return(true);
}

bool CAN_EraseProgramBlock(uint32_t addr)
{
	if ((myCANId == CAN_MASTER_ID) && (addr == CAN_MASTER_ID))
	{
		InvalidateProgram();
		return(true);
	}
	if ((myCANId == CAN_MASTER_ID) && (addr == CAN_GLOBAL_ID))
	{
		// Don't erase the CAN_MASTER_ID just yet...
		InvalidateProgram();
	}

	TxHeader.DLC = 0;
	TxHeader.ExtId = formExtendedIdentifier(addr, CAN_ERASE_PROGRAM_BLOCK);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}

	InvalidateProgram();
	return(true);
}

bool reportSwitch(bool state)
{

	TxHeader.DLC = 1;
	TxHeader.ExtId = formExtendedIdentifier(CAN_MASTER_ID, CAN_SWITCH_STATE | CAN_ACK_RESPONSE_BIT);

	TxData[0] = (false == state) ? 0x00 : 0x01;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}
	return(true);
}

bool loadStartChildren(void)
{
	uint32_t tmp = GetLoadBase();

	// CREATE AND START LOAD Error Timer
	if (NULL == CAN_LoadErrorHandle)
	{
		Error_Handler();
	}

	loadTimerExpired = false;
	nodeSentLoadError = false;
	osTimerStart(CAN_LoadErrorHandle, 5000);

	TxHeader.DLC = 4;
	TxHeader.ExtId = formExtendedIdentifier(GetLoadId(), CAN_PROGRAM_SET_BASE);
	TxData[0] = (uint8_t)((tmp>>24) & 0xFF);
	TxData[1] = (uint8_t)((tmp>>16) & 0xFF);
	TxData[2] = (uint8_t)((tmp>>8) & 0xFF);
	TxData[3] = (uint8_t)(tmp & 0xFF);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}

	// If anyone comes back with an error in under 5 seconds,
	// BAIL!!!
	// While the timer is alive AND no one reports an error, loop.
	while(false == loadTimerExpired)
	{
		if (true == nodeSentLoadError)
		{
			return(false);
		}
	}
	return(true);
}

bool CAN_ProgramStart(int id, uint32_t baseAddr)
{
	LoadSetup(id, baseAddr);

	if ((myCANId == CAN_MASTER_ID) && (id == CAN_MASTER_ID))
	{
		if (false == IAmInLowFlash())
		{
			return(false);
		}
		EraseProgramBlock();
		InvalidateProgram();
		masterLoadError = false;
		return(true);
	}
	if (id == CAN_GLOBAL_ID)
	{
		if (false == IAmInLowFlash())
		{
			return(false);
		}
		EraseProgramBlock();
		InvalidateProgram();
		masterLoadError = false;
	}

	// fire-out to designated child(ren).
	packetSequenceIndex = 0;
	packetByteIndex = 0;
	memset(payload, 0, 8);
	return(loadStartChildren());
}


bool packetReady(uint8_t ch)
{
    payload[packetByteIndex] = ch;
    packetByteIndex++;
    packetByteIndex &= 0x07;

	if (0 == packetByteIndex)
	{
		return(true);
	}
	return(false);
}

bool CAN_ProgramChar(uint8_t ch)
{
    uint32_t res = HAL_OK;

    if ((myCANId == CAN_MASTER_ID) && (GetLoadId() == CAN_MASTER_ID))
	{
		return(WriteToFlashBuffer(ch, true));
	}
	if (CAN_GLOBAL_ID == GetLoadId())
	{
		res = WriteToFlashBuffer(ch, true);	//TO MASTER and Children!
	}
	else
	{
		res = WriteToFlashBuffer(ch, false);	//To CHILDREN ONLY
	}

	if (true == packetReady(ch))
	{
		// Write to CAN
		TxHeader.DLC = 8;
		TxHeader.ExtId = formExtendedIdentifier(GetLoadId(), CAN_PROGRAM_BLOCK + packetSequenceIndex);

		res = HAL_CAN_AddTxMessage(&hcan, &TxHeader, payload, &TxMailbox) == HAL_OK;
		memset(payload, 0, 8);
		packetSequenceIndex++;
		packetSequenceIndex &= 0x07;
	}

	return(res);
}


bool CAN_ProgramClose(void)
{
	if ((myCANId == CAN_MASTER_ID) && (GetLoadId() == CAN_MASTER_ID))
	{
		if (false == DidLoadOccur())
		{
			return(false);
		}
		// write to FLASH
		FlushFlashBuffer();
		if (false == masterLoadError)
		{
			ValidateProgram();
		}
		return(true);
	}

	// Write whatever is in the accumulated packet to CAN
	TxHeader.DLC = 8;
	TxHeader.ExtId = formExtendedIdentifier(GetLoadId(), CAN_PROGRAM_CLOSE);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, payload , &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}

	if (CAN_GLOBAL_ID == GetLoadId())
	{
		if (false == DidLoadOccur())
		{
			return(false);
		}
		// write to FLASH
		FlushFlashBuffer();
		if (false == masterLoadError)
		{
			ValidateProgram();
		}
	}
	return(true);
}

bool CAN_RestartNode(int id)
{
	if ((myCANId == CAN_MASTER_ID) && (id == CAN_MASTER_ID))
	{
		// write to FLASH
		RestartNode();
		return(true);
	}

	// Write to CAN
	TxHeader.DLC = 0;
	TxHeader.ExtId = formExtendedIdentifier(id, CAN_RESTART_NODE);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, GetPacket() , &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}

	if (CAN_GLOBAL_ID == id)
	{
		RestartNode();	// RESTART AFTER telling others to restart!
	}
	return(true);
}

bool CAN_GetReportVersion(int id)
{
	if ((myCANId == CAN_MASTER_ID) && (id == CAN_MASTER_ID))
	{
		// write to FLASH
		WriteUARTString("MASTER NODE: ");
		WriteUARTString(versionString);
		WriteUARTString("\n");
		return(true);
	}

	if (id == CAN_GLOBAL_ID)
	{
		WriteUARTString("MASTER NODE: ");
		WriteUARTString(versionString);
		WriteUARTString("\n");
	}

	// Write to CAN
	TxHeader.DLC = 0;
	TxHeader.ExtId = formExtendedIdentifier(id, CAN_REPORT_VERSION);

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, GetPacket() , &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  return(false);
	}
	return(true);
}

/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	COMPLETE_CAN_RX_MSG messageGuts;
	BaseType_t pxHigherPriorityTaskWoken;

	/* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader_0, RxData_0) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  messageGuts.RxHeader = RxHeader_0;
  memcpy(messageGuts.RxData, RxData_0, 8);
  xQueueSendFromISR(CAN_ReceiveHandle, (const void *)&messageGuts, &pxHigherPriorityTaskWoken);

  errorCountCAN = 0;

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	COMPLETE_CAN_RX_MSG messageGuts;
	BaseType_t pxHigherPriorityTaskWoken;

	/* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader_1, RxData_1) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  messageGuts.RxHeader = RxHeader_1;
  memcpy(messageGuts.RxData, RxData_1, 8);
  xQueueSendFromISR(CAN_ReceiveHandle, (const void *)&messageGuts, &pxHigherPriorityTaskWoken);

  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  errorCountCAN = 0;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	errorCountCAN++;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    /* Toggle LED2 */
    sendFromSwitch = true;
  }
}

void doMASTER(COMPLETE_CAN_RX_MSG *messageGutsPtr)
{
	uint16_t source;
	uint16_t destination;
	uint16_t command;
	//char *msgPtr = (char *)pvPortMalloc(128);

	getEIDParts(messageGutsPtr->RxHeader.ExtId, &source, &destination, &command);

	//sprintf(msgPtr, "Msg node %d: ", source);

	switch (command)
	{
	case CAN_REQUEST_NEW_ADDRESS:
	{
		if	 (CAN_TEMPORARY_ID == source)
		{
			WriteUARTString("Need Node address: ");
			//strcat(msgPtr, "CAN_REQUEST_NEW_ADDRESS\n");
		}
	}
	  break;

	case (CAN_ADDRESS_AUTO_ASSIGNED | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_ADDRESS_AUTO_ASSIGNED\n");
		break;

	case (CAN_GET_ADDRESS | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_GET_ADDRESS\n");
		break;

	case (CAN_ASSIGN_ADDRESS | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_ASSIGN_ADDRESS\n");
		break;

	case (CAN_REQUEST_NEW_ADDRESS | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_REQUEST_NEW_ADDRESS\n");
		break;

	case (CAN_LED_FLASH_CONTROL | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_LED_FLASH_CONTROL\n");
		break;

	case (CAN_LED_STATE_CONTROL | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_LED_STATE_CONTROL\n");
		break;

	case (CAN_SWITCH_STATE | CAN_ACK_RESPONSE_BIT):
		//strcat(msgPtr, "ACK CAN_SWITCH_STATE: ");
		switch(messageGutsPtr->RxData[0])
		{
		case 0x00:
			//strcat(msgPtr, "CLOSED\n");
			break;
		case 0x01:
			//strcat(msgPtr, "OPEN\n");
			break;
		default:
			//strcat(msgPtr, "UNKNONW\n");
			break;
		}
	  break;

	  case (CAN_PROGRAM_SET_BASE | CAN_ERROR_RESPONSE_BIT):
		{
		  nodeSentLoadError = true;
		}
	  break;

	default:
	  //strcat(msgPtr, "Unsupported Message\n");
	  break;

	}
	//WriteUARTString(msgPtr);
	//vPortFree(msgPtr);
	UART_ReportReceivedMessage(source, destination, command, messageGutsPtr->RxData);
}

void doTEMPORARY(COMPLETE_CAN_RX_MSG *messageGutsPtr)
{
	uint16_t				source;
	uint16_t				destination;
	uint16_t				command;

	getEIDParts(messageGutsPtr->RxHeader.ExtId, &source, &destination, &command);

	if (CAN_ASSIGN_ADDRESS == command)
	{
		uint32_t newNodeAddress = 0;
		newNodeAddress |= messageGutsPtr->RxData[0];	newNodeAddress <<= 8;
		newNodeAddress |= messageGutsPtr->RxData[1];
		TxHeader.ExtId = newNodeAddress << 20;
		myCANId = newNodeAddress;
		ProgramIdIntoFlash(myCANId);
		reply(CAN_ASSIGN_ADDRESS);
	}
}

bool programmingChild(COMPLETE_CAN_RX_MSG *messageGutsPtr)
{
	static CHILD_PROGRAMMING_STATE cpState = CPS_INIT;

	uint16_t				source;
	uint16_t				destination;
	uint16_t				command;

	getEIDParts(messageGutsPtr->RxHeader.ExtId, &source, &destination, &command);

	switch(command)
	{
	  case CAN_PROGRAM_SET_BASE:
	  {
		  switch(cpState)
		  {
		  case CPS_INIT:
		  case CPS_START: // Multiple starts are permitted...to start with...
			  break;

		  default: //FAIL ON ALL OTHER CASES SINCE WE'RE ALREADY LOADING!!!
			  childLoadError = true;
			  replyWithError(command, PROG_ERR_BAD_RESTART);
			  return(false);
			  break;
		  }

		  // ERROR IF WE'RE NOT RUNNING FROM LOW FLASH!!!
		  if (false == IAmInLowFlash())
		  {
			  replyWithError(command, PROG_ERR_NOT_IN_LOW_FLASH);
			  cpState = CPS_INIT;
			  childLoadError = false;
			  return(true); // bypass further command processing
		  }
		  // Do it
		  uint32_t baseAddr = 0;

		  baseAddr |= messageGutsPtr->RxData[0];
		  baseAddr <<= 8;
		  baseAddr |= messageGutsPtr->RxData[1];
		  baseAddr <<= 8;
		  baseAddr |= messageGutsPtr->RxData[2];
		  baseAddr <<= 8;
		  baseAddr |= messageGutsPtr->RxData[3];
		  LoadSetup(myCANId, baseAddr);

		  loadRxSeq = 0;
		  loadBlockCount = 0;
		  cpState = CPS_START;
		  reply(command);
		  return(true);
	  }
		  break;

	  case CAN_PROGRAM_BLOCK_0:
	  case CAN_PROGRAM_BLOCK_1:
	  case CAN_PROGRAM_BLOCK_2:
	  case CAN_PROGRAM_BLOCK_3:
	  case CAN_PROGRAM_BLOCK_4:
	  case CAN_PROGRAM_BLOCK_5:
	  case CAN_PROGRAM_BLOCK_6:
	  case CAN_PROGRAM_BLOCK_7:
	  {
		  switch(cpState)
		  {
		  case CPS_START:
			  cpState = CPS_PROG_BLOCK;
			  // fall thru intentionally -- this is legit.
		  case CPS_PROG_BLOCK:
			  break;

		  default:	// No other case is legit -- can't jump immediately into programming
			  	  	// without base set as above.
			  replyWithError(command, PROG_ERR_NO_PROG_BASE);
			  cpState = CPS_INIT;
			  childLoadError = true;
			  return(true); // bypass further command processing
			  break;
		  }

		// Check command for proper sequence
		// If out of sequence, reply with error!!!
		if (0 != (loadRxSeq ^ (command & 0x0F)))
		{
			replyWithError(command, PROG_ERR_SEQUENCE_ERR);
			cpState = CPS_INIT;
			childLoadError = true;
			return(true); // bypass further command processing
		}

		loadRxSeq++;
		loadRxSeq %= 8;
		for (int i = 0; i < messageGutsPtr->RxHeader.DLC; i++)
		{
			if (HAL_OK == WriteToFlashBuffer(messageGutsPtr->RxData[i], true))
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
			else
			{
				replyWithError(command, PROG_ERR_FAIL_FLASH_WRITE);
				cpState = CPS_INIT;
				childLoadError = true;
				return(true); // bypass further command processing
			}
		}
		loadBlockCount++;
		//reply(command);
		return(true); // bypass further command processing
	  }
		  break;

	  case CAN_PROGRAM_CLOSE:
	  {

		  for (int i = 0; i < messageGutsPtr->RxHeader.DLC; i++)
		  {
			  if (HAL_OK != WriteToFlashBuffer(messageGutsPtr->RxData[i], true))
			  {
				  childLoadError = true;
				  replyWithError(command, PROG_ERR_FAIL_FLASH_WRITE);
				  cpState = CPS_INIT;
				  return(true); // bypass further command processing
			  }
		  }
		  if (false == DidLoadOccur())
		  {
			  childLoadError = true;
			  replyWithError(command, PROG_ERR_NO_LOAD);
			  cpState = CPS_INIT;
			  return(true); // bypass further command processing
		  }
		  if (HAL_OK != FlushFlashBuffer())
		  {
			  replyWithError(command, PROG_ERR_FAIL_FLASH_WRITE);
			  cpState = CPS_INIT;
			  childLoadError = true;
			  return(true); // bypass further command processing
		  }
		  if (false == childLoadError)
		  {
			  ValidateProgram();
		  }
		  cpState = CPS_INIT;
		  reply(command);
		  return(true);
	  }
		  break;

	  default:
	  {
		  // Catch the case where a non-programming command came in and we're
		  // in the middle of programming.  NOT COOL.
		  if (CPS_INIT != cpState)
		  {
			  replyWithError(command, PROG_ERR_CMD_DURING_LOAD);
			  cpState = CPS_INIT;
			  childLoadError = true;
			  return(true);
		  }
		  return(false);	// We're NOT programming and this ISN'T a programming command.
	  }
		  break;
	}
}

void doCHILD(COMPLETE_CAN_RX_MSG *messageGutsPtr)
{
	  uint16_t				source;
	  uint16_t				destination;
	  uint16_t				command;

	  if (true == programmingChild(messageGutsPtr))
	  {
		  return;
	  }

	  getEIDParts(messageGutsPtr->RxHeader.ExtId, &source, &destination, &command);

	  switch(command)
	  {
	  case CAN_GET_ADDRESS:
	  {
		uint32_t tmp = GetMyLocationInFlash();

		TxHeader.DLC = 4;
		TxData[0] = (uint8_t)((tmp>>24) & 0xFF);
		TxData[1] = (uint8_t)((tmp>>16) & 0xFF);
		TxData[2] = (uint8_t)((tmp>>8) & 0xFF);
		TxData[3] = (uint8_t)(tmp & 0xFF);
	  }
	  	  break;
	  case CAN_LED_FLASH_CONTROL:
	  {
		  doLEDFlash((messageGutsPtr->RxData[0] == 0) ? false : true);
	  }
		  break;
	  case CAN_LED_STATE_CONTROL:
	  {
		  doLEDState((messageGutsPtr->RxData[0] == 0) ? false : true);
	  }
		  break;
	  case CAN_ERASE_SYS_BLOCK:
	  {
		  EraseSystemBlock();
		  myCANId = GetIdFromFlash();	//Now back to square 1
		  return; // SHORT CIRCUIT -- DON'T SEND REPLY
	  }
		  break;

	  case CAN_REPORT_VERSION:
	  {
			TxHeader.DLC = 4;
			TxData[0] = versionCode[0];
			TxData[1] = versionCode[1];
			TxData[2] = versionCode[2];
			TxData[3] = versionCode[3];
	  }
		  break;


	  case CAN_ERASE_PROGRAM_BLOCK:
		  InvalidateProgram();
		  break;

	  case CAN_RESTART_NODE:
		  RestartNode();
		  return;
		  break;

	  default:
		  replyWithError(command, GEN_ERR_UNSUPPORTED_CMD);
		  return;
		  break;
	  }

	  reply(command);

}

void	 DoCANProcessing(void)
{

	COMPLETE_CAN_RX_MSG messageGuts;

	if (false == StartSync())
	{
		return;
	}

	ReportFlash();
	setFiltersForNodeType();

	if (true == sendFromSwitch)
	{
		if (CAN_MASTER_ID == myCANId)
		{
			  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
			  {
				  WriteUARTString("Msg node 1: CLOSED\n");
			  }
			  else
			  {
				  WriteUARTString("Msg node 1: OPEN\n");
			  }
		}
		else if (CAN_DEFAULT_ID == myCANId)
		{
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
			{
				myCANId = CAN_TEMPORARY_ID;
				setFiltersForNodeType();
				CAN_RequestAddress();
			}
		}
		else
		{
			  reportSwitch((GPIO_PIN_RESET == HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) ? 0x00 : 0x01);
		}
		sendFromSwitch = false;
	}

  if (pdTRUE == xQueueReceive(CAN_ReceiveHandle, &messageGuts, 0))
  {
	  switch(myCANId)
	  {
	  case CAN_MASTER_ID:
		  doMASTER(&messageGuts);
		  break;
	  case CAN_TEMPORARY_ID:
		  doTEMPORARY(&messageGuts);
		  break;
	  case CAN_DEFAULT_ID:
		  // Should NEVER get here!
		  break;
	  default:
		  doCHILD(&messageGuts);
		  break;
	  }
  }
  refresh();

}

void taskCANReceive(void const * argument)
{
	myCANId = GetIdFromFlash();
	setFilters(CAN_FILTER_GLOBAL);


	if (true == IAmInLowFlash())
	{
		flashRate = 2500;
	}
	else
	{
		flashRate = 1000;
	}
	osTimerStart(LEDFlashHandle, flashRate);
	flashMe = true;

	/* Infinite loop */
	for(;;)
	{
	  DoCANProcessing();
	  osThreadYield();
	}
}
