/*
 * UARTHandler.c
 *
 *  Created on: Oct 12, 2018
 *      Author: mdupont
 */


#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include "usart.h"

#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "CharQueue.h"
#include "UARTHandler.h"
#include "CANHandler.h"

extern osTimerId FileTransferHandle;
extern osSemaphoreId UARTContrlHandle;

bool 			trafficOnUART = false;
uint8_t			char2q = 0;
bool 			UART_Raw = false;

#define MY_BUFFER_LENGTH 256

static bool		startSync = false;

bool				txCplt = false;
QUEUE_MGT_STRUCT qStruct;
bool				timerFlag_loadTimer = false;

typedef struct _COMMAND_TABLE_ENTRY
{
	const char *commandString;
	void (*functionPtr)(char *ptr);
	const char *argDescriptionStr;
} COMMAND_TABLE_ENTRY;

void dummy(char *ptr){}
void printHelp(char *);
void myID(char *);
void getAddresses(char *);
void assignAddress(char *);
void flashLED(char *);
void flashLED(char *);
void stateLED(char *);
void stateLED(char *);
void eraseSysBlock(char *);
void eraseProg(char *);
void loadProg(char *);
void resetNode(char *);
void getVersion(char *);

COMMAND_TABLE_ENTRY commandTable[] =
{
		{"?",			printHelp,		"\n"},
		{"HELP",			printHelp,		"\n"},
		{"MYADR",		myID,			"\n"},
		{"GETADRS",		getAddresses,	"\n"},
		{"ASSIGN",		assignAddress,	"\n"},
		{"FLASH",		flashLED,		" <ID>\n"},
		{"FLASH_OFF",	flashLED,		" <ID>\n"},
		{"ON",			stateLED,		" <ID>\n"},
		{"OFF",			stateLED,		" <ID>\n"},
		{"SYS_ERA",		eraseSysBlock,	"\n"},
		{"PROG_ERA",		eraseProg,		" <ID>\n"},
		{"LOAD",			loadProg,		" <ID> <loadBaseAddr>\n"},
		{"RESET",		resetNode,		" <ID>\n"},
		{"VER",			getVersion,		" <ID>\n"},
		{NULL,			dummy,			NULL}
};


void cbFileTransfer(void const * argument)
{
	timerFlag_loadTimer = true;
}

bool StartSync(void)
{
	return(startSync);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	txCplt = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	if (UART_Raw)
	{
		CQ_EnqueueChar(&qStruct, char2q);
		HAL_UART_Receive_IT(UartHandle, &char2q, 1);
		if (true == CQ_AboveWaterMark(&qStruct))
		{
			UartHandle->Instance->TDR = 0x13; // XOFF
		}
	}
	else
	{
		if ('\0' == char2q)
		{
			HAL_UART_Receive_IT(UartHandle, &char2q, 1);
			return;
		}


		trafficOnUART = true;

		CQ_EnqueueChar(&qStruct, char2q);

		if (1 == qStruct.term_count)
		{
			return;
		}
	}

	HAL_UART_Receive_IT(UartHandle, &char2q, 1);
}



void RearmUART(void)
{
	  HAL_UART_Abort_IT(&huart2);
}


void WriteUARTString(char *strPtr)
{
	osSemaphoreWait(UARTContrlHandle, 0);
	while (HAL_UART_STATE_BUSY_TX == (HAL_UART_GetState(&huart2) & HAL_UART_STATE_BUSY_TX))
	{
		taskYIELD();
	}

	txCplt = false;

	if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)strPtr, strlen((const char *)strPtr)) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}


	while (false == txCplt)
	{
	  taskYIELD();
	}
	osSemaphoreRelease(UARTContrlHandle);
}

char GetUARTChar(void)
{
	int loopCount = 0;

	  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&char2q, 1) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  while ((0 == qStruct.char_count) && (10 > loopCount) && ('\0' == char2q))
	  {
		  osDelay(100);
		  loopCount++;
	  }

	  HAL_UART_AbortReceive(&huart2);
	  CQ_Flush(&qStruct);

	  return(char2q);
}

char *GetUARTString(void)
{
	int characterCount;
	char *strPtr = NULL;

	  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&char2q, 1) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  while (0 == qStruct.term_count)
	  {
		  taskYIELD();
	  }

	  characterCount = qStruct.char_count - 1;
	  if (0 != characterCount)
	  {
		  strPtr = pvPortMalloc(characterCount + 1);

		  for(int i = 0; 0 != characterCount; characterCount--, i++)
		  {
			  CQ_DequeueChar(&qStruct,(uint8_t *)&(strPtr[i]));
			  strPtr[i+1] = '\0';
		  }
	  }

	  CQ_Flush(&qStruct);
	  return(strPtr);
}


void UART_ReportReceivedMessage(uint16_t source, uint16_t destination, uint16_t command, uint8_t *rxData)
{
	char *ptr;

	ptr = (char *)pvPortMalloc(128);
	sprintf(ptr, "0x%04X 0x%04X 0x%04X 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			source,
			destination,
			command,
			rxData[0],
			rxData[1],
			rxData[2],
			rxData[3],
			rxData[4],
			rxData[5],
			rxData[6],
			rxData[7]);
	WriteUARTString(ptr);
	vPortFree(ptr);
}

bool getArgument(char *stringPtr, int argNum, char *argPtr, int argBufSize)
{
	int charCount = 0;

	argBufSize--;	// make room for null terminator
	for (int i = 0; i < argNum; i++)
	{
		while(' ' != *stringPtr)
		{
			stringPtr++;
			if ('\0' == *stringPtr)
			{
				return(false);
			}
		}

		while(' ' == *stringPtr)	// isspace() didn't work here!
		{
			stringPtr++;
			if ('\0' == *stringPtr)
			{
				return(false);
			}
		}
	}

	while((false == isspace(*stringPtr)) && ('\0' != *stringPtr))
	{
		*argPtr = *stringPtr;
		argPtr++;
		*argPtr = '\0';

		stringPtr++;
		charCount++;
		if (charCount >= argBufSize)
		{
			return(false);
		}
	}
	return(true);
}

void printHelp(char *ptr)
{
	COMMAND_TABLE_ENTRY *cmdPtr = &commandTable[0];

	while (NULL != cmdPtr->argDescriptionStr)
	{
		WriteUARTString((char *)cmdPtr->commandString);
		WriteUARTString((char *)cmdPtr->argDescriptionStr);
		cmdPtr++;
	}
}

void myID(char *ptr)
{
	char *reportBuffer;
	reportBuffer = (char *)pvPortMalloc(64);
	sprintf(reportBuffer, "My ID is: %08lX\n", CAN_MyID());
	WriteUARTString(reportBuffer);
	vPortFree(reportBuffer);
}

void getAddresses(char *ptr)
{
	CAN_GetAddresses();
}

void assignAddress(char *ptr)
{
	uint32_t addr = 0;
	char argBuffer[10];

	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%lX", &addr);
	CAN_AssignAddress(addr);
}

void flashLED(char *ptr)
{
	int channel;
	bool state = true;
	char argBuffer[10];

	state = (NULL == strstr(ptr, "OFF"));
	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_LEDFlashControl(channel, state);
}

void stateLED(char *ptr)
{
	int channel;
	bool state = true;
	char argBuffer[10];

	state = (NULL == strstr(ptr, "OFF"));
	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_LEDStateControl(channel, state);
}

void eraseSysBlock(char *ptr)
{
	int channel;
	char argBuffer[10];

	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_EraseSysBlock(channel);
}

void eraseProg(char *ptr)
{
	int channel;
	char argBuffer[10];

	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_EraseProgramBlock(channel);
}

void loadProg(char *strPtr)
{
	char argBuffer[10];
	int id;
	uint32_t baseAddress;
	volatile int dwell = 60000;

	if (false == getArgument(strPtr, 1, argBuffer, 10))
	{
		WriteUARTString("\n1) No ID! Aborting.\n");
		return;
	}
	sscanf(argBuffer, "%d", &id);

	if (false == getArgument(strPtr, 2, argBuffer, 10))
	{
		WriteUARTString("\n2) No base address! Aborting.\n");
		return;
	}
	sscanf(argBuffer, "%lX", &baseAddress);

	// FLUSH COM BUFFER so errant terminators don't get sent to the target
	if (true == CAN_ProgramStart(id, baseAddress))
	{
		uint8_t byteIn;
		timerFlag_loadTimer = false;
		osTimerStart(FileTransferHandle,dwell);

		osDelay(100);
		UART_Raw = true;
		if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&char2q, 1) != HAL_OK)
		{
		  _Error_Handler(__FILE__, __LINE__);
		}
		osDelay(100);

		CQ_Flush(&qStruct);

		while(false == timerFlag_loadTimer)
		{
			if (false == CQ_DequeueChar(&qStruct, &byteIn))
			{
				huart2.Instance->TDR = 0x11; //XON
				WriteUARTString(".");
				osDelay(100);
				continue;
			}
			dwell = 500;
			CAN_ProgramChar(byteIn);
			osTimerStart(FileTransferHandle,dwell);
		}

		CAN_ProgramClose();
		UART_Raw = false;
		WriteUARTString("\nLoad CLOSED\n");
	}
	else
	{
		WriteUARTString("\nAbort - no load\n");
	}
}

void resetNode(char *ptr)
{
	int channel;
	char argBuffer[10];

	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_RestartNode(channel);
}

void getVersion(char *ptr)
{
	int channel;
	char argBuffer[10];

	if (false == getArgument(ptr, 1, argBuffer, 10))
	{
		return;
	}
	sscanf(argBuffer, "%d", &channel);

	CAN_GetReportVersion(channel);

}
void DoUARTCommand(char *strPtr)
{
	char argBuffer[20];
	COMMAND_TABLE_ENTRY *cmdTablePtr = &commandTable[0];

	WriteUARTString(strPtr);

	WriteUARTString("\r\n>");

	if (false == getArgument(strPtr, 0, argBuffer, sizeof(argBuffer) - 1))
	{
		return;
	}

	while (NULL != cmdTablePtr->commandString)
	{
		if (0 == strcmp(cmdTablePtr->commandString, argBuffer))
		{
			(cmdTablePtr->functionPtr)(strPtr);
			return;
		}
		cmdTablePtr++;
	}
}

void MessagesUART_Init(void)
{
	char *pBuffer;

	pBuffer = (char *)pvPortMalloc(MY_BUFFER_LENGTH);
	if (NULL == pBuffer)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	CQ_Init(&qStruct, (uint8_t *)pBuffer, MY_BUFFER_LENGTH);
}

void UART_Start(void)
{
	static bool initialized = false;

	if (true == initialized)
	{
		return;
	}

	WriteUARTString("\x11");

	// Do I already have an ID?
	//	YES:
	//		Am I already MASTER?
	//			YES - Return to allow CLI to work
	//			NO - Delete this task -- not needed as a child node
	//	NO:
	//		Poll UART to see if a host is attached AND
	//		Poll "IDENT" switch to see if a user is
	//		trying to "commission" this node
	//			Did a response appear at the UART?
	//				YES: THIS IS THE MASTER - assign ID of 0x001
	//					Return to allow CLI to work
	//				NO: Keep trying
	//			Did the "IDENT" switch get hit?
	//				YES: Notify CAN side that it needs to ask for an ID.
	//					Delete this task -- not needed as a child node
	//				NO: Keep trying.
	//
	if (true == CAN_IAmMaster())
	{
		initialized = true;
		WriteUARTString("MASTER AVAILABLE\n");
		return;
	}
	if (true == CAN_IAmAssignedChild())
	{
		osThreadTerminate(osThreadGetId());
	}
}

void	 DoUARTProcessing(void)
{
	char *rxStrPtr;

	UART_Start();

	if (false == trafficOnUART)
	{
		GetUARTChar();
		return;
	}

	if (false == CAN_IAmMaster())
	{
		// A host is connected to this node.  This is the MASTER.
		// Claim it and move on.
		CAN_ClaimMaster();
	}

	rxStrPtr = GetUARTString();

	if (NULL != rxStrPtr)
	{
		DoUARTCommand(rxStrPtr);
		vPortFree(rxStrPtr);
		rxStrPtr = NULL;
	}

	RearmUART();
}

void taskUARTReceive(void const * argument)
{
	MessagesUART_Init();
	RearmUART();

	/* Infinite loop */
	for(;;)
	{
	  startSync = true;
	  DoUARTProcessing();

	  osThreadYield();
	}
}

