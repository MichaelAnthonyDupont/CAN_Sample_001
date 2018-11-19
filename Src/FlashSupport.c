/*
 * FlashSupport.c
 *
 *  Created on: Oct 23, 2018
 *      Author: mdupont
 */

#include "UARTHandler.h"
#include "FlashSupport.h"

#define VALID_PROGRAM_SIGNATURE ((uint32_t)0xAA55CC33)


// Bootloader Stuff:
typedef union _BYTE_PACKET
{
	uint8_t array[8];
	uint64_t value;
} BYTE_PACKET;

BYTE_PACKET			bytePacket;
int32_t				loadIndex = 0;
int					loadId = -1;
uint8_t				*loadPtr;
uint8_t				*loadBasePtr;
bool					loadStartOk = false;


typedef  void (*pFunction)(void);
pFunction			JumpToApplication;
volatile uint32_t 	JumpAddress;
volatile uint32_t 	BaseOfUpperExec;
volatile uint32_t	BaseOfLowerExec;
volatile uint32_t	StackVal;

static const char 	*locationString = "I'M HERE!";
#ifdef _MY_RELOCATED_RELEASE
const char *versionString = "000.000.001.144";	//Format: MAJ.MIN.BUILD.TIME  (HHM -- Hours in 24 hour time; M -- 10 minute increment
const uint8_t versionCode[] = {0,0,1,140};
#else
const char *versionString = "018.011.006.144";
const uint8_t versionCode[] = {18,11,5,140};
#endif


int GetLoadId(void)
{
	return(loadId);
}
void SetLoadId(int id)
{
	loadId = id;
}

void SetLoadBase(uint32_t addr)
{
	loadBasePtr = (uint8_t *)addr;
}

uint32_t GetLoadBase(void)
{
	return((uint32_t) loadBasePtr);
}

int GetLoadIndex(void)
{
	return(loadIndex);
}

uint32_t GetMyLocationInFlash(void)
{
	return((uint32_t)locationString);
}

uint8_t *GetPacket(void)
{
	return(bytePacket.array);
}

bool DidLoadOccur(void)
{
	return((loadBasePtr != loadPtr) && (loadBasePtr < loadPtr));
}

void LoadSetup(int id, uint32_t baseAddr)
{
	// reset load variables
	loadIndex = 0;
	SetLoadId(id);
	SetLoadBase(baseAddr);
	loadPtr = (uint8_t *)GetLoadBase();
}

bool IAmInLowFlash(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);

	uint32_t *relocBlockBase = (uint32_t *)(FLASH_BASE + APPLICATION_OFFSET);

	return(FLASH_BASE == (uint32_t)relocBlockBase);
}

bool UpperBlockIsEmpty(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);

	uint32_t *baseOfSysBlock = (uint32_t *)(FLASH_BASE + ((flashBlocks-1) * 1024));
	uint32_t *baseOfUpperExec = (uint32_t *)(RELO_APP_BASE);
	uint32_t *baseOfValidationSignature = baseOfSysBlock + 1;


	if (0xFFFFFFFF != *baseOfValidationSignature)
	{
		return(false);
	}

	while (baseOfUpperExec < baseOfSysBlock)
	{
		for (int i = 0; i < 256; i++)
		{
			if (0xFFFFFFFF != baseOfUpperExec[i])
			{
				return(false);
			}
		}
		baseOfUpperExec += 256;
	}
	return (true);

}

bool ValidProgramInHighFlash(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);

	uint32_t *baseOfSysBlock = (uint32_t *)(FLASH_BASE + ((flashBlocks-1) * 1024));
	uint32_t *baseOfUpperExec = (uint32_t *)(RELO_APP_BASE);

	baseOfSysBlock++;

	if (VALID_PROGRAM_SIGNATURE != *baseOfSysBlock)
	{
		return(false);
	}

	for (int i = 0; i < 256; i++)
	{
		if (0xFFFFFFFF != baseOfUpperExec[i])
		{
			return(true);
		}
	}
	return (false);
}

void JumpToHighFlash(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);

	uint8_t *baseOfUpperExec = (uint8_t *)(RELO_APP_BASE);
	BaseOfUpperExec = (uint32_t *)baseOfUpperExec;
    JumpAddress = *(volatile unsigned long*) (baseOfUpperExec + 4);

	__disable_irq();
	__set_CONTROL(0); // Assure privileged operation
	__set_MSP(*(__IO uint32_t*) BaseOfUpperExec);

     JumpToApplication = (pFunction) JumpAddress;
     JumpToApplication();
     while(1);

}

void RestartNode(void)
{
	uint8_t *baseOfLowerExec = (uint8_t *)(FLASH_BASE);
	BaseOfLowerExec = (uint32_t *)baseOfLowerExec;
    JumpAddress = *(volatile unsigned long*) (baseOfLowerExec + 4);

	portDISABLE_INTERRUPTS();

	__disable_irq();
	__set_CONTROL(0); // Assure privileged operation
	__set_MSP(*(__IO uint32_t*) BaseOfLowerExec);

     JumpToApplication = (pFunction) JumpAddress;
     JumpToApplication();
     while(1);

}

uint16_t GetIdFromFlash(void)
{
	uint16_t myCANId;

	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	uint32_t *sysBlockBase = (uint32_t *)(FLASH_BASE + ((flashBlocks-1) * 1024));

	myCANId = ((*sysBlockBase) & 0x1FF);
	return(myCANId);
}

void ProgramIdIntoFlash(uint32_t id)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	uint32_t *sysBlockBase = (uint32_t *)(FLASH_BASE + ((flashBlocks-1) * 1024));

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, sysBlockBase, id);
	HAL_FLASH_Lock();
}

void EraseSystemBlock(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	uint32_t *sysBlockBase = (uint32_t *)(FLASH_BASE + (flashBlocks *1024) - FLASH_PAGE_SIZE);

	HAL_FLASH_Unlock();
	FLASH_PageErase(sysBlockBase);
    FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    CLEAR_BIT (FLASH->CR, (FLASH_CR_PER));  // HOLY SHIT:  https://stackoverflow.com/questions/28498191/cant-write-to-flash-memory-after-erase
	HAL_FLASH_Lock();

}

void EraseProgramBlock(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	uint32_t *sysBlockBase = (uint32_t *)(FLASH_BASE + (flashBlocks * 1024) - FLASH_PAGE_SIZE);
	uint32_t *baseOfUpperExec = (uint32_t *)(RELO_APP_BASE);

	while(baseOfUpperExec < sysBlockBase)
	{
		HAL_FLASH_Unlock();
		FLASH_PageErase(baseOfUpperExec);
	    FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
	    CLEAR_BIT (FLASH->CR, (FLASH_CR_PER));  // HOLY SHIT:  https://stackoverflow.com/questions/28498191/cant-write-to-flash-memory-after-erase
		HAL_FLASH_Lock();
		baseOfUpperExec += 256;
		baseOfUpperExec += 256;
	}
}

void InvalidateProgram(void)
{
	int myID = GetIdFromFlash();
	EraseSystemBlock();
	ProgramIdIntoFlash(myID);
}

void ValidateProgram(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	uint32_t *sysBlockBase = (uint32_t *)(FLASH_BASE + ((flashBlocks-1) * 1024));
	sysBlockBase += 1;

	InvalidateProgram();

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, sysBlockBase, VALID_PROGRAM_SIGNATURE);
	HAL_FLASH_Lock();
}


uint32_t WriteToFlashBuffer(uint8_t ch, bool writeMyFlash)
{
    uint32_t res = HAL_OK;

    if (0 == loadIndex)
    {
    		bytePacket.value = 0xFFFFFFFFFFFFFFFF;
    }

    bytePacket.array[loadIndex] = ch;
	loadIndex++;
	loadIndex %= 8;

	if ((0 == loadIndex) && (true == writeMyFlash))
	{
		// write to FLASH
		HAL_FLASH_Unlock();
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)loadPtr, bytePacket.value);
		HAL_FLASH_Lock();
		loadPtr += 8;
	}
	return(res);
}

uint32_t FlushFlashBuffer(void)
{
    uint32_t res = HAL_OK;

    HAL_FLASH_Unlock();
	res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)loadPtr, bytePacket.value);
	HAL_FLASH_Lock();

	return(res);
}


void ReportFlash(void)
{
	uint16_t flashBlocks = *((uint16_t *)FLASHSIZE_BASE);
	static bool report = false;
	char *ptr;

	if (true == report)
	{
		return;
	}

	ptr = (char *)pvPortMalloc(64);
	sprintf(ptr, "MemStats: 0x%08LX 0x%04X 0x%08LX 0x%08LX\n",
			FLASH_BASE,
			flashBlocks,
			FLASH_BASE + (flashBlocks*1024) - 1,
			FLASH_BASE + ((flashBlocks-1) * 1024));
	WriteUARTString(ptr);
	sprintf(ptr, "%s memory\n", (IAmInLowFlash() ? "LOW" : "HIGH"));
	WriteUARTString(ptr);
	vPortFree(ptr);
	report = true;
}
