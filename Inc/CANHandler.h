/*
 * CANHandler.h
 *
 *  Created on: Oct 12, 2018
 *      Author: mdupont
 */

#ifndef CANHANDLER_H_
#define CANHANDLER_H_

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdbool.h>

uint32_t CAN_MyID(void);

bool CAN_IAmMaster(void);
bool CAN_IAmAssignedChild(void);
void CAN_ClaimMaster(void);

bool CAN_GetAddresses(void);
bool CAN_AssignAddress(uint32_t addr);
bool CAN_LEDFlashControl(uint32_t addr, bool state);
bool CAN_LEDStateControl(uint32_t addr, bool state);

bool CAN_EraseSysBlock(uint32_t addr);
bool CAN_EraseProgramBlock(uint32_t addr);
bool CAN_ProgramStart(int id, uint32_t baseAddr);
bool CAN_ProgramChar(uint8_t ch);
bool CAN_ProgramClose(void);
bool CAN_RestartNode(int id);
bool CAN_GetReportVersion(int id);


void cbCANLoadError(void const * argument);
void cbLEDFlash(void const * argument);
void taskCANReceive(void const * argument);




#endif /* CANHANDLER_H_ */
