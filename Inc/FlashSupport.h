/*
 * FlashSupport.h
 *
 *  Created on: Oct 23, 2018
 *      Author: mdupont
 */

#ifndef FLASHSUPPORT_H_
#define FLASHSUPPORT_H_

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdbool.h>

extern const char *versionString;
extern const uint8_t versionCode[];

int GetLoadId(void);
void SetLoadId(int id);
uint32_t GetLoadBase(void);
void SetLoadBase(uint32_t addr);
int GetLoadIndex(void);
uint8_t *GetPacket(void);
uint32_t GetMyLocationInFlash(void);
uint16_t GetIdFromFlash(void);
void ProgramIdIntoFlash(uint32_t id);

void InvalidateProgram(void);
void ValidateProgram(void);

void EraseSystemBlock(void);
void EraseProgramBlock(void);

void RestartNode(void);

bool IAmInLowFlash(void);
bool ValidProgramInHighFlash(void);
bool UpperBlockIsEmpty(void);
void JumpToHighFlash(void);

bool DidLoadOccur(void);
void LoadSetup(int id, uint32_t baseAddr);
uint32_t WriteToFlashBuffer(uint8_t ch, bool writeMyFlash);
uint32_t FlushFlashBuffer(void);

void ReportFlash(void);

#endif /* FLASHSUPPORT_H_ */
