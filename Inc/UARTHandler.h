/*
 * UARTHandler.h
 *
 *  Created on: Oct 12, 2018
 *      Author: mdupont
 */

#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

#include <stdbool.h>

bool StartSync(void);
void WriteUARTString(char *strPtr);
void UART_ReportReceivedMessage(uint16_t source, uint16_t destination, uint16_t command, uint8_t *rxData);

extern void cbFileTransfer(void const * argument);
extern void taskUARTReceive(void const * argument);

#endif /* UARTHANDLER_H_ */
