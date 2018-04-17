/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           uart.h
** Last modified Date:  2011-05-05
** Last Version:        V1.00
** Descriptions:        Xmodem1K协议——底层串口传输
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Lanwuqiang
** Created date:        2011-05-05
** Version:             V1.00
** Descriptions:        Xmodem1K协议——底层串口传输
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         
** Modified date:       
** Version:
** Descriptions:        
**
** Rechecked by:
*********************************************************************************************************/
#ifndef __UART_H 
#define __UART_H

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* Baud rate to be used by UART interface */
#define BAUD_RATE					38400

/*********************************************************************************************************
** Function name:	vUARTInit
**
** Descriptions:	Initialize UART0 port, setup pin select, clock, parity,
**                  stop bits, FIFO, etc.
**
** Parameters:		u32BaudRate - UART baudrate
**
** Returned value:	None
**
*********************************************************************************************************/
extern void vUARTInit(uint32_t u32BaudRate);

/*********************************************************************************************************
** Function name:	vUARTReceive
**
** Descriptions:	Reads received data from UART0 FIFO
**
** Parameters:		pu8Buffer - Pointer to buffer in which received characters
** 					are to be stored.
**
** Returned value:	Number of character read out of receive FIFO.
**
**********************************************************************************************************/
extern uint8_t u8UARTReceive(uint8_t *pu8Buffer);

/*********************************************************************************************************
** Function name:	vUARTSend
**
** Descriptions:	Send a block of data to the UART0.
**
** parameters:		pu8Buffer - Pointer to buffer containing data to be sent.
** 					u32Len - Number of bytes to send.
**
** Returned value:	None
**
**********************************************************************************************************/
extern void vUARTSend(uint8_t *pu8Buffer, uint32_t u32Len);
extern void Printf(char *fmt,...);

#endif /* end __UART_H */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
