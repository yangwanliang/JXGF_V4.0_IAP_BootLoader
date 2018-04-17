/****************************************Copyright (c)****************************************************
**                            Guangzhou ZLGMCU Development Co., LTD
**
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           crc.c
** Last modified Date:  2011-12-23
** Last Version:        V1.0
** Descriptions:        The u16CRC_Calc16() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Liu zhipeng
** Created date:        2011-12-23
** Version:             V1.00
** Descriptions:        整理应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         
** Modified date:       
** Version:             
** Descriptions:        
**
**--------------------------------------------------------------------------------------------------------
** Modified by:        
** Modified date:      
** Version:            
** Descriptions:       
**
** Rechecked by:
*********************************************************************************************************/
#include "..\lpc177x_8x\lpc177x_8x.h"
#include "uart.h"
#include <string.h>
#include "..\XMODEM1K\xmodem1k.h"
/* UART line status register (LSR) bit definitions */
#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

/*****************************************************************************
** Function name:	vUARTInit
**
** Descriptions:	Initialize UART0 port, setup pin select, clock, parity,
**                  stop bits, FIFO, etc.
**
** Parameters:		u32BaudRate - UART baudrate
**
** Returned value:	None
**
*****************************************************************************/
void vUARTInit(uint32_t u32BaudRate)
{
	uint32_t Fdiv;
	uint32_t regVal;

	/* Not using interrupts */
	NVIC_DisableIRQ(UART0_IRQn);

	/* UART I/O config */
    LPC_SC->PCONP |= 0x02000000;
	LPC_IOCON->P4_28 &= ~0x07;
	LPC_IOCON->P4_28 |= 2;                   /* P4.28 U3_TXD */
	LPC_IOCON->P4_29 &= ~0x07;
	LPC_IOCON->P4_29 |= 2;                   /* P4.29 U3_RXD*/


	LPC_UART3->LCR = 0x83;             /* 8 bits, no Parity, 1 Stop bit */
	Fdiv = (PeripheralClock/16)/u32BaudRate ;	/*baud rate */

	LPC_UART3->DLM = Fdiv / 256;
	LPC_UART3->DLL = Fdiv % 256;
	LPC_UART3->LCR = 0x03;		/* DLAB = 0 */
	LPC_UART3->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	/* Read to clear the line status. */
	regVal = LPC_UART3->LSR;

	/* Ensure a clean start, no data in either TX or RX FIFO. */
	while ( (LPC_UART3->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );

	while ( LPC_UART3->LSR & LSR_RDR )
	{
		regVal = LPC_UART3->RBR;	/* Dump data from RX FIFO */
	}
	regVal=regVal;
}

/*****************************************************************************
** Function name:	vUARTReceive
**
** Descriptions:	Reads received data from UART0 FIFO
**
** Parameters:		pu8Buffer - Pointer to buffer in which received characters
** 					are to be stored.
**
** Returned value:	Number of character read out of receive FIFO.
**
*****************************************************************************/
uint8_t u8UARTReceive(uint8_t *pu8Buffer)
{
	uint8_t u8Len = 0;

	if (LPC_UART3->LSR & LSR_RDR)
	{
		*pu8Buffer = LPC_UART3->RBR;
		u8Len++;
	}
	return u8Len;
}

/*****************************************************************************
** Function name:	vUARTSend
**
** Descriptions:	Send a block of data to the UART0.
**
** parameters:		pu8Buffer - Pointer to buffer containing data to be sent.
** 					u32Len - Number of bytes to send.
**
** Returned value:	None
**
*****************************************************************************/
void vUARTSend(uint8_t *pu8Buffer, uint32_t u32Len)
{
    if(u32Len==0)
    {
        u32Len=strlen((const char *)pu8Buffer);
    }
	while ( u32Len != 0 )
	{
		/* Send character to UART */
		LPC_UART3->THR = *pu8Buffer;
		/* Wait until transmission is complete */
		while ((LPC_UART3->LSR & LSR_TEMT) == 0);
		pu8Buffer++;
		u32Len--;
	}
}

void Printf(char *fmt,...)
{
    va_list ap;
    uint8_t str[100]={0};
    va_start(ap,fmt);
    vsprintf((char *)str,fmt,ap);        
	vUARTSend(str,strlen((char *)str));
	va_end(ap);
}


/******************************************************************************
**                            End Of File
******************************************************************************/
