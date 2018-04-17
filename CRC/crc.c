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
#include "crc.h"

/*****************************************************************************
** Function name:	u16CRC_Calc16
**
** Descriptions:	Calculate 16-bit CRC value used by Xmodem-1K protocol.
**
** Parameters:	    pu8Data - Pointer to buffer containing 8-bit data values.
** 					i16Len - Number of 8-bit values to be included in calculation.
**
** Returned value:  16-bit CRC
**
******************************************************************************/
uint16_t u16CRC_Calc16(const uint8_t *pu8Data, int32_t i32Len)
{
	uint8_t i;
	uint16_t u16CRC = 0;

    while(--i32Len >= 0)
    {
    	i = 8;
    	u16CRC = u16CRC ^ (((uint16_t)*pu8Data++) << 8);

    	do
        {
    		if (u16CRC & 0x8000)
    		{
    			u16CRC = u16CRC << 1 ^ 0x1021;
    		}
    		else
    		{
    			u16CRC = u16CRC << 1;
    		}
        }
    	while(--i);
    }
    return u16CRC;
}

/*****************************************************************************
**                            End Of File
******************************************************************************/
