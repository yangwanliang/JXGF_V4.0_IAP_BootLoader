/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2009-05-12
** Last Version:        V1.01
** Descriptions:        The main() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Chengmingji
** Created date:        2011-07-24
** Version:             V1.00
** Descriptions:        添加用户应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         liweifeng
** Modified date:       2011-11-11
** Version:                V1.01 
** Descriptions:        编写IAP在线升级工程
**
** Rechecked by:        
*********************************************************************************************************/
#include "..\lpc177x_8x\lpc177x_8x.h"
#include "../UART/uart.h"
#include "../IAP/iap.h"
#include "../CRC/crc.h"
#include "../XMODEM1K/xmodem1k.h"
#include <string.h>
#include <stdio.h>

/* 
 * Define flash memory address at which user application is located 
 */
#define APP_START_ADDR						0x00010000UL
#define APP_END_ADDR						0x00080000UL                /* LPC1788 512K Flash           */
#define VCTRL_HIGH              (1ul << 22)//p3.22
#define VCTRL_HIGH_OPEN()       LPC_GPIO3->DIR |= VCTRL_HIGH;LPC_GPIO3->CLR = VCTRL_HIGH
#define VCTRL_HIGH_CLOSE()      LPC_GPIO3->DIR |= VCTRL_HIGH;LPC_GPIO3->SET = VCTRL_HIGH

#define VCTRL_MID               (1ul << 21)//p3.21
#define VCTRL_MID_OPEN()        LPC_GPIO3->DIR |= VCTRL_MID;LPC_GPIO3->CLR = VCTRL_MID
#define VCTRL_MID_CLOSE()       LPC_GPIO3->DIR |= VCTRL_MID;LPC_GPIO3->SET = VCTRL_MID

#define VCTRL_LOW               (1ul << 20)//p3.20
#define VCTRL_LOW_OPEN()        LPC_GPIO3->DIR |= VCTRL_LOW;LPC_GPIO3->CLR = VCTRL_LOW
#define VCTRL_LOW_CLOSE()       LPC_GPIO3->DIR |= VCTRL_LOW;LPC_GPIO3->SET = VCTRL_LOW

#define VCTRL_KEY               (1ul << 23)//p3.23
#define VCTRL_OPENKEY()         LPC_GPIO3->DIR |= VCTRL_KEY;LPC_GPIO3->CLR = VCTRL_KEY
#define VCTRL_CLOSEKEY()	    LPC_GPIO3->DIR |= VCTRL_KEY;LPC_GPIO3->SET = VCTRL_KEY

/* 
 * Define the flash sectors used by the application 
 */
#define APP_START_SECTOR					16
#define APP_END_SECTOR						29		                    /* LPC1788 512K Flash           */

volatile uint32_t IAPFlagTimeOut;

static uint32_t u32InvalidateApp(void);
void ExceuteApplication(void);
static void vBootLoader_Task(void);
static uint32_t u32BootLoader_AppPresent(void);
static uint32_t u32Bootloader_WriteCRC(uint16_t u16CRC);
static uint32_t u32BootLoader_ProgramFlash(uint8_t *pu8Data, uint16_t u16Len);

/*********************************************************************************************************
** Function name:       Timer0_Init
** Descriptions:        定时器0初始化程序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void  Timer0_Init (uint32_t ulsec)
{
    LPC_TIM0->TCR  = 0x02;
    LPC_TIM0->IR   = 0x01;
    LPC_TIM0->CTCR = 0;
    LPC_TIM0->TC   = 0;
    LPC_TIM0->PR   = 0;
    LPC_TIM0->MR0  = PeripheralClock * ulsec;                           /* nS中断1次                    */
    LPC_TIM0->MCR  = 0x05;                                              /* 匹配后产生中断               */
    LPC_TIM0->TCR  = 0x01;                                              /* 启动定时器                   */
}


/*********************************************************************************************************
** Function name:       main
** Descriptions:        Bootloader控制循环
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
int main(void)
{
	uint8_t uartBuffer[16], len;

	SystemInit();
	WDDIR();
    VCTRL_OPENKEY();
    VCTRL_MID_OPEN();
    VCTRL_HIGH_CLOSE();    
    vUARTInit(BAUD_RATE); 
    vUARTSend((uint8_t *)("\r\nJXGF_V4.0 Ready For Updates >"),0);    
    Timer0_Init (10);
    WATCH_DOG();
    len = 0;
    while (1) {
        if ((LPC_TIM0->IR & 0x01) == 0x01) {
             LPC_TIM0->IR   = 0x01;                                     /* 清除中断标志                 */
             IAPFlagTimeOut = 1;										/* 等待升级命令超时             */
             vUARTSend((uint8_t *)("\r\nTimeout waiting for the upgrade command!"),0);
             break;
        }
        if (u8UARTReceive(uartBuffer) > 0)
        {                               /* 判断升级命令 IAP             */

            if ((uartBuffer[0] == 'I')||(uartBuffer[0] == 'i')) {
                len = 1;
            }
            if ((uartBuffer[0] == 'A')||(uartBuffer[0] == 'a')) {
                if (len == 1) {
                    len = 2;
                } else {
                    len = 0;
                }
            }
            if ((uartBuffer[0] == 'P')||(uartBuffer[0] == 'p')) {
                if (len == 2) {
                    len = 3;
                } else {
                    len = 0;
                }
            }
			if ((uartBuffer[0] == 'Q')||(uartBuffer[0] == 'q')) {
                IAPFlagTimeOut=1;
				break;				
            }
        }
        if (len == 3) 
        {
                IAPFlagTimeOut = 0;
                vUARTSend((uint8_t *)("\r\nReceiving a successful upgrade command!"), 0);
                break;

        }
		WATCH_DOG();
    }

    if (IAPFlagTimeOut == 0) 
    {
        if (u32InvalidateApp() == 0)
	    {
		    vUARTSend((uint8_t *)("\r\nErase mark CRC failed!"),0);
            if ((SelSector(APP_END_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
            && (EraseSector(APP_END_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)) {
                vUARTSend((uint8_t *)("\r\nErase the last sector!"),0);
            }
            /* Restart and bootloader will begin */
		    SCB->AIRCR = (0x05fa << 16) + 1;
	    }
    }

	/* Verify if a valid user application is present in the upper sectors
	   of flash memory. */
	if (u32BootLoader_AppPresent() == 0)
	{
        vUARTSend((uint8_t *)("\r\nInto upgrade state!"),0);
        /* Valid application not present, execute bootloader task that will
		   obtain a new application and program it to flash.. */
		vBootLoader_Task();

		/* Above function only returns when new application image has been
		   successfully programmed into flash. Begin execution of this new
		   application by resetting the device. */
		if (u32BootLoader_AppPresent() != 0) {
		    vUARTSend((uint8_t *)("\r\nExecute user code!"),0);
            WATCH_DOG();
		    SCB->VTOR  = APP_START_ADDR;                                /* 重新映射向量表               */
            ExceuteApplication();
		} else {
		    vUARTSend((uint8_t *)("\r\nUpgrade failure!"),0);
		    SCB->AIRCR = (0x05fa << 16) + 1;
		}
	    
	}
	else
	{
		vUARTSend((uint8_t *)("\r\nExecute user code!"),0);
        /* Valid application located in the next sector(s) of flash so execute */
        WATCH_DOG();
		SCB->VTOR  = APP_START_ADDR;                                    /* 重新映射向量表               */
        ExceuteApplication();
											  
		/* User application execution should now start and never return here.... */
	}
	/* This should never be executed.. */
	return 0;
}

__asm void ExceuteApplication(void)
{
		/* Load main stack pointer with application stack pointer initial value,
		   stored at first location of application area */
		ldr r0, =0x10000
		ldr r0, [r0]
		mov sp, r0

		/* Load program counter with application reset vector address, located at
		   second word of application area. */
		ldr r0, =0x10004
		ldr r0, [r0]
        BX  r0
		NOP
}

/*****************************************************************************
 ** Function name:  vBootLoader_Task
 **
 ** Description:	Erases application flash area and starts XMODEM client so
 ** 				that new application can be downloaded.
 **
 ** Parameters:	    None
 **
 ** Returned value: None
 **
 *****************************************************************************/
static void vBootLoader_Task(void)
{	
	/* Erase the application flash area so it is ready to be reprogrammed with the new application */
	vUARTSend((uint8_t *)("\r\nErasing board flash..."), 0);
	if (SelSector(APP_START_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
	{
		if (EraseSector(APP_START_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
		{
			uint16_t u16CRC = 0;
			vUARTSend((uint8_t *)("\r\nFlash erase success!"),0);
			/* Start the xmodem client, this function only returns when a
			   transfer is complete. Pass it pointer to function that will
			   handle received data packets */
			vXmodem1k_Client(&u32BootLoader_ProgramFlash);			
			/* Programming is now complete, calculate the CRC of the flash image */
			u16CRC = u16CRC_Calc16((const uint8_t *)APP_START_ADDR, (APP_END_ADDR - APP_START_ADDR - 4));

			/* Write the CRC value into the last 16-bit location of flash, this
			   will be used to check for a valid application at startup  */
			(void)u32Bootloader_WriteCRC(u16CRC);
			vUARTSend((uint8_t *)("\r\nProgram finished!"),0);
		}
		else
		{
			while(1);
		}
	}
}

/*****************************************************************************
 ** Function name:	u32Bootloader_WriteCRC
 **
 ** Description:	Writes a 16-bit CRC value to the last location in flash
 ** 				memory, the bootloader uses this value to check for a valid
 ** 				application at startup.
 **
 ** Parameters:	    u16CRC - CRC value to be written to flash
 **
 ** Returned value: 1 if CRC written to flash successfully, otherwise 0.
 **
 *****************************************************************************/
static uint32_t u32Bootloader_WriteCRC(uint16_t u16CRC)
{
	uint32_t i;
	uint32_t u32Result = 0;
	uint32_t a32DummyData[IAP_FLASH_PAGE_SIZE_WORDS];
	uint32_t *pu32Mem = (uint32_t *)(APP_END_ADDR - IAP_FLASH_PAGE_SIZE_BYTES);

	/* First copy the data that is currently present in the last page of
	   flash into a temporary buffer */
	for (i = 0 ; i < IAP_FLASH_PAGE_SIZE_WORDS; i++)
	{
		a32DummyData[i] = *pu32Mem++;
	}

	/* Set the CRC value to be written back */
	a32DummyData[IAP_FLASH_PAGE_SIZE_WORDS - 1] = (uint32_t)u16CRC;

	if (SelSector(APP_END_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
	{
		/* Now write the data back, only the CRC bits have changed */
		if (RamToFlash((APP_END_ADDR - IAP_FLASH_PAGE_SIZE_BYTES),
				                  (uint32_t)a32DummyData,
				                  IAP_FLASH_PAGE_SIZE_BYTES) == CMD_SUCCESS)
		{
			u32Result = 1;
		}
	}
	return (u32Result);
}

/*****************************************************************************
 ** Function name:	u32BootLoader_ProgramFlash
 **
 ** Description:
 **
 ** Parameters:	    None
 **
 ** Returned value: 0 if programming failed, otherwise 1.
 **
 *****************************************************************************/
static uint32_t u32BootLoader_ProgramFlash(uint8_t *pu8Data, uint16_t u16Len)
{
	uint32_t u32Result = 0;

	static uint32_t u32NextFlashWriteAddr = APP_START_ADDR;

	if ((pu8Data != 0) && (u16Len != 0))
	{
		/* Prepare the flash application sectors for reprogramming */
		if (SelSector(APP_START_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
		{
			/* Ensure that amount of data written to flash is at minimum the
			   size of a flash page */
			if (u16Len < IAP_FLASH_PAGE_SIZE_BYTES)
			{
				u16Len = IAP_FLASH_PAGE_SIZE_BYTES;
			}

			/* Write the data to flash */
			if (RamToFlash(u32NextFlashWriteAddr, (uint32_t)pu8Data, u16Len) == CMD_SUCCESS)
			{
				/* Check that the write was successful */
				if (Compare(u32NextFlashWriteAddr, (uint32_t)pu8Data, u16Len) == CMD_SUCCESS)
				{
					/* Write was successful */
					u32NextFlashWriteAddr += u16Len;
					u32Result = 1;
				}
			}
		}
	}
	return (u32Result);
}

/*****************************************************************************
 ** Function name:  u32BootLoader_AppPresent
 **
 ** Description:	Checks if an application is present by comparing CRC of
 ** 				flash contents with value present at last location in flash.
 **
 ** Parameters:	    None
 **
 ** Returned value: 1 if application present, otherwise 0.
 **
 *****************************************************************************/
static uint32_t u32BootLoader_AppPresent(void)
{
	uint16_t u16CRC = 0;
	uint32_t u32AppPresent = 0;
	uint16_t *pu16AppCRC = (uint16_t *)(APP_END_ADDR - 4);

	/* Check if a CRC value is present in application flash area */
	if (*pu16AppCRC != 0xFFFFUL)
	{
		/* Memory occupied by application CRC is not blank so calculate CRC of
		   image in application area of flash memory, and check against this
		   CRC.. */
		u16CRC = u16CRC_Calc16((const uint8_t *)APP_START_ADDR, (APP_END_ADDR - APP_START_ADDR - 4));

		if (*pu16AppCRC == u16CRC)
		{
			u32AppPresent = 1;
		}
	}
	return u32AppPresent;
}

/*****************************************************************************
 ** Function name:  u32InvalidateApp
 **
 ** Description:	Corrupt the application CRC value that is written into the
 ** 				last location of flash by the bootloader.
 **
 ** Parameters:	    None
 **
 ** Returned value: Zero if unsuccessful, otherwise 1
 **
 *****************************************************************************/
uint32_t u32InvalidateApp(void)
{
	uint32_t i;
	uint32_t u32Result = 0;
	uint32_t a32DummyData[IAP_FLASH_PAGE_SIZE_WORDS];
	uint32_t *pu32Mem = (uint32_t *)(APP_END_ADDR - IAP_FLASH_PAGE_SIZE_BYTES);

	/* First copy the data that is currently present in the last page of
	   flash into a temporary buffer */
	for (i = 0 ; i < IAP_FLASH_PAGE_SIZE_WORDS; i++)
	{
		a32DummyData[i] = *pu32Mem++;
	}
 
	/* Set the CRC value to be written back, corrupt by setting all ones to zeros */
	a32DummyData[IAP_FLASH_PAGE_SIZE_WORDS - 1] = 0x0000;

	if (SelSector(APP_END_SECTOR, APP_END_SECTOR) == CMD_SUCCESS)
	{
		/* Now write the data back, only the CRC bits have changed */
		if (RamToFlash((APP_END_ADDR - IAP_FLASH_PAGE_SIZE_BYTES),
				                  (uint32_t)a32DummyData,
				                  IAP_FLASH_PAGE_SIZE_BYTES) == CMD_SUCCESS)
		{
			u32Result = 1;
		}
	}
	return (u32Result);
}


void myprintf (unsigned long temp)
{
    return;
}

void HardFault_Handler ( void )
{
    unsigned long  uiFaultADDR = 0;
    unsigned long  uiFaultCFSR = 0;
    unsigned long *puiSrcAddr  = (unsigned long*)__get_MSP();
    /***************************************************************
    ** 进入异常后依次将xPSR,PC,LR,R12,R3,R2,R1,R0压栈
    ** 因此进入异常前的LR和PC分别存储在MSP中的第6个和第7个字的位置
    ***************************************************************/
    unsigned long  uiLRValue  = puiSrcAddr[5 + 0];
    unsigned long  uiPCValue  = puiSrcAddr[6 + 0];
    unsigned long  uiRtnAddr  = uiLRValue & 0xfffffffe;             /* LR寄存器的最低位不代表地址                   */

    if(SCB->HFSR & (1<<1)) {
        SCB->HFSR = SCB->HFSR;
        while(1);                                                   /* 硬fault 因调试事件而产生                     */
    } else if (SCB->HFSR & 0x80000000) {                          
        SCB->HFSR = SCB->HFSR;
        while(1);                                                   /* 硬fault是在取向量时发生的                    */
    } else if (SCB->HFSR & 0x40000000) {                            /* 总线,用法、存储管理fault                     */
        SCB->HFSR   = SCB->HFSR;                                    /* 清除fault标志                                */
        uiFaultCFSR = SCB->CFSR;                                    /* 记录fault状态                                */
        if(uiFaultCFSR & 0x9B) {                                    /* 存储器管理 fault                             */
             if(uiFaultCFSR & (1<<7)) {                             /* MMARVALID 存储器管理fault 的地址存储在MMAR   */
                 uiFaultADDR = SCB->MMFAR;
             }
             if(uiFaultCFSR & (1<<0)) while(1);                     /* IACCVIOL 取指访问违例                        */
             if(uiFaultCFSR & (1<<1)) while(1);                     /* DACCVIOL 数据访问违例                        */
             if(uiFaultCFSR & (1<<3)) while(1);                     /* MUNSTKERR 出栈时发生错误                     */
             if(uiFaultCFSR & (1<<4)) while(1);                     /* MSTKERR 入栈时发生错误                       */

        } else if (uiFaultCFSR & (0x9F << 8)) {                     /* 总线fault                                    */
             if(uiFaultCFSR & (0x80<<8)) {                          /* BFARVALID 总线fault 的地址存储在MMAR 中      */
                 uiFaultADDR = SCB->BFAR;
             }
             if(uiFaultCFSR & (0x1<<8)) while(1);                   /* IBUSERR 取指访问违例                         */
             if(uiFaultCFSR & (0x2<<8)) while(1);                   /* PRECISERR 精确的数据访问违例                 */
             if(uiFaultCFSR & (0x4<<8)) while(1);                   /* IMPRECISERR 不精确的数据访问违例             */
             if(uiFaultCFSR & (0x8<<8)) while(1);                   /* UNSTKERR 出栈时发生错误                      */
             if(uiFaultCFSR & (0x10<<8)) while(1);                  /* STKERR 入栈时发生错误                        */

        } else if (uiFaultCFSR & (0x30f << 16)) {                   /*用法fault                                     */

             if(uiFaultCFSR & (0x1<<16)) while(1);                  /* UNDEFINSTR 执行的指令其编码是未定义的        */
             if(uiFaultCFSR & (0x2<<16)) while(1);                  /* INVSTATE 试图切入ARM 状态                    */
             if(uiFaultCFSR & (0x4<<16)) while(1);                  /* INVPC 异常返回时试图非法地加载EXC_RETURN到PC */
             if(uiFaultCFSR & (0x8<<16)) while(1);                  /* NOCP 试图执行协处理器相关指令                */
             if(uiFaultCFSR & (0x100<<16)) while(1);                /* UNALIGNED 未对齐访问导致的fault              */
             if(uiFaultCFSR & (0x200<<16)) while(1);                /* DIVBYZERO 除法运算时除数为零                 */
        }
    }

    myprintf(uiFaultADDR);
    myprintf(*puiSrcAddr);
    myprintf(uiLRValue);
    myprintf(uiPCValue);
    myprintf(uiRtnAddr);

    while (1);
}
/*********************************************************************************************************
**  End Of File
*********************************************************************************************************/
