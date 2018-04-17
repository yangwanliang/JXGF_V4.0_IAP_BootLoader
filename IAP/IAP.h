#ifndef __IAP_H
#define	__IAP_H

//#include "config.h"
#include "lpc177x_8x.h"
/* ����IAP������ */
//											����		����
#define		IAP_SELECTOR		50		// ѡ������		����ʼ�����š����������š�
#define		IAP_RAMTOFLASH		51		// ��������		��FLASHĿ���ַ��RAMԴ��ַ��д���ֽ�����ϵͳʱ��Ƶ�ʡ�		
#define		IAP_ERASESECTOR		52		// ��������		����ʼ�����š����������š�ϵͳʱ��Ƶ�ʡ�
#define		IAP_BLANKCHK		53		// �������		����ʼ�����š����������š�
#define		IAP_READPARTID		54		// ������ID		���ޡ�
#define		IAP_BOOTCODEID		55		// ��Boot�汾��	���ޡ�
#define		IAP_COMPARE			56		// �Ƚ�����		��FLASH��ʼ��ַ��RAM��ʼ��ַ����Ҫ�Ƚϵ��ֽ�����
#define		IAP_REINVOKE_ISP	57		// ���µ���ISP	���ޡ�

/* ����IAP����״̬�� */
#define		CMD_SUCCESS			0
#define		INVALID_COMMAND		1
#define		SRC_ADDR_ERROR		2
#define		DST_ADDR_ERROR		3
#define		DST_ADDR_NOT_MAPPED	5	 
#define		SRC_ADDR_NOT_MAPPED	4
#define		COUNT_ERROR			7
#define		SECTOR_NOT_BLANK	8
#define		SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION		9
#define		COMMPARE_ERROR		10
#define		BUSY				11

#define IAP_FLASH_PAGE_SIZE_BYTES							256
#define IAP_FLASH_PAGE_SIZE_WORDS							(IAP_FLASH_PAGE_SIZE_BYTES >> 2)



/*********************************************************************************************************
**��������:  SelSector
**��������:  IAP����������ѡ�񣬴���Ϊ50
**��ڲ���:  sec1	��ʼ����
**			 sec2	��ֹ����
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	SelSector(uint8_t	sec1,uint8_t	sec2);


/*********************************************************************************************************
**��������:  RamToFlash 
**��������:  ����RAM�����ݵ�FLASH���������51
**��ڲ���:  dst		Ŀ���ַ����FLASH��ʼ��ַ����512�ֽ�Ϊ�ֽ�
**			 src		Դ��ַ����RAM��ַ����ַ�����ֶ���
**			 no		    �����ֽڸ�����Ϊ256/512/1024/4096
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	RamToFlash(uint32_t	dst, uint32_t	src, uint32_t	no);


/*********************************************************************************************************
**��������:  RamToFlash
**��������:  �����������������52
**��ڲ���:  sec1	��ʼ����
**			 sec2	��ֹ����
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	EraseSector(uint32_t	sec1, uint32_t	sec2);


/*********************************************************************************************************
**��������:  BlankCHK
**��������:  ����������������53
**��ڲ���:  sec1	��ʼ����
**			 sec2	��ֹ����
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	BlankCHK(uint32_t	sec1,uint32_t	sec2);


/*********************************************************************************************************
**��������:  ReadParID
**��������:  ������ID���������54
**��ڲ���:  ����ID��ַָ��
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	ReadParID(uint32_t *Device_ID);


/*********************************************************************************************************
**��������:  BootCodeID
**��������:  ��boot����ID���������55
**��ڲ���:  boot����ID��ַָ��
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	BootCodeID(uint32_t *Boot_ID);
	

/*********************************************************************************************************
**��������:  Compare 
**�������ܣ� У�����ݣ��������56
**��ڲ���:  dst		Ŀ���ַ����RAM/FLASH��ʼ��ַ����ַ�����ֶ���
**			 src		Դ��ַ����RAM/RAM��ַ����ַ�����ֶ���
**			 no		    �����ֽڸ����������ܱ�4����
**���ڲ���:  IAP ����״̬��
**			 IAP����ֵ��paramout��������
********************************************************************************************************/
extern	uint32_t	Compare(uint32_t	dst, uint32_t	src, uint32_t	no);


/*********************************************************************************************************
**��������: Reinvoke_ISP 
**��������: ���µ���ISP���������57��
**��ڲ���: ��
**���ڲ���: ��
********************************************************************************************************/
extern void  Reinvoke_ISP(void);


/*********************************************************************************************************
**��������:  WriteFlash
**��������:  ��FLASH��д������
**��ڲ���:  dst		Ŀ���ַ����FLASH��ʼ��ַ����ַ������256�ֽڶ��룬����ַ�ĵ�8λ����Ϊ0
**			 src		Դ��ַ����RAM��ַ����ַ�����ֶ��룬����ַ�ĵ���λ����Ϊ0
**			 no		    д���ֽ���������Ϊ256/512/1024/4096
**���ڲ���:  Compare�ķ���ֵ�������ɹ���ȷ����0
********************************************************************************************************/
extern	uint8_t	WriteFlash(uint32_t dst, uint32_t src, uint32_t no);
#endif