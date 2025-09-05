#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

/* 1¡¢Í·ÎÄ¼þ°üº¬ */
#include "main.h"

/* 2¡¢ºê¶¨Òå */
#define SF_MAX_PAGE_SIZE	(4 * 1024)
#define	SPI_BUFFER_SIZE		(4 * 1024)

/* ÉäÔ´²ÎÊýÔÚFLASHÖÐ´æ·ÅµÄÆðÊ¼µØÖ· */
#define XRAY_PARAMENT_ADDDR			0x000000

/* 3¡¢Êý¾ÝÀàÐÍ¶¨Òå */
/* ¶¨Òå´®ÐÐFlash ID */
enum
{
	W25Q64BV_ID    = 0xEF4017, /* BV, JV, FV */
	W25Q128_ID     = 0xEF4018,
    W25Q256_ID     = 0xEF4019
};
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

typedef struct
{
	uint32_t ChipID;		/* Ð¾Æ¬ID */
	char ChipName[16];		/* Ð¾Æ¬ÐÍºÅ×Ö·û´®£¬Ö÷ÒªÓÃÓÚÏÔÊ¾ */
	uint32_t TotalSize;		/* ×ÜÈÝÁ¿ */
	uint16_t SectorSize;	/* ÉÈÇø´óÐ¡ */
}SFLASH_T;
extern SFLASH_T flash_parpm;

extern uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];
extern uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];
extern uint32_t g_spiLen;

/* 4¡¢º¯ÊýÉùÃ÷ */
void bsp_read_info(void);
uint8_t bsp_write_buffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint32_t _usWriteSize);
void bsp_read_buffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);
void get_flash_parament(uint8_t *buff, uint32_t size);
void wirte_flash_parament(uint8_t *buff, uint32_t size);
void bsp_erase_sector(uint32_t _uiSectorAddr);

#endif
