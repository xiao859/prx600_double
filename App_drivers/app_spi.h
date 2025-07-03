#ifndef APP_SPI_H_
#define APP_SPI_H_

#include "main.h"
#include "spi.h"

#define SF_MAX_PAGE_SIZE	(4 * 1024)
#define	SPI_BUFFER_SIZE		(4 * 1024)


#define XRAY_PARAMENT_ADDDR			0x000000

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
	uint32_t ChipID;		
	char ChipName[16];		
	uint32_t TotalSize;		
	uint16_t SectorSize;	
}SFLASH_T;
extern SFLASH_T flash_parpm;

extern uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];
extern uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];
extern uint32_t g_spiLen;


void bsp_read_info(void);
uint8_t bsp_write_buffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint32_t _usWriteSize);
void bsp_read_buffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);
void get_flash_parament(uint8_t *buff, uint32_t size);
void wirte_flash_parament(uint8_t *buff, uint32_t size);

#endif
