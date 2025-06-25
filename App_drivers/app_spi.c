#include "app_spi.h"
#include "main.h"
#include "app_uart.h"
//#include "delay.h"
#include <string.h>


SPI_HandleTypeDef hspi1;
SFLASH_T flash_parpm;

#define SF_CS_0() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
#define SF_CS_1() HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];
uint8_t g_spiRxBuf[SPI_BUFFER_SIZE];
static uint8_t spi_w_buff[4*1024];	/* 用于写函数，先读出整个扇区，修改缓冲区后，再整个扇区回写 */
uint32_t g_spiLen;
__IO uint32_t wTransferState = TRANSFER_WAIT;
#define CMD_AAI 0xAD	/* AAI 连续编程指令(FOR SST25VF016B) */
#define CMD_DISWR 0x04	/* 禁止写, 退出AAI状态 */
#define CMD_EWRSR 0x50	/* 允许写状态寄存器的命令 */
#define CMD_WRSR 0x01	/* 写状态寄存器命令 */
#define CMD_WREN 0x06	/* 写使能命令 */
#define CMD_READ 0x03	/* 读数据区命令 */
#define CMD_RDSR 0x05	/* 读状态寄存器命令 */
#define CMD_RDID 0x9F	/* 读器件ID命令 */
#define CMD_SE 0x20		/* 擦除扇区命令 */
#define CMD_BE 0xC7		/* 批量擦除命令 */
#define DUMMY_BYTE 0xA5 /* 哑命令，可以为任意值，用于读操作 */

#define WIP_FLAG 0x01 	/* 状态寄存器中的正在编程标志（WIP) */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	wTransferState = TRANSFER_ERROR;
}

void bsp_spi_transfer(void)
{
	if (g_spiLen > SPI_BUFFER_SIZE)
	{
		return;
	}

	wTransferState = TRANSFER_WAIT;

	if (HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t *)g_spiTxBuf, (uint8_t *)g_spiRxBuf, g_spiLen) != HAL_OK)
	{
		Error_Handler();
	}

	while (wTransferState == TRANSFER_WAIT)
	{
		;
	}
}
static void bsp_write_enable(void)
{
	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (CMD_WREN);
	bsp_spi_transfer();
	SF_CS_1();
}
static void bsp_waitfor_write_end(void)
{
	SF_CS_0();
	g_spiTxBuf[0] = (CMD_RDSR);
	g_spiLen = 2;
	bsp_spi_transfer();
	SF_CS_1();

	while (1)
	{
		SF_CS_0();
		g_spiTxBuf[0] = (CMD_RDSR);
		g_spiTxBuf[1] = 0;
		g_spiLen = 2;
		bsp_spi_transfer();
		SF_CS_1();

		if ((g_spiRxBuf[1] & WIP_FLAG) != SET)
		{
			break;
		}
	}
}

uint32_t bsp_read_flash_id(void)
{
	uint32_t uiid;
	uint8_t id1, id2, id3;

	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[0] = (CMD_RDID);
	g_spiLen = 4;
	bsp_spi_transfer();

	id1 = g_spiRxBuf[1];
	id2 = g_spiRxBuf[2];
	id3 = g_spiRxBuf[3];

	SF_CS_1();

	uiid = ((uint32_t)id1 << 16) | ((uint32_t)id2 << 8) | id3;
	// send_message(id1, id2, id3);

	return uiid;
}

void bsp_read_info(void)
{
	{
		flash_parpm.ChipID = bsp_read_flash_id();
		switch (flash_parpm.ChipID)
		{
			case W25Q128_ID:
				strcpy(flash_parpm.ChipName, "W25Q128");
				flash_parpm.TotalSize = 16 * 1024 * 1024;
				flash_parpm.SectorSize = 4 * 1024;
				break;
			case W25Q256_ID:
				strcpy(flash_parpm.ChipName, "W25Q256");
				flash_parpm.TotalSize = 16 * 1024 * 1024;
				flash_parpm.SectorSize = 4 * 1024;
			break;
			case W25Q64BV_ID:
				strcpy(flash_parpm.ChipName, "W25Q64BV");
				flash_parpm.TotalSize = 8 * 1024 * 1024;
				flash_parpm.SectorSize = 4 * 1024;
			break;
			default:
				strcpy(flash_parpm.ChipName, "Unknow Flash");
				flash_parpm.TotalSize = 2 * 1024 * 1024;
				flash_parpm.SectorSize = 4 * 1024;
				break;
		}
	}
}
void bsp_erase_sector(uint32_t _uiSectorAddr)
{
	bsp_write_enable();

	/* 擦除扇区操作 */
	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = CMD_SE;
	g_spiTxBuf[g_spiLen++] = ((_uiSectorAddr & 0xFF0000) >> 16);
	g_spiTxBuf[g_spiLen++] = ((_uiSectorAddr & 0xFF00) >> 8);
	g_spiTxBuf[g_spiLen++] = (_uiSectorAddr & 0xFF);
	bsp_spi_transfer();
	SF_CS_1();

	bsp_waitfor_write_end();
}
void bsp_erase_chip(void)
{
	bsp_write_enable();

	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = CMD_BE;
	bsp_spi_transfer();
	SF_CS_1();
	bsp_waitfor_write_end();
}

void bsp_page_write(uint8_t *_pBuf, uint32_t _uiWriteAddr, uint16_t _usSize)
{
	uint32_t i, j;

	for (j = 0; j < _usSize / 256; j++)
	{
		bsp_write_enable();

		SF_CS_0();
		g_spiLen = 0;
		g_spiTxBuf[g_spiLen++] = (0x02);
		g_spiTxBuf[g_spiLen++] = ((_uiWriteAddr & 0xFF0000) >> 16);
		g_spiTxBuf[g_spiLen++] = ((_uiWriteAddr & 0xFF00) >> 8);
		g_spiTxBuf[g_spiLen++] = (_uiWriteAddr & 0xFF);
		for (i = 0; i < 256; i++)
		{
			g_spiTxBuf[g_spiLen++] = (*_pBuf++);
		}
		bsp_spi_transfer();
		SF_CS_1();
		bsp_waitfor_write_end();

		_uiWriteAddr += 256;
	}
	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (CMD_DISWR);
	bsp_spi_transfer();
	SF_CS_1();

	bsp_waitfor_write_end();
}

void bsp_read_buffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize)
{
	uint16_t rem;
	uint16_t i;

	if ((_uiSize == 0) ||(_uiReadAddr + _uiSize) > flash_parpm.TotalSize)
	{
		return;
	}
	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (CMD_READ);
	g_spiTxBuf[g_spiLen++] = ((_uiReadAddr & 0xFF0000) >> 16);
	g_spiTxBuf[g_spiLen++] = ((_uiReadAddr & 0xFF00) >> 8);
	g_spiTxBuf[g_spiLen++] = (_uiReadAddr & 0xFF);
	bsp_spi_transfer();
	for (i = 0; i < _uiSize / SPI_BUFFER_SIZE; i++)
	{
		g_spiLen = SPI_BUFFER_SIZE;
		bsp_spi_transfer();

		memcpy(_pBuf, g_spiRxBuf, SPI_BUFFER_SIZE);
		_pBuf += SPI_BUFFER_SIZE;
	}

	rem = _uiSize % SPI_BUFFER_SIZE;
	if (rem > 0)
	{
		g_spiLen = rem;
		bsp_spi_transfer();

		memcpy(_pBuf, g_spiRxBuf, rem);
	}
	SF_CS_1();
}
static uint8_t bsp_cmp_data(uint32_t _uiSrcAddr, uint8_t *_ucpTar, uint32_t _uiSize)
{
	uint16_t i, j;
	uint16_t rem;
	if ((_uiSrcAddr + _uiSize) > flash_parpm.TotalSize)
	{
		return 1;
	}

	if (_uiSize == 0)
	{
		return 0;
	}

	SF_CS_0();
	g_spiLen = 0;
	g_spiTxBuf[g_spiLen++] = (CMD_READ);
	g_spiTxBuf[g_spiLen++] = ((_uiSrcAddr & 0xFF0000) >> 16);
	g_spiTxBuf[g_spiLen++] = ((_uiSrcAddr & 0xFF00) >> 8);
	g_spiTxBuf[g_spiLen++] = (_uiSrcAddr & 0xFF);
	bsp_spi_transfer();

	for (i = 0; i < _uiSize / SPI_BUFFER_SIZE; i++)
	{
		g_spiLen = SPI_BUFFER_SIZE;
		bsp_spi_transfer();

		for (j = 0; j < SPI_BUFFER_SIZE; j++)
		{
			if (g_spiRxBuf[j] != *_ucpTar++)
			{
				goto NOTEQ;		/* 不相等 */
			}
		}
	}

	rem = _uiSize % SPI_BUFFER_SIZE;	/* 剩余字节 */
	if (rem > 0)
	{
		g_spiLen = rem;
		bsp_spi_transfer();

		for (j = 0; j < rem; j++)
		{
			if (g_spiRxBuf[j] != *_ucpTar++)
			{
				goto NOTEQ;		/* 不相等 */
			}
		}
	}
	SF_CS_1();
	return 0;

NOTEQ:
	SF_CS_1();
	return 1;
}
//返 回 值: 0 : 不需要擦除， 1 ：需要擦除
static uint8_t bsp_need_erase(uint8_t * _ucpOldBuf, uint8_t *_ucpNewBuf, uint16_t _usLen)
{
	uint16_t i;
	uint8_t ucOld;

	for (i = 0; i < _usLen; i++)
	{
		ucOld = *_ucpOldBuf++;
		ucOld = ~ucOld;
		if ((ucOld & (*_ucpNewBuf++)) != 0)
		{
			return 1;
		}
	}
	return 0;
}
static uint8_t bsp_auto_write_sector(uint8_t *_ucpSrc, uint32_t _uiWrAddr, uint16_t _usWrLen)
{
	uint16_t i;
	uint16_t j;
	uint32_t uiFirstAddr;
	uint8_t ucNeedErase;
	uint8_t cRet;

	if (_usWrLen == 0)
	{
		return 1;
	}


	if (_uiWrAddr >= flash_parpm.TotalSize)
	{
		return 0;
	}

	if (_usWrLen > flash_parpm.SectorSize)
	{
		return 0;
	}

	bsp_read_buffer(spi_w_buff, _uiWrAddr, _usWrLen);
	if (memcmp(spi_w_buff, _ucpSrc, _usWrLen) == 0)
	{
		return 1;
	}

	ucNeedErase = 0;
	if (bsp_need_erase(spi_w_buff, _ucpSrc, _usWrLen))
	{
		ucNeedErase = 1;
	}

	uiFirstAddr = _uiWrAddr & (~(flash_parpm.SectorSize - 1));

	if (_usWrLen == flash_parpm.SectorSize)
	{
		for	(i = 0; i < flash_parpm.SectorSize; i++)
		{
			spi_w_buff[i] = _ucpSrc[i];
		}
	}
	else
	{

		bsp_read_buffer(spi_w_buff, uiFirstAddr, flash_parpm.SectorSize);

		i = _uiWrAddr & (flash_parpm.SectorSize - 1);
		memcpy(&spi_w_buff[i], _ucpSrc, _usWrLen);
	}

	cRet = 0;
	for (i = 0; i < 3; i++)
	{

		if (ucNeedErase == 1)
		{
			bsp_erase_sector(uiFirstAddr);
		}

		bsp_page_write(spi_w_buff, uiFirstAddr, flash_parpm.SectorSize);

		if (bsp_cmp_data(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
		{
			cRet = 1;
			break;
		}
		else
		{
			if (bsp_cmp_data(_uiWrAddr, _ucpSrc, _usWrLen) == 0)
			{
				cRet = 1;
				break;
			}
			for (j = 0; j < 10000; j++);
		}
	}

	return cRet;
}
uint8_t bsp_write_buffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint32_t _usWriteSize)
{
	uint32_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

	Addr = _uiWriteAddr % flash_parpm.SectorSize;
	count = flash_parpm.SectorSize - Addr;
	NumOfPage =  _usWriteSize / flash_parpm.SectorSize;
	NumOfSingle = _usWriteSize % flash_parpm.SectorSize;

	if (Addr == 0)
	{
		if (NumOfPage == 0)
		{
			if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
			{
				return 0;
			}
		}
		else
		{
			while (NumOfPage--)
			{
				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, flash_parpm.SectorSize) == 0)
				{
					return 0;
				}
				_uiWriteAddr +=  flash_parpm.SectorSize;
				_pBuf += flash_parpm.SectorSize;
			}
			if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
			{
				return 0;
			}
		}
	}
	else
	{
		if (NumOfPage == 0)
		{
			if (NumOfSingle > count)
			{
				temp = NumOfSingle - count;

				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, count) == 0)
				{
					return 0;
				}

				_uiWriteAddr +=  count;
				_pBuf += count;

				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, temp) == 0)
				{
					return 0;
				}
			}
			else
			{
				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, _usWriteSize) == 0)
				{
					return 0;
				}
			}
		}
		else	/* 数据长度大于等于扇区大小 */
		{
			_usWriteSize -= count;
			NumOfPage =  _usWriteSize / flash_parpm.SectorSize;
			NumOfSingle = _usWriteSize % flash_parpm.SectorSize;
			if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, count) == 0)
			{
				return 0;
			}

			_uiWriteAddr +=  count;
			_pBuf += count;

			while (NumOfPage--)
			{
				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, flash_parpm.SectorSize) == 0)
				{
					return 0;
				}
				_uiWriteAddr +=  flash_parpm.SectorSize;
				_pBuf += flash_parpm.SectorSize;
			}

			if (NumOfSingle != 0)
			{
				if (bsp_auto_write_sector(_pBuf, _uiWriteAddr, NumOfSingle) == 0)
				{
					return 0;
				}
			}
		}
	}
	return 1;	/* 成功 */
}

/* 读参数，位置在w25q64上 */
void get_flash_parament(uint8_t *buff, uint32_t size)
{
	bsp_read_buffer((uint8_t *)buff, XRAY_PARAMENT_ADDDR, size);

	return;
}

/* 读参数，位置在w25q64上 */
void wirte_flash_parament(uint8_t *buff, uint32_t size)
{
	bsp_write_buffer((uint8_t *)buff, XRAY_PARAMENT_ADDDR, size);

	return;
}
