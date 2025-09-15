#include "updata.h"
#include "stdbool.h"
#include "app_uart.h"
#include "app_spi.h"
#include "stmflash.h"
#include "stmflash.h"
#include "xray.h"
#include "ct_exposure.h"

FLASH_OBProgramInitTypeDef OBInit;
uint32_t g_iapbuf[512];       /* 2K字节缓存 */

bool isFlashBank1(void)
{
	/* Allow Access to Flash control registers and user Flash */
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/* Get the Dual boot configuration status */
	HAL_FLASHEx_OBGetConfig(&OBInit);

	/* Enable/Disable dual boot feature */
	OBInit.OptionType = OPTIONBYTE_USER;
	OBInit.USERType   = OB_USER_BFB2;

	if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void bankSwitch(void)
{
	/* Allow Access to Flash control registers and user Flash */
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/* Get the Dual boot configuration status */
	HAL_FLASHEx_OBGetConfig(&OBInit);

	/* Enable/Disable dual boot feature */
	OBInit.OptionType = OPTIONBYTE_USER;
	OBInit.USERType   = OB_USER_BFB2;

	if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
	{
		OBInit.USERConfig = OB_BFB2_DISABLE;
	}
	else
	{
		OBInit.USERConfig = OB_BFB2_ENABLE;
	}

	if(HAL_FLASHEx_OBProgram (&OBInit) != HAL_OK)
	{
		while (1)
		{
			/* Make LED2 blink (100ms on, 2s off) to indicate error */
            falut_led(1);
			HAL_Delay(100);
		}
	}

	/* Start the Option Bytes programming process */
	if (HAL_FLASH_OB_Launch() != HAL_OK)
	{
		while (1)
		{
			/* Make LED2 blink (100ms on, 2s off) to indicate error */
            falut_led(1);
			HAL_Delay(100);
		}
	}

	/* Prevent Access to option bytes sector */
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

void FLASH_Erase_Bank(uint32_t FLASH_BANK_SELECT)
{
    // 解锁FLASH
    HAL_FLASH_Unlock();

    // 擦除FLASH页面（如果需要）
    // 注意：STM32的FLASH存储器在写入之前需要先擦除，擦除操作会将页面内容设置为全1（0xFFFFFFFF）
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
    EraseInitStruct.Banks = FLASH_BANK_SELECT;
    // 擦除一个bank
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // 锁定FLASH
    HAL_FLASH_Lock();
}

void FLASH_Erase_Page(void)
{
    // 解锁FLASH
    HAL_FLASH_Unlock();

    // 擦除FLASH页面（如果需要）
    // 注意：STM32的FLASH存储器在写入之前需要先擦除，擦除操作会将页面内容设置为全1（0xFFFFFFFF）
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page        = ((uint32_t)0x08040000);
	EraseInitStruct.NbPages     = 128;

    // 擦除128页
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // 锁定FLASH
    HAL_FLASH_Lock();
}

void iap_write_appbin(uint32_t appxaddr, uint8_t *appbuf, uint32_t appsize)
{
    uint32_t t;
    uint16_t i = 0;
    uint32_t temp;
    uint32_t fwaddr = appxaddr; /* 当前写入的地址 */
    uint8_t *dfu = appbuf;

    for (t = 0; t < appsize; t += 4)
    {
        temp = (uint32_t)dfu[3] << 24;
        temp |= (uint32_t)dfu[2] << 16;
        temp |= (uint32_t)dfu[1] << 8;
        temp |= (uint32_t)dfu[0];
        dfu += 4;               /* 偏移2个字节 */
        g_iapbuf[i++] = temp;

        if (i == 512)
        {
            i = 0;
            stmflash_write(fwaddr, g_iapbuf, 512);
            fwaddr += 2048;     /* 偏移2048  16 = 2 * 8  所以要乘以2 */
        }
    }

    if (i)
    {
        stmflash_write(fwaddr, g_iapbuf, i);  /* 将最后的一些内容字节写进去 */
    }
}

void bank_erase()
{
    if(isFlashBank1())
	{
		FLASH_Erase_Bank(FLASH_BANK_2);
		send_message(0x33, 0x22, 0x22);
	}
	else
	{
		FLASH_Erase_Bank(FLASH_BANK_1);
		send_message(0x33, 0x11, 0x11);
	}

    return;
}

void bank_write()
{
    __disable_irq();
    iap_write_appbin(ADDR_FLASH_BANK2_START_ADDR, &upgrade_buf.recv_buff[8], upgrade_buf.recv_cnt);
    __enable_irq();

    return;
}

bool crc_check()
{
    uint32_t crc_expect = (uint32_t)(upgrade_buf.recv_buff[7]) + (uint32_t)(upgrade_buf.recv_buff[6] << 8) +
                          (uint32_t)(upgrade_buf.recv_buff[5] << 16) + (uint32_t)(upgrade_buf.recv_buff[4] << 24);

    upgrade_buf.crc_rslt = ~upgrade_buf.crc_rslt;

    if (upgrade_buf.crc_rslt == crc_expect) return true;

    return false;
}

void upgrade_proc()
{
// uint32_t crc_expect, app_length_expect;
    /* 校验CRC，校验起始地址 */
    upgrade_buf.time_cnt = 0;
    while(1) {
        if (upgrade_buf.recv_cnt > 0 && upgrade_buf.time_cnt > 100) {
            // 发送结束了

            if (!crc_check()) {
                set_hv_state(HVPS_SM_ID_IDLE,0);
							  set_hv_state(HVPS_SM_ID_IDLE,1);
                /* 回复crc校验失败 */
                send_message(0x33, 0xFF, 0xFF);
                return;
            }
            uint32_t length = (uint32_t)(upgrade_buf.recv_buff[3]) + (uint32_t)(upgrade_buf.recv_buff[2] << 8) +
                              (uint32_t)(upgrade_buf.recv_buff[1] << 16) + (uint32_t)(upgrade_buf.recv_buff[0] << 24);
            if (length == upgrade_buf.recv_cnt) {
                send_message(0x33, 0xEE, 0xEE);
            }

            bank_erase();

            bank_write();

            bankSwitch();

            set_hv_state(HVPS_SM_ID_IDLE,0);
						set_hv_state(HVPS_SM_ID_IDLE,1);
        }
        upgrade_buf.time_cnt++;
        HAL_Delay(10);
    }

}


uint32_t crc_table[256] = {
    0x00000000, 0x77073096,  0xEE0E612C, 0x990951BA,   0x076DC419, 0x706AF48F,  0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4,  0xE0D5E91E, 0x97D2D988,   0x09B64C2B, 0x7EB17CBD,  0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2,  0xF3B97148, 0x84BE41DE,   0x1ADAD47D, 0x6DDDE4EB,  0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0,  0xFD62F97A, 0x8A65C9EC,   0x14015C4F, 0x63066CD9,  0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E,  0xD56041E4, 0xA2677172,   0x3C03E4D1, 0x4B04D447,  0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C,  0xDBBBC9D6, 0xACBCF940,   0x32D86CE3, 0x45DF5C75,  0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A,  0xC8D75180, 0xBFD06116,   0x21B4F4B5, 0x56B3C423,  0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808,  0xC60CD9B2, 0xB10BE924,   0x2F6F7C87, 0x58684C11,  0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106,  0x98D220BC, 0xEFD5102A,   0x71B18589, 0x06B6B51F,  0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934,  0x9609A88E, 0xE10E9818,   0x7F6A0DBB, 0x086D3D2D,  0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162,  0x856530D8, 0xF262004E,   0x6C0695ED, 0x1B01A57B,  0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950,  0x8BBEB8EA, 0xFCB9887C,   0x62DD1DDF, 0x15DA2D49,  0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE,  0xA3BC0074, 0xD4BB30E2,   0x4ADFA541, 0x3DD895D7,  0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC,  0xAD678846, 0xDA60B8D0,   0x44042D73, 0x33031DE5,  0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA,  0xBE0B1010, 0xC90C2086,   0x5768B525, 0x206F85B3,  0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998,  0xB0D09822, 0xC7D7A8B4,   0x59B33D17, 0x2EB40D81,  0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6,  0x03B6E20C, 0x74B1D29A,   0xEAD54739, 0x9DD277AF,  0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84,  0x0D6D6A3E, 0x7A6A5AA8,   0xE40ECF0B, 0x9309FF9D,  0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2,  0x1E01F268, 0x6906C2FE,   0xF762575D, 0x806567CB,  0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0,  0x10DA7A5A, 0x67DD4ACC,   0xF9B9DF6F, 0x8EBEEFF9,  0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E,  0x38D8C2C4, 0x4FDFF252,   0xD1BB67F1, 0xA6BC5767,  0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C,  0x36034AF6, 0x41047A60,   0xDF60EFC3, 0xA867DF55,  0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A,  0x256FD2A0, 0x5268E236,   0xCC0C7795, 0xBB0B4703,  0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28,  0x2BB45A92, 0x5CB36A04,   0xC2D7FFA7, 0xB5D0CF31,  0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226,  0x756AA39C, 0x026D930A,   0x9C0906A9, 0xEB0E363F,  0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14,  0x7BB12BAE, 0x0CB61B38,   0x92D28E9B, 0xE5D5BE0D,  0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242,  0x68DDB3F8, 0x1FDA836E,   0x81BE16CD, 0xF6B9265B,  0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70,  0x66063BCA, 0x11010B5C,   0x8F659EFF, 0xF862AE69,  0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE,  0x4E048354, 0x3903B3C2,   0xA7672661, 0xD06016F7,  0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC,  0x40DF0B66, 0x37D83BF0,   0xA9BCAE53, 0xDEBB9EC5,  0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A,  0x53B39330, 0x24B4A3A6,   0xBAD03605, 0xCDD70693,  0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8,  0x5D681B02, 0x2A6F2B94,   0xB40BBE37, 0xC30C8EA1,  0x5A05DF1B, 0x2D02EF8D
};

uint32_t crc_calculate(uint8_t byte, uint32_t crc)
{
    uint8_t index = (crc ^ byte) & 0xFF;

    return (crc >> 8) ^ crc_table[index];
}




