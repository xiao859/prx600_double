#ifndef _UPDATA_H
#define _UPDATA_H

/* 1、头文件包含 */
#include "stdbool.h"
#include "stdint.h"
#include "sys.h"

#define APP_FLASH_SIZE         (1024 * 100)   // 208KB
#define BOOT_FLASH_START_ADDR  0x08000000
#define APP1_FLASH_START_ADDR  0x0800C000
#define APP2_FLASH_START_ADDR  0x08040000
#define APP1_FLASH_END_ADDR (APP1_FLASH_START_ADDR + APP_FLASH_SIZE - 1)
#define APP2_FLASH_END_ADDR (APP2_FLASH_START_ADDR + APP_FLASH_SIZE - 1)

#define PROGRAM_MAX_SIZE (1024 * 1)   //1KB  must 4 bytes align

#define PARAM_FLASH_SIZE 128  // parameter size (bytes)  need no large PROGRAM_MAX_SIZE must 4 bytes align

#define PARAM_FLASH_START_ADDR   0x08008000 // parameter start address
#define PARAM_FLASH_END_ADDR    (PARAM_FLASH_START_ADDR + PARAM_FLASH_SIZE - 1)

#if(PARAM_FLASH_SIZE > PROGRAM_MAX_SIZE)
#error Parameter size is overlarge !
#endif

#define PARAM_FLASH_VALID_FLAG_ADDR  (PARAM_FLASH_START_ADDR + 0x04) //parameter :code valid flag

#define PARAM_FLASH_CODE_SIZE_ADDR      (PARAM_FLASH_START_ADDR + 0x04 * 3)//(Do not modify)
#define PARAM_CODE_VALID_ADDR           (PARAM_FLASH_START_ADDR + 0x04 * 4)//first  user
#define PARAM_CODE_INDEX_ADDR           (PARAM_FLASH_START_ADDR + 0x04 * 5)//first  user
#define PARAM_FLASH_DEVICE_ID_ADDR      (PARAM_FLASH_START_ADDR + 0x04 * 6)

#define PARAM_CODE_VALID_FLAG           0x5A5A5A5A
#define PARAM_CODE_INVALID_FLAG         0xAAAA5555
#define PARAM_CODE_INDEX_FLAG           0xA5A5A501
#define PARAM_FLASH_DEVICE_CODEKEY	    0x68107002

#define DEVICE_ID_SIZE 8   //(bytes) 4 bytes align

typedef void (*iapfun)(void);                   /* 定义一个函数类型的参数 */

void update_process(void);


#endif

