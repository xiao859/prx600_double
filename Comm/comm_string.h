#ifndef COMM_STRING_H_
#define COMM_STRING_H_

/* 1、头文件包含 */
#include <stdint.h>

/* 2、宏定义 */
#define LC_CMD_NUM              28

/* 3、数据类型定义 */
typedef struct
{
    char cmdstr[20];
    int  (*func_ptr)(volatile uint8_t *buff, char *p);
} LC_Command_string;

/* 4、函数声明 */
void registerFunc_init(void);
void cmd_parser_string(void);

#endif
