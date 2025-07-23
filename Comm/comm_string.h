#ifndef COMM_STRING_H_
#define COMM_STRING_H_


#include <stdint.h>


#define LC_CMD_NUM              26


typedef struct
{
    char cmdstr[20];
    int  (*func_ptr)(volatile uint8_t *buff, char *p);
} LC_Command_string;


void registerFunc_init(void);
void cmd_parser_string(void);

#endif
