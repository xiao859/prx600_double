#ifndef _APP_USART_H
#define _APP_USART_H

/* 1、头文件包含 */
#include "stdio.h"
//#include "update.h"
#include "main.h"

/* 2、宏定义 */
#define USART_BUFFER_LEN        100               /* 定义最大接收字节数 120 KB */

#define MESSAGE_PACK_LENGTH     6

#define SCI_CMD_HEADER1         0xAA
#define SCI_CMD_HEADER2         0x55
#define SCI_ACK_HEADER1         0x33
#define SCI_ACK_HEADER2         0xBB

/* 3、数据类型定义 */
/* 串口消息包结构 */
typedef struct {
    uint8_t head1;
    uint8_t head2;
    uint8_t msg_id;
    uint8_t data1;
    uint8_t data2;
    uint8_t checksum;
    uint16_t rsvd;
} message_protocol;

typedef struct {
    uint8_t     recv_byte;
    uint32_t    recv_len;
    uint32_t    recv_complete;
    uint32_t    uart_rx_cnt;                        /* 接收的字节数 */
    uint8_t     uart_rx_buf[USART_BUFFER_LEN];
    uint8_t     uart_tx_buf[USART_BUFFER_LEN];
} UART_FILE;

extern volatile UART_FILE uart5;
extern volatile UART_FILE uart4;
extern volatile UART_FILE uart3;

/* 4、函数声明 */
void restart_usart_receive(USART_TypeDef *Instance);

void send_message(uint8_t msg_id, uint8_t data1, uint8_t data2);
void debug_tx3(const char *format,...);
#endif


