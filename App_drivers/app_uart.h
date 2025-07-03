#ifndef _APP_USART_H
#define _APP_USART_H


#include "stdio.h"
//#include "update.h"
#include "main.h"


#define USART_BUFFER_LEN        100               /* max */

#define MESSAGE_PACK_LENGTH     6

#define SCI_CMD_HEADER1         0xAA
#define SCI_CMD_HEADER2         0x55
#define SCI_ACK_HEADER1         0x33
#define SCI_ACK_HEADER2         0xBB

#define FRAME_BUF_NUM      10
typedef struct
{
    uint8_t head1;
    uint8_t head2;
    uint8_t msg_id;
    uint8_t data1;
    uint8_t data2;
    uint8_t checksum;
} message_protocol;

typedef struct
{
    message_protocol data[FRAME_BUF_NUM];//每帧缓存内容
    uint8_t head;//主程序读取位置索引
    uint8_t tail;//接收中断中写入位置索引
    uint8_t count;//当前缓存帧数
} UARTFIFO_t;

typedef struct
{
    uint8_t     recv_byte;
    uint32_t    recv_len;
    uint8_t     recv_complete;
    uint32_t    receiving;
    uint32_t    uart_rx_cnt;
    uint8_t     uart_rx_buf[USART_BUFFER_LEN];
    uint8_t     uart_tx_buf[USART_BUFFER_LEN];
} UART_FILE;

extern UART_FILE uart5;
extern UART_FILE uart4;
extern volatile UART_FILE uart3;

extern UARTFIFO_t uart4_frame_fifo;
extern UARTFIFO_t uart5_frame_fifo;

void restart_usart_receive(USART_TypeDef *Instance);
uint8_t uart_check(uint8_t* data, uint8_t len);

void send_message(uint8_t msg_id, uint8_t data1, uint8_t data2);
void debug_tx3(const char *format, ...);
#endif


