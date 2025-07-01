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
    uint32_t    uart_rx_cnt;                        
    uint8_t     uart_rx_buf[USART_BUFFER_LEN];
    uint8_t     uart_tx_buf[USART_BUFFER_LEN];
} UART_FILE;

extern volatile UART_FILE uart5;
extern volatile UART_FILE uart4;
extern volatile UART_FILE uart3;

void restart_usart_receive(USART_TypeDef *Instance);

void send_message(uint8_t msg_id, uint8_t data1, uint8_t data2);
void debug_tx3(const char *format,...);
#endif


