#include "app_uart.h"
#include "stm32g4xx_hal.h"
//#include "update.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

volatile UART_FILE uart1 = {0};
volatile UART_FILE uart3 = {0};
volatile UART_FILE uart4 = {0};
volatile UART_FILE uart5 = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4) {            /*  */
        uart4.recv_len++;
        if (uart4.uart_rx_cnt < USART_BUFFER_LEN)
        {
            uart4.uart_rx_buf[uart4.uart_rx_cnt] = uart4.recv_byte;
            uart4.uart_rx_cnt++;
        }
        if (uart4.recv_len >= 6) {
            uart4.recv_complete  = 1;
        }

        HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart4.recv_byte, 1);
    } else if(huart->Instance == UART5) {            /*  */
        uart5.recv_len++;
        if (uart5.uart_rx_cnt < USART_BUFFER_LEN)
        {
            uart5.uart_rx_buf[uart5.uart_rx_cnt] = uart5.recv_byte;
            uart5.uart_rx_cnt++;
        }
        if (uart5.recv_len >= 6) {
            uart5.recv_complete  = 1;
        }

        HAL_UART_Receive_IT(&huart5, (uint8_t *)&uart5.recv_byte, 1);
    } else if (huart->Instance == USART3) {
        uart3.recv_len++;
        if (uart3.recv_byte == '\n') uart3.recv_complete  = 1;

        if (uart3.uart_rx_cnt < USART_BUFFER_LEN)
        {
            uart3.uart_rx_buf[uart3.uart_rx_cnt] = uart3.recv_byte;
            uart3.uart_rx_cnt++;
        }

        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3.recv_byte, 1);
    }
}

void restart_usart_receive(USART_TypeDef *Instance)
{
    if (Instance == UART4) {
        uart4.recv_complete = 0;
        uart4.recv_len = 0;
        uart4.uart_rx_cnt = 0;
        HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart4.recv_byte, 1);
    } else if (Instance == UART5) {
        uart5.recv_complete = 0;
        uart5.recv_len = 0;
        uart5.uart_rx_cnt = 0;
        HAL_UART_Receive_IT(&huart5, (uint8_t *)&uart5.recv_byte, 1);
    } else if (Instance == USART3) {
        uart3.recv_complete = 0;
        uart3.recv_len = 0;
        uart3.uart_rx_cnt = 0;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3.recv_byte, 1);
    }

    return;
}

void send_message(uint8_t msg_id, uint8_t data1, uint8_t data2)
{
    message_protocol msg_reply;
    msg_reply.head1 = SCI_ACK_HEADER1;
    msg_reply.head2 = SCI_ACK_HEADER2;
    msg_reply.msg_id = msg_id;
    msg_reply.data1  = data1;
    msg_reply.data2  = data2;
    msg_reply.checksum = ( 0 - (msg_reply.msg_id + msg_reply.data1 + msg_reply.data2)) & 0xFF;

    memcpy((uint8_t *)uart4.uart_tx_buf, &msg_reply, MESSAGE_PACK_LENGTH);
    HAL_UART_Transmit(&huart4, (uint8_t*)uart4.uart_tx_buf, MESSAGE_PACK_LENGTH, 1000);

    memcpy((uint8_t *)uart5.uart_tx_buf, &msg_reply, MESSAGE_PACK_LENGTH);
    HAL_UART_Transmit(&huart5, (uint8_t*)uart5.uart_tx_buf, MESSAGE_PACK_LENGTH, 1000);
}

void debug_tx3(const char *format,...)
{
	unsigned char UartTx3Buf[128];
	uint16_t len;
	va_list args;
	va_start(args,format);
	len = vsnprintf((char*)UartTx3Buf,sizeof(UartTx3Buf)+1,(char*)format,args);
	va_end(args);
	HAL_UART_Transmit(&huart3, UartTx3Buf, len, 1000);

	return;
}


