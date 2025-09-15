#include "app_uart.h"
#include "stm32g4xx_hal.h"
#include "app_fun.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "updata.h"

volatile UART_FILE uart1 = {0};
volatile UART_FILE uart3 = {0};
UART_FILE uart4 = {0};
UART_FILE uart5 = {0};

UARTFIFO_t uart4_frame_fifo = {0};
UARTFIFO_t uart5_frame_fifo = {0};

upgrade_uart upgrade_buf = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)             /*  */
    {

        uart4.recv_len++;
        if (uart4.recv_byte == 0x0A)
            uart4.recv_complete  = 1;

        if (uart4.uart_rx_cnt < USART_BUFFER_LEN)
        {
            uart4.uart_rx_buf[uart4.uart_rx_cnt] = uart4.recv_byte;
            uart4.uart_rx_cnt++;
        }

        HAL_UART_Receive_IT(&huart4, (uint8_t *)&uart4.recv_byte, 1);
    }

    else if (huart->Instance == UART5)               /*  */
    {
        uint8_t d = uart5.recv_byte;
        if (get_hv_state(1) == HVPS_SM_ID_UPDATE_RUN && upgrade_buf.uart_idx == 4)
        {
            upgrade_buf.time_cnt = 0;
            upgrade_buf.recv_buff[upgrade_buf.recv_cnt] = d;
            upgrade_buf.recv_cnt++;
            if (upgrade_buf.recv_cnt > 8)
            {
                upgrade_buf.crc_rslt = crc_calculate(d, upgrade_buf.crc_rslt);
            }
        }
        else
        {

            if (!uart5.receiving)
            {
                if (d == 0xAA)
                {
                    uart5.uart_rx_buf[0] = d;
                    uart5.uart_rx_cnt = 1;
                    uart5.receiving = 1;
                }
            }
            else
            {
                uart5.uart_rx_buf[uart5.uart_rx_cnt++] = d;

                if (uart5.uart_rx_cnt == 6)
                {
                    // 校验
                    uint8_t check_recv = uart_check(&(uart5.uart_rx_buf[2]), 4);
                    if (check_recv == 1)
                    {
                        if (uart5.uart_rx_buf[2] == 0x29)
                            Setenable((message_protocol *)uart5.uart_rx_buf);
                        // 放入FIFO
                        else if (uart5_frame_fifo.count < FRAME_BUF_NUM)
                        {
                            uint8_t idx = uart5_frame_fifo.tail;
                            memcpy(&uart5_frame_fifo.data[idx], uart5.uart_rx_buf, 6);
                            uart5_frame_fifo.tail = (uart5_frame_fifo.tail + 1) % FRAME_BUF_NUM;
                            uart5_frame_fifo.count++;
                        }
                    }
                    // 重置状态
                    uart5.receiving = 0;
                    uart5.uart_rx_cnt = 0;
                }
            }
        }
        // 再次开启下一字节接收
        HAL_UART_Receive_IT(&huart5, &uart5.recv_byte, 1);
    }
}

void restart_usart_receive(USART_TypeDef *Instance)
{
    if (Instance == UART4)
    {
        uart4.recv_complete = 0;
        uart4.recv_len = 0;
        uart4.uart_rx_cnt = 0;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart4.recv_byte, 1);
    }

    return;
}

void send_message(uint8_t msg_id, uint8_t data1, uint8_t data2)
{
    message_protocol msg_reply;
    msg_reply.head1 = SCI_ACK_HEADER1;
    msg_reply.head2 = SCI_ACK_HEADER2;
    msg_reply.msg_id = msg_id;
    msg_reply.data2  = data2;
    msg_reply.data1  = data1;
    msg_reply.checksum = (0 - (msg_reply.msg_id + msg_reply.data1 + msg_reply.data2)) & 0xFF;

    memcpy((uint8_t *)uart5.uart_tx_buf, &msg_reply, MESSAGE_PACK_LENGTH);
    HAL_UART_Transmit(&huart5, (uint8_t*)uart5.uart_tx_buf, MESSAGE_PACK_LENGTH, 1000);
}

uint8_t uart_check(uint8_t* data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum = sum + data[i];
    }
    if ((sum & 0xFF) == 0x00)
        return 1;
    else
        return 0;
}

void debug_tx3(const char *format, ...)
{
    unsigned char UartTx4Buf[128];
    uint16_t len;
    va_list args;
    va_start(args, format);
    len = vsnprintf((char*)UartTx4Buf, sizeof(UartTx4Buf) +1, (char*)format, args);
    va_end(args);
    HAL_UART_Transmit(&huart4, UartTx4Buf, len, 1000);

    return;
}


