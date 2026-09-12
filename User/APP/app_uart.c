#include "app_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    int32_t param1;
    int32_t param2;
} UartData_t;

volatile UartData_t uart_data;

#define UART_RX_BUF_SIZE 64
static char rx_buf[UART_RX_BUF_SIZE];
static uint8_t rx_index = 0;

void UartTask(void *argument)
{
    uint8_t ch;

    for (;;)
    {
        if (xQueueReceive(uart1_rx_queue, &ch, portMAX_DELAY) == pdPASS)
        {
            if (ch == '\n')
            {
                rx_buf[rx_index] = '\0';
                parse_uart_frame(rx_buf);
                rx_index = 0;
            }
            else if (rx_index < UART_RX_BUF_SIZE - 1)
            {
                rx_buf[rx_index++] = ch;
            }
        }
    }
}

void parse_uart_frame(char *frame)
{
    int p1, p2;

    if (sscanf(frame, "$SET,%d,%d", &p1, &p2) == 2)
    {
        uart_data.param1 = p1;
        uart_data.param2 = p2;
    }
}
