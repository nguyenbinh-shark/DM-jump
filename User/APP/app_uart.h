#ifndef APP_UART_H
#define APP_UART_H

void UartTask(void *argument);
void parse_uart_frame(char *frame);

#endif
