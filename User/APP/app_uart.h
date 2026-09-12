#ifndef APP_UART_H
#define APP_UART_H
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "chassisR_task.h"

extern uint8_t uart1_rx_byte;

/* ===== UART command data ===== */
typedef struct
{
    float velocity;      // Vxxx - target velocity (m/s)
    float position_x;    // Xxxx - target position X (m)
    float yaw_turn;      // Yxxx - yaw turn rate (rad/s)
    float leg_height;    // Hxxx - leg length (m)
    float roll_angle;    // Rxxx - roll angle (rad)
    float line_error;    // Lxxx - line following error (normalized -1.0 to 1.0)
    uint8_t jump_trigger; // J1 - jump command
    uint8_t enable;      // E1/E0 - enable/disable control
} UartCmd_t;

extern UartCmd_t uart_cmd;

/* ===== UART task ===== */
extern SemaphoreHandle_t uart_data_mutex;
extern SemaphoreHandle_t uart_tx_mutex;
extern QueueHandle_t     uart1_rx_queue;

void UartTask(void *argument);
void parse_uart_frame(char *frame);
void uart_send_str(const char *str);
void uart_send_feedback(chassis_t *chassis, INS_t *ins);

#endif
