#include "app_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "tim.h"
#include "ins_task.h"
#include "user_lib.h"
#include "chassisR_task.h"

extern uint8_t uart1_rx_byte;

SemaphoreHandle_t uart_data_mutex;
SemaphoreHandle_t uart_tx_mutex;
QueueHandle_t     uart1_rx_queue;
UartCmd_t uart_cmd = {
    .velocity = 0.0f,
    .position_x = 0.0f,
    .yaw_turn = 0.0f,
    .leg_height = 0.08f,
    .roll_angle = 0.0f,
    .jump_trigger = 0,
    .enable = 0
};
void UartTask(void *argument)
{
    uint8_t ch;
    static char rx_buf[32];
    static uint8_t rx_index = 0;
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);

    for (;;)
    {
        if (xQueueReceive(uart1_rx_queue, &ch, portMAX_DELAY) == pdPASS)
        {
            if (ch == '\n' || ch == '\r')
            {
                rx_buf[rx_index] = '\0';

                if (rx_index > 0)
                {
                    parse_uart_frame(rx_buf);
                }

                rx_index = 0;
            }
            else
            {
                if (rx_index < sizeof(rx_buf) - 1)
                {
                    rx_buf[rx_index++] = ch;
                }
            }
        }
    }
}

void parse_uart_frame(char *cmd)
{

    char key;
    int value;
    char tx[64];

    // Nhận dạng lệnh: "V800", "H80", "X1000", "Y500", "R100", "J1", "E1"
    if (sscanf(cmd, "%c%d", &key, &value) != 2)
    {
        uart_send_str("ERR\r\n");
        return;
    }

    xSemaphoreTake(uart_data_mutex, portMAX_DELAY);

    switch (key)
    {
        case 'V':   // Velocity (m/s * 1000) - gửi V800 cho 0.8 m/s
            uart_cmd.velocity = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK V=%.3f\r\n", uart_cmd.velocity);
            break;

        case 'X':   // Position X (m * 1000) - gửi X1000 cho 1.0 m
            uart_cmd.position_x = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK X=%.3f\r\n", uart_cmd.position_x);
            break;

        case 'Y':   // Yaw turn rate (rad/s * 1000) - gửi Y500 cho 0.5 rad/s
            uart_cmd.yaw_turn = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK Y=%.3f\r\n", uart_cmd.yaw_turn);
            break;

        case 'H':   // Leg height (m * 1000) - gửi H80 cho 0.08 m
            uart_cmd.leg_height = (float)value * 0.001f;
            // Giới hạn 60mm - 210mm như PS2
            if (uart_cmd.leg_height < 0.060f) uart_cmd.leg_height = 0.060f;
            if (uart_cmd.leg_height > 0.210f) uart_cmd.leg_height = 0.210f;
            snprintf(tx, sizeof(tx), "OK H=%.3f\r\n", uart_cmd.leg_height);
            break;

        case 'R':   // Roll angle (rad * 1000) - gửi R100 cho 0.1 rad
            uart_cmd.roll_angle = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK R=%.3f\r\n", uart_cmd.roll_angle);
            break;

        case 'L':   // Line error (-1000 to 1000) - gửi L-500 cho line lệch trái
            uart_cmd.line_error = (float)value * 0.001f;
            // Giới hạn -1.0 đến 1.0 (normalized)
            if (uart_cmd.line_error < -1.0f) uart_cmd.line_error = -1.0f;
            if (uart_cmd.line_error > 1.0f) uart_cmd.line_error = 1.0f;
            snprintf(tx, sizeof(tx), "OK L=%.3f\r\n", uart_cmd.line_error);
            break;

        case 'J':   // Jump trigger (0 or 1)
            uart_cmd.jump_trigger = (value != 0) ? 1 : 0;
            snprintf(tx, sizeof(tx), "OK J=%d\r\n", uart_cmd.jump_trigger);
            break;

        case 'E':   // Enable/Disable (0 or 1)
            uart_cmd.enable = (value != 0) ? 1 : 0;
            snprintf(tx, sizeof(tx), "OK E=%d\r\n", uart_cmd.enable);
            break;
				case 'B':   // Buzzer control: B0=off, B1=beep, B2=alarm
				{
						if (value == 0)
						{
								// Tắt còi
								HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
								snprintf(tx, sizeof(tx), "OK B=OFF\r\n");
						}
						else if (value == 1)
						{
								// Single beep - bật còi (Python tự tắt sau 200ms)
								__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);
								HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
								snprintf(tx, sizeof(tx), "OK B=BEEP\r\n");
						}
						else if (value == 2)
						{
								// Continuous alarm - bật còi liên tục
								__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);
								HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
								snprintf(tx, sizeof(tx), "OK B=ALARM\r\n");
						}
						else
						{
								snprintf(tx, sizeof(tx), "ERR B=%d\r\n", value);
						}
				}
				break;
				case 'C':   // Set CCR2 (duty PWM)
				{
						uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim12);

						if (value < 0) value = 0;
						if (value > (int)arr) value = arr;

						__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, value);
						HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
						snprintf(tx, sizeof(tx),
										 "OK CCR2=%d\r\n", value);
				}
				break;
        default:
            xSemaphoreGive(uart_data_mutex);
            uart_send_str("ERR CMD\r\n");
            return;
    }
    xSemaphoreGive(uart_data_mutex);
    uart_send_str(tx);
}
/**
 * @brief Send odometry feedback to Python
 * @param chassis: pointer to chassis_t structure
 * @param ins: pointer to INS_t structure
 * @note Format: "F<v_filter>,<x_filter>,<yaw>,<yaw_rate>\r\n"
 *       Example: "F0.523,1.234,0.785,0.100\r\n"
 */
void uart_send_feedback(chassis_t *chassis, INS_t *ins)
{
    char tx[64];
    
    // Format: F<velocity>,<position>,<yaw>,<yaw_rate>
    // velocity: m/s (filtered)
    // position: m (estimated)
    // yaw: rad (total angle from IMU)
    // yaw_rate: rad/s (from gyro)
    snprintf(tx, sizeof(tx), "F%.3f,%.3f,%.3f,%.3f\r\n",
             chassis->v_filter,      // Vận tốc lọc Kalman (m/s)
             chassis->x_filter,      // Vị trí ước tính (m)
             ins->YawTotalAngle,     // Góc yaw tích lũy (rad)
             ins->Gyro[2]);          // Vận tốc góc yaw (rad/s)
    
    uart_send_str(tx);
}

void uart_send_str(const char *str)
{
    /* n?u CHUA d�ng mutex th� c� th? b? 2 d�ng n�y */
    xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)str,
                      strlen(str),
                      HAL_MAX_DELAY);

    xSemaphoreGive(uart_tx_mutex);
}

