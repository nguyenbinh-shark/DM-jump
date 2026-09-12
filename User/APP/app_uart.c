/**
 * @file      app_uart.c
 * @brief     UART command parsing and telemetry feedback implementation
 *            Hiện thực giải mã lệnh UART và truyền dữ liệu phản hồi đo đạc
 * @author    Trần Nguyên Bình
 * @email     trannguyenbinh.shark@gmail.com
 * @website   https://nguyenbinh-shark.github.io/
 * @version   1.0.0
 * @date      2024-2026
 * 
 * @copyright Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * @license   MIT License (see LICENSE file in root directory)
 * 
 * ==============================================================================
 * DESCRIPTION / MÔ TẢ:
 * English:
 *   Implements the high-speed UART communication protocol via USART1:
 *   - Receives byte stream via interrupt and puts into FreeRTOS RX queue.
 *   - Parses commands: 'V' (velocity), 'X' (position), 'Y' (yaw rate),
 *     'H' (leg height), 'R' (roll angle), 'L' (line tracking), 'J' (jump),
 *     'E' (enable), 'B' (buzzer), 'C' (PWM compare).
 *   - Transmits periodic odometry feedback frames "F<v>,<x>,<yaw>,<yaw_rate>\r\n".
 *   - Synchronized using mutexes for thread-safe data access and transmission.
 * 
 * Tiếng Việt:
 *   Hiện thực giao thức giao tiếp UART tốc độ cao qua USART1:
 *   - Nhận luồng byte qua ngắt và đẩy vào hàng đợi FreeRTOS RX queue.
 *   - Giải mã lệnh: 'V' (vận tốc), 'X' (vị trí), 'Y' (tốc độ yaw),
 *     'H' (chiều cao chân), 'R' (góc nghiêng roll), 'L' (dò line), 'J' (bật nhảy),
 *     'E' (bật/tắt), 'B' (còi báo), 'C' (độ rộng xung PWM).
 *   - Truyền khung phản hồi đo đạc chu kỳ "F<v>,<x>,<yaw>,<yaw_rate>\r\n".
 *   - Đồng bộ hóa đa luồng an toàn bằng RTOS Mutex khi đọc ghi dữ liệu và truyền UART.
 * ==============================================================================
 */

#include "app_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "tim.h"
#include "ins_task.h"
#include "user_lib.h"
#include "chassisR_task.h"

/* Private variables ---------------------------------------------------------*/
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

/**
 * @brief FreeRTOS task that listens for incoming bytes and reconstructs frames
 *        Task FreeRTOS lắng nghe các byte nhận được và ghép thành khung lệnh hoàn chỉnh
 * @param argument Unused task parameter / Tham số không sử dụng
 */
void UartTask(void *argument)
{
    uint8_t ch;
    static char rx_buf[32];
    static uint8_t rx_index = 0;

    /* Start UART receive in interrupt mode (1 byte at a time) */
    /* Bắt đầu nhận UART ở chế độ ngắt (từng byte một) */
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);

    for (;;)
    {
        /* Block waiting for next byte from interrupt queue */
        /* Chờ byte tiếp theo từ hàng đợi ngắt */
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

/**
 * @brief Parse incoming ASCII command and update control parameters
 *        Giải mã chuỗi lệnh ASCII nhận được và cập nhật tham số điều khiển
 * @param cmd Null-terminated ASCII command string (e.g. "V800", "H80", "J1")
 */
void parse_uart_frame(char *cmd)
{
    char key;
    int value;
    char tx[64];

    /* Recognize command format: "<key><int_value>" (e.g., "V800", "H80", "X1000", "Y500", "R100", "J1", "E1") */
    /* Nhận dạng định dạng lệnh: "<ký tự><giá trị số nguyên>" */
    if (sscanf(cmd, "%c%d", &key, &value) != 2)
    {
        uart_send_str("ERR\r\n");
        return;
    }

    /* Lock data mutex for thread-safe parameter update */
    /* Khóa mutex để đảm bảo an toàn truy cập dữ liệu giữa các task */
    xSemaphoreTake(uart_data_mutex, portMAX_DELAY);

    switch (key)
    {
        case 'V':   /* Velocity (m/s * 1000): e.g. V800 -> 0.800 m/s / Vận tốc tiến/lùi */
            uart_cmd.velocity = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK V=%.3f\r\n", uart_cmd.velocity);
            break;

        case 'X':   /* Position X (m * 1000): e.g. X1000 -> 1.000 m / Vị trí mục tiêu X */
            uart_cmd.position_x = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK X=%.3f\r\n", uart_cmd.position_x);
            break;

        case 'Y':   /* Yaw turn rate (rad/s * 1000): e.g. Y500 -> 0.500 rad/s / Tốc độ xoay yaw */
            uart_cmd.yaw_turn = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK Y=%.3f\r\n", uart_cmd.yaw_turn);
            break;

        case 'H':   /* Leg height (m * 1000): e.g. H80 -> 0.080 m / Chiều cao chân */
            uart_cmd.leg_height = (float)value * 0.001f;
            /* Clamp height to safe physical range [60mm, 210mm] identical to PS2 limits */
            /* Giới hạn hành trình an toàn [60mm, 210mm] tương tự tay cầm PS2 */
            if (uart_cmd.leg_height < 0.060f) uart_cmd.leg_height = 0.060f;
            if (uart_cmd.leg_height > 0.210f) uart_cmd.leg_height = 0.210f;
            snprintf(tx, sizeof(tx), "OK H=%.3f\r\n", uart_cmd.leg_height);
            break;

        case 'R':   /* Roll angle (rad * 1000): e.g. R100 -> 0.100 rad / Góc nghiêng roll thân xe */
            uart_cmd.roll_angle = (float)value * 0.001f;
            snprintf(tx, sizeof(tx), "OK R=%.3f\r\n", uart_cmd.roll_angle);
            break;

        case 'L':   /* Line tracking error (-1000 to 1000): e.g. L-500 -> left deviation / Sai lệch line */
            uart_cmd.line_error = (float)value * 0.001f;
            /* Clamp normalized range [-1.0, 1.0] / Giới hạn chuẩn hóa [-1.0, 1.0] */
            if (uart_cmd.line_error < -1.0f) uart_cmd.line_error = -1.0f;
            if (uart_cmd.line_error > 1.0f) uart_cmd.line_error = 1.0f;
            snprintf(tx, sizeof(tx), "OK L=%.3f\r\n", uart_cmd.line_error);
            break;

        case 'J':   /* Jump trigger (0 or 1) / Kích hoạt bật nhảy */
            uart_cmd.jump_trigger = (value != 0) ? 1 : 0;
            snprintf(tx, sizeof(tx), "OK J=%d\r\n", uart_cmd.jump_trigger);
            break;

        case 'E':   /* Master enable/disable (0 or 1) / Bật hoặc tắt hệ thống điều khiển */
            uart_cmd.enable = (value != 0) ? 1 : 0;
            snprintf(tx, sizeof(tx), "OK E=%d\r\n", uart_cmd.enable);
            break;

        case 'B':   /* Buzzer control: B0=off, B1=beep, B2=alarm / Điều khiển còi chip */
            if (value == 0)
            {
                /* Turn off buzzer / Tắt còi */
                HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
                snprintf(tx, sizeof(tx), "OK B=OFF\r\n");
            }
            else if (value == 1)
            {
                /* Single beep - host will turn off after timeout / Kêu 1 tiếng bíp ngắn */
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);
                HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
                snprintf(tx, sizeof(tx), "OK B=BEEP\r\n");
            }
            else if (value == 2)
            {
                /* Continuous alarm / Báo động kêu liên tục */
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);
                HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
                snprintf(tx, sizeof(tx), "OK B=ALARM\r\n");
            }
            else
            {
                snprintf(tx, sizeof(tx), "ERR B=%d\r\n", value);
            }
            break;

        case 'C':   /* Direct PWM CCR2 compare register setting / Đặt giá trị chu kỳ xung CCR2 */
        {
            uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim12);

            if (value < 0) value = 0;
            if (value > (int)arr) value = arr;

            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, value);
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            snprintf(tx, sizeof(tx), "OK CCR2=%d\r\n", value);
            break;
        }

        default:
            xSemaphoreGive(uart_data_mutex);
            uart_send_str("ERR CMD\r\n");
            return;
    }

    xSemaphoreGive(uart_data_mutex);
    uart_send_str(tx);
}

/**
 * @brief Send real-time odometry feedback to Host / Python / ROS
 *        Gửi dữ liệu phản hồi đo đạc thời gian thực về máy tính / Python / ROS
 * @param chassis Pointer to chassis structure / Con trỏ cấu trúc chassis
 * @param ins Pointer to INS attitude structure / Con trỏ cấu trúc tư thế INS
 * @note  Format: "F<velocity>,<position>,<yaw>,<yaw_rate>\r\n"
 *        Example: "F0.523,1.234,0.785,0.100\r\n"
 */
void uart_send_feedback(chassis_t *chassis, INS_t *ins)
{
    char tx[64];

    /* Format telemetry string:
     * - chassis->v_filter:  Kalman-filtered forward velocity (m/s) / Vận tốc lọc Kalman
     * - chassis->x_filter:  Estimated longitudinal position (m) / Vị trí ước tính dọc trục
     * - ins->YawTotalAngle: Continuous accumulated yaw angle (rad) / Góc yaw tích lũy
     * - ins->Gyro[2]:       Yaw angular velocity (rad/s) / Vận tốc góc quay yaw
     */
    snprintf(tx, sizeof(tx), "F%.3f,%.3f,%.3f,%.3f\r\n",
             chassis->v_filter,
             chassis->x_filter,
             ins->YawTotalAngle,
             ins->Gyro[2]);

    uart_send_str(tx);
}

/**
 * @brief Transmit null-terminated string via USART1 with thread safety
 *        Truyền chuỗi kết thúc bằng null qua USART1 có bảo vệ bằng mutex
 * @param str Pointer to string / Con trỏ chuỗi cần truyền
 */
void uart_send_str(const char *str)
{
    /* Take TX mutex to ensure atomic string transmission without interleaving */
    /* Lấy TX mutex để đảm bảo truyền chuỗi nguyên tử, tránh xen kẽ giữa các task */
    xSemaphoreTake(uart_tx_mutex, portMAX_DELAY);

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)str,
                      strlen(str),
                      HAL_MAX_DELAY);

    xSemaphoreGive(uart_tx_mutex);
}
