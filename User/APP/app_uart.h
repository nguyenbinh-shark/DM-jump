/**
 * @file      app_uart.h
 * @brief     UART command interface and telemetry feedback header
 *            Header xử lý giao tiếp lệnh UART và truyền dữ liệu phản hồi đo đạc
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
 *   Provides definitions, data structures, and function prototypes for the high-
 *   speed UART communication interface (USART1 @ 115200 baud). Handles ASCII
 *   motion commands (velocity, position, height, roll, jump, enable, buzzer)
 *   and transmits real-time state feedback to the host PC / ROS controller.
 * 
 * Tiếng Việt:
 *   Cung cấp định nghĩa, cấu trúc dữ liệu và nguyên mẫu hàm cho giao tiếp UART
 *   tốc độ cao (USART1 @ 115200 baud). Xử lý giải mã lệnh ASCII (vận tốc, vị trí,
 *   chiều cao chân, góc roll, bật nhảy, bật/tắt, còi báo) và gửi dữ liệu phản hồi
 *   trạng thái thời gian thực về máy tính điều khiển / ROS.
 * ==============================================================================
 */

#ifndef APP_UART_H
#define APP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <stdint.h>
#include "chassisR_task.h"

/* Exported variables --------------------------------------------------------*/
extern uint8_t uart1_rx_byte; /**< Single byte buffer for UART RX interrupt / Bộ đệm 1 byte ngắt UART RX */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief UART command structure / Cấu trúc dữ liệu lệnh nhận qua UART
 */
typedef struct
{
    float velocity;       /**< Target forward/backward velocity in m/s (cmd 'V') / Vận tốc tiến/lùi mục tiêu (m/s) */
    float position_x;     /**< Target X position in m (cmd 'X') / Vị trí X mục tiêu (m) */
    float yaw_turn;       /**< Target yaw turning rate in rad/s (cmd 'Y') / Tốc độ quay góc yaw mục tiêu (rad/s) */
    float leg_height;     /**< Target leg height in m (cmd 'H', 0.060m - 0.210m) / Chiều cao chân mục tiêu (m) */
    float roll_angle;     /**< Target body roll angle in rad (cmd 'R') / Góc nghiêng roll thân xe (rad) */
    float line_error;     /**< Normalized line following error [-1.0, 1.0] (cmd 'L') / Sai lệch bám line chuẩn hóa */
    uint8_t jump_trigger; /**< Trigger jump state machine (cmd 'J1') / Kích hoạt máy trạng thái bật nhảy */
    uint8_t enable;       /**< Master enable flag (cmd 'E1' / 'E0') / Cờ cho phép/khóa hệ thống điều khiển */
} UartCmd_t;

extern UartCmd_t uart_cmd;

/* RTOS synchronization objects / Đối tượng đồng bộ hóa RTOS */
extern SemaphoreHandle_t uart_data_mutex; /**< Mutex protecting uart_cmd data / Mutex bảo vệ dữ liệu uart_cmd */
extern SemaphoreHandle_t uart_tx_mutex;   /**< Mutex protecting UART TX transmission / Mutex bảo vệ truyền UART TX */
extern QueueHandle_t     uart1_rx_queue;  /**< FreeRTOS queue for incoming bytes / Hàng đợi nhận byte ngắt UART1 */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief FreeRTOS task processing incoming UART frames
 *        Task FreeRTOS xử lý các khung lệnh nhận từ UART
 * @param argument Task parameter / Tham số truyền vào task
 */
void UartTask(void *argument);

/**
 * @brief Parse and execute a single ASCII command frame
 *        Giải mã và thực thi một khung lệnh ASCII
 * @param frame Null-terminated string containing command / Chuỗi kết thúc bằng null chứa lệnh
 */
void parse_uart_frame(char *frame);

/**
 * @brief Thread-safe transmission of a null-terminated string via UART
 *        Hàm truyền chuỗi an toàn trong môi trường đa luồng qua UART
 * @param str Pointer to string to transmit / Con trỏ chuỗi cần truyền
 */
void uart_send_str(const char *str);

/**
 * @brief Transmit chassis and IMU odometry feedback frame to host PC / ROS
 *        Gửi khung dữ liệu phản hồi đo đạc chassis và IMU về máy tính / ROS
 * @param chassis Pointer to chassis state struct / Con trỏ cấu trúc trạng thái chassis
 * @param ins Pointer to INS state struct / Con trỏ cấu trúc trạng thái INS
 */
void uart_send_feedback(chassis_t *chassis, INS_t *ins);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_H */
