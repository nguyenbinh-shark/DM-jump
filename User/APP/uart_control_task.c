/**
 * @file      uart_control_task.c
 * @brief     UART control task implementation
 *            Hiện thực task điều khiển robot qua giao tiếp UART
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
 *   FreeRTOS task running at 100Hz. Monitors and forwards external UART motion
 *   setpoints to the chassis control tasks when operating in remote PC/ROS mode.
 * 
 * Tiếng Việt:
 *   Task FreeRTOS chu kỳ 100Hz. Giám sát và chuyển tiếp các điểm đặt chuyển động
 *   từ UART ngoại vi tới các task điều khiển khung gầm khi hoạt động ở chế độ PC/ROS.
 * ==============================================================================
 */

#include "uart_control_task.h"
#include "app_uart.h"
#include "chassisR_task.h"
#include "cmsis_os.h"

/* External variables --------------------------------------------------------*/
extern UartCmd_t uart_cmd;
extern chassis_t chassis_move;

/**
 * @brief UART Control Task loop
 *        Vòng lặp chính của Task điều khiển UART
 * @param argument Unused task parameter / Tham số không sử dụng
 */
void UART_Control_Task(void const * argument)
{
    /* 100Hz periodic loop (10ms delay) / Vòng lặp chu kỳ 100Hz (trễ 10ms) */
    for (;;)
    {
        osDelay(10);
    }
}
