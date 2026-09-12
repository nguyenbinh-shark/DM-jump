/**
 * @file      uart_control_task.h
 * @brief     UART control task header
 *            Header cho task điều khiển qua giao tiếp UART
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
 *   Declares the FreeRTOS UART control task entry point and interfaces.
 * 
 * Tiếng Việt:
 *   Khai báo điểm khởi chạy task điều khiển UART FreeRTOS và các giao diện liên quan.
 * ==============================================================================
 */

#ifndef __UART_CONTROL_TASK_H
#define __UART_CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief UART control task entry point
 *        Điểm vào của task điều khiển UART
 * @param argument Task argument / Tham số của task
 */
void UART_Control_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif /* __UART_CONTROL_TASK_H */
