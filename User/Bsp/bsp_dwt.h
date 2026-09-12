/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @brief   Đo thời gian thực phân giải micro-giây bằng ARM Cortex-M DWT Cycle Counter
 *          High-Resolution Timing BSP using ARM Cortex-M DWT (Bilingual EN/VI)
 * @author  Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
 * @website https://nguyenbinh-shark.github.io/
 * @github  https://github.com/nguyenbinh-shark/DM-jump
 * @date    2024 - 2026
 * @note    Wheeled-Bipedal Jumping Robot (DM-jump) Firmware
 *          Target MCU: STM32H723VGT6 | FreeRTOS | Keil MDK-ARM
 *
 * Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * Distributed under the MIT License.
 ******************************************************************************
 */

#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"

/**
 * @brief Cấu trúc thời gian hệ thống độ chính xác cao / High-resolution system time
 */
typedef struct
{
    uint32_t s;     /*!< Giây / Seconds */
    uint16_t ms;    /*!< Mili-giây / Milliseconds */
    uint16_t us;    /*!< Micro-giây / Microseconds */
} DWT_Time_t;

/* --- Các hàm API đo thời gian và trì hoãn DWT / Public API functions --- */
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

extern DWT_Time_t SysTime;

#ifdef __cplusplus
}
#endif

#endif /* _BSP_DWT_H */
