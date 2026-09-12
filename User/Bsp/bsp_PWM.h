/**
 ******************************************************************************
 * @file    bsp_PWM.h
 * @brief   Board Support Package điều khiển xung PWM mạch sấy IMU
 *          IMU Thermal Heater PWM Control BSP Header (Bilingual EN/VI)
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

#ifndef __BSP_IMU_PWM_H
#define __BSP_IMU_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "tim.h"

/**
 * @brief  Thiết lập giá trị độ rộng xung PWM cho kênh Timer
 *         Set PWM duty cycle compare value for specified Timer channel
 * @param  tim_pwmHandle: Con trỏ cấu trúc Timer của HAL / Pointer to TIM_HandleTypeDef
 * @param  Channel:       Kênh Timer (TIM_CHANNEL_1 .. TIM_CHANNEL_4) / Timer channel
 * @param  value:         Giá trị so sánh CCR mục tiêu / Compare register value
 */
void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_PWM_H */
