/**
 ******************************************************************************
 * @file    bsp_PWM.c
 * @brief   Hiện thực điều khiển xung PWM mạch sấy ổn nhiệt cảm biến IMU
 *          IMU Thermal Heater PWM Control BSP Implementation (Bilingual EN/VI)
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

#include "bsp_PWM.h"

/**
 * @brief  Thiết lập giá trị độ rộng xung PWM cho kênh Timer
 *         Set PWM duty cycle compare value for specified Timer channel
 * @param  tim_pwmHandle: Con trỏ cấu trúc Timer của HAL / Pointer to TIM_HandleTypeDef
 * @param  Channel:       Kênh Timer (TIM_CHANNEL_1 .. TIM_CHANNEL_4) / Timer channel
 * @param  value:         Giá trị so sánh CCR mục tiêu / Compare register value
 */
void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    /* Giới hạn giá trị không vượt quá chu kỳ tràn ARR / Clamp to ARR period */
    if (value > tim_pwmHandle->Instance->ARR)
    {
        value = tim_pwmHandle->Instance->ARR;
    }

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    default:
        break;
    }
}
