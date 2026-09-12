/**
 ******************************************************************************
 * @file    BMI088Middleware.h
 * @brief   Tầng trung gian giao tiếp phần cứng SPI/GPIO cho cảm biến BMI088
 * @author  Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
 * @date    2024 - 2026
 * @note    Wheeled-Bipedal Jumping Robot (DM-jump) Firmware
 *          Target MCU: STM32H723VGT6 | FreeRTOS | Keil MDK-ARM
 * @link    https://github.com/nguyenbinh-shark/DM-jump
 * @website https://nguyenbinh-shark.github.io/
 *
 * Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * Distributed under the MIT License.
 ******************************************************************************
 */

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BMI088_USE_SPI

void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);

void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);

uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;
#endif

#ifdef __cplusplus
}
#endif

#endif /* BMI088MIDDLEWARE_H */
