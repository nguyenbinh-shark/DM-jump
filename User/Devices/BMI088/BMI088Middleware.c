/**
 ******************************************************************************
 * @file    BMI088Middleware.c
 * @brief   Hiện thực các hàm giao tiếp SPI và chân Chip Select (CS) cho BMI088
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

#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;

/**
 * @brief Kéo chân CS cảm biến gia tốc (Accel) xuống mức LOW để chọn chip
 */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Kéo chân CS cảm biến gia tốc (Accel) lên mức HIGH để bỏ chọn chip
 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Kéo chân CS con quay (Gyro) xuống mức LOW để chọn chip
 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Kéo chân CS con quay (Gyro) lên mức HIGH để bỏ chọn chip
 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Truyền nhận đồng thời 1 byte qua giao thức SPI với cảm biến BMI088
 * @param  txdata: Byte dữ liệu cần truyền
 * @retval Byte dữ liệu nhận về từ SPI bus
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
