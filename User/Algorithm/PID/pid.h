/**
 ******************************************************************************
 * @file    pid.h
 * @brief   Bộ điều khiển PID kinh điển (Vị trí & Số gia) chống bão hòa tích phân
 *          Classic PID Controller Header (Position & Delta) with Anti-Windup
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

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Chế độ tính toán PID / PID computation modes
 */
enum PID_MODE
{
    PID_POSITION = 0,   /*!< PID vị trí / Position PID mode */
    PID_DELTA           /*!< PID số gia / Incremental (velocity) PID mode */
};

/**
 * @brief Cấu trúc dữ liệu bộ điều khiển PID / PID controller structure
 */
typedef struct
{
    uint8_t mode;       /*!< Chế độ hoạt động / Operation mode */
    
    /* Hệ số bộ điều khiển / Gain coefficients */
    fp32 Kp;            /*!< Hệ số tỉ lệ / Proportional gain */
    fp32 Ki;            /*!< Hệ số tích phân / Integral gain */
    fp32 Kd;            /*!< Hệ số vi phân / Derivative gain */

    fp32 max_out;       /*!< Giới hạn bão hòa ngõ ra cực đại / Maximum output limit */
    fp32 max_iout;      /*!< Giới hạn chống bão hòa tích phân / Maximum integral anti-windup limit */

    fp32 set;           /*!< Giá trị mục tiêu / Target setpoint */
    fp32 fdb;           /*!< Giá trị đo phản hồi / Measured feedback */

    fp32 out;           /*!< Ngõ ra tổng hợp / Total output */
    fp32 Pout;          /*!< Thành phần tỉ lệ / Proportional output */
    fp32 Iout;          /*!< Thành phần tích phân / Integral output */
    fp32 Dout;          /*!< Thành phần vi phân / Derivative output */
    fp32 Dbuf[3];       /*!< Bộ đệm vi phân: [0] hiện tại, [1] trước, [2] cũ / Derivative history buffer */
    fp32 error[3];      /*!< Bộ đệm sai số: [0] hiện tại, [1] trước, [2] cũ / Error history buffer */
} PidTypeDef;

/* --- Các hàm API khởi tạo và tính toán PID / Public API functions --- */
void PID_init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
void PID_clear(PidTypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
