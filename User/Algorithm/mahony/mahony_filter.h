/**
 ******************************************************************************
 * @file    mahony_filter.h
 * @brief   Bộ lọc định hướng tư thế Mahony AHRS ước lượng Quaternion và góc Euler
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

#ifndef _MAHONY_FILTER_H
#define _MAHONY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include "stm32h7xx.h"
#include "arm_math.h"

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

/**
 * @brief Cấu trúc dữ liệu vector 3 trục kiểu float
 */
typedef struct Axis3f_t
{
    float x;
    float y;
    float z;
} Axis3f;

/**
 * @brief Cấu trúc quản lý trạng thái và con trỏ hàm bộ lọc Mahony AHRS
 */
struct MAHONY_FILTER_t
{
    /* Hệ số khuếch đại và tham số đầu vào */
    float Kp, Ki;                   /*!< Hệ số tỉ lệ Kp và tích phân Ki bù sai lệch */
    float dt;                       /*!< Chu kỳ lấy mẫu (giây) */
    Axis3f gyro, acc;               /*!< Giá trị đo con quay (rad/s) và gia tốc (m/s^2) */

    /* Biến trạng thái nội bộ */
    float exInt, eyInt, ezInt;      /*!< Thành phần tích phân sai số 3 trục */
    float q0, q1, q2, q3;           /*!< Quaternion định hướng không gian [w, x, y, z] */
    float rMat[3][3];               /*!< Ma trận quay hệ quy chiếu thân robot sang thế giới */

    /* Kết quả góc Euler đầu ra (rad) */
    float pitch, roll, yaw;

    /* Con trỏ hàm API */
    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

/* --- Các hàm API khởi tạo và cập nhật bộ lọc Mahony --- */
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);

#ifdef __cplusplus
}
#endif

#endif /* _MAHONY_FILTER_H */
