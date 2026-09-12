/**
 ******************************************************************************
 * @file    INS_task.h
 * @brief   Tác vụ định hướng quán tính (INS) và hợp nhất dữ liệu tư thế 1000Hz (Bilingual EN/VI)
 *          Inertial Navigation System (INS) & 1000Hz Attitude Fusion Task Header
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

#ifndef __INS_TASK_H
#define __INS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1   /*!< Chu kỳ tác vụ INS: 1ms (1000Hz) / INS Task period 1ms */

/**
 * @brief Cấu trúc dữ liệu trạng thái định vị quán tính / INS state structure
 */
typedef struct
{
    float q[4];                 /*!< Quaternion tư thế không gian [q0, q1, q2, q3] / Attitude quaternion */

    float Gyro[3];              /*!< Tốc độ góc 3 trục (rad/s) / Angular velocity */
    float Accel[3];             /*!< Gia tốc đo thô 3 trục (m/s^2) / Raw measured acceleration */
    float MotionAccel_b[3];     /*!< Gia tốc chuyển động hệ thân xe (Body frame) / Motion accel in body frame */
    float MotionAccel_n[3];     /*!< Gia tốc chuyển động hệ quy chiếu thế giới (Navigation frame) / Motion accel in nav frame */

    float AccelLPF;             /*!< Hệ số lọc thông thấp gia tốc / Acceleration LPF coefficient */

    /* Các vector đơn vị hệ toạ độ gia tốc / Coordinate unit vectors */
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    /* Các góc Euler (rad) / Euler angles */
    float Roll;                 /*!< Góc nghiêng ngang thân (Roll) / Roll angle */
    float Pitch;                /*!< Góc nghiêng dọc thân (Pitch) / Pitch angle */
    float Yaw;                  /*!< Góc quay hướng (Yaw) / Yaw angle */
    float YawTotalAngle;        /*!< Góc Yaw tích lũy liên tục / Continuous unwrapped yaw angle */
    float YawAngleLast;         /*!< Góc Yaw chu kỳ trước / Previous yaw angle */
    float YawRoundCount;        /*!< Số vòng quay trục Yaw / Yaw revolution counter */
    
    float v_n;                  /*!< Vận tốc chuyển động trong hệ quy chiếu thế giới / Linear velocity in nav frame */
    float x_n;                  /*!< Toạ độ vị trí trong hệ quy chiếu thế giới / Linear position in nav frame */
    
    uint8_t ins_flag;           /*!< Cờ báo INS đã khởi động ổn định / INS ready convergence flag */
} INS_t;

/**
 * @brief Cấu trúc tham số IMU / IMU parameter structure
 */
typedef struct
{
    uint8_t flag;
    float scale[3];
    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

/* --- Các hàm API công khai / Public API functions --- */
void INS_Init(void);
void INS_task(void);

/* Chuyển đổi vector giữa hệ thân xe (Body) và hệ thế giới (Earth) qua Quaternion */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/* Chuyển đổi Radian sang Degree / Radian to degree conversion */
float rad_to_deg(float rad);

#ifdef __cplusplus
}
#endif

#endif /* __INS_TASK_H */
