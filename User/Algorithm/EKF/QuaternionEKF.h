/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @brief   Bộ lọc Kalman mở rộng Quaternion EKF ước lượng tư thế và bù trôi Gyro
 * @author  Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
 * @date    2024 - 2026
 * @note    Wheeled-Bipedal Jumping Robot (DM-jump) Firmware
 *          Target MCU: STM32H723VGT6 | FreeRTOS | Keil MDK-ARM
 * @link    https://github.com/nguyenbinh-shark/DM-jump
 *
 * Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * Distributed under the MIT License.
 ******************************************************************************
 */

#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kalman_filter.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * @brief Cấu trúc dữ liệu ước lượng trạng thái Quaternion EKF
 */
typedef struct
{
    uint8_t Initialized;            /*!< Trạng thái khởi tạo bộ lọc */
    KalmanFilter_t IMU_QuaternionEKF;/*!< Cấu trúc đối tượng Kalman Filter ma trận */
    uint8_t ConvergeFlag;           /*!< Cờ hội tụ trạng thái */
    uint8_t StableFlag;             /*!< Cờ trạng thái ổn định */
    uint64_t ErrorCount;            /*!< Bộ đếm số lần kiểm tra Chi-Square lỗi */
    uint64_t UpdateCount;           /*!< Tổng số chu kỳ cập nhật */

    float q[4];                     /*!< Quaternion tư thế ước lượng [q0, q1, q2, q3] */
    float GyroBias[3];              /*!< Độ lệch trôi tĩnh con quay ước lượng [bx, by, bz] (rad/s) */

    float Gyro[3];                  /*!< Vận tốc góc đo được sau khi bù bias */
    float Accel[3];                 /*!< Gia tốc tuyến tính đo được */

    float OrientationCosine[3];     /*!< Cosine chỉ hướng trọng trường */

    float accLPFcoef;               /*!< Hệ số lọc thông thấp cho gia tốc kế */
    float gyro_norm;                /*!< Độ lớn vector vận tốc góc */
    float accl_norm;                /*!< Độ lớn vector gia tốc */
    float AdaptiveGainScale;        /*!< Hệ số thích nghi Kalman Gain theo gia tốc ngoài */

    float Roll;                     /*!< Góc Roll (rad) */
    float Pitch;                    /*!< Góc Pitch (rad) */
    float Yaw;                      /*!< Góc Yaw (rad) */

    float YawTotalAngle;            /*!< Góc Yaw tích lũy không giới hạn [-inf, +inf] */

    float Q1;                       /*!< Hiệp phương sai nhiễu quá trình cập nhật Quaternion */
    float Q2;                       /*!< Hiệp phương sai nhiễu quá trình bù Gyro Bias */
    float R;                        /*!< Hiệp phương sai nhiễu đo lường gia tốc kế */

    float dt;                       /*!< Chu kỳ cập nhật bộ lọc (giây) */
    mat ChiSquare;                  /*!< Ma trận kiểm định Chi-Square phân kỳ */
    float ChiSquare_Data[1];
    float ChiSquareTestThreshold;   /*!< Ngưỡng phát hiện dị thường Chi-Square */
    float lambda;                   /*!< Hệ số làm mờ (Fading Factor) cho ma trận hiệp phương sai P */

    int16_t YawRoundCount;          /*!< Bộ đếm số vòng quay trục Yaw */
    float YawAngleLast;             /*!< Góc Yaw tại chu kỳ trước */
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;

/* --- Các hàm API khởi tạo và cập nhật EKF --- */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#ifdef __cplusplus
}
#endif

#endif /* _QUAT_EKF_H */
