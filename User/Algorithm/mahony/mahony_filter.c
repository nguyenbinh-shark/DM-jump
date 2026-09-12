/**
 ******************************************************************************
 * @file    mahony_filter.c
 * @brief   Hiện thực giải thuật lọc bù Mahony AHRS hợp nhất dữ liệu IMU 6 trục
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

#include "mahony_filter.h"

struct MAHONY_FILTER_t mahony_filter;

/**
 * @brief  Cập nhật ma trận quay 3x3 (Rotation Matrix) từ quaternion hiện tại
 * @param  mahony_filter: Con trỏ cấu trúc bộ lọc Mahony
 */
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float q1q1 = mahony_filter->q1 * mahony_filter->q1;
    float q2q2 = mahony_filter->q2 * mahony_filter->q2;
    float q3q3 = mahony_filter->q3 * mahony_filter->q3;

    float q0q1 = mahony_filter->q0 * mahony_filter->q1;
    float q0q2 = mahony_filter->q0 * mahony_filter->q2;
    float q0q3 = mahony_filter->q0 * mahony_filter->q3;
    float q1q2 = mahony_filter->q1 * mahony_filter->q2;
    float q1q3 = mahony_filter->q1 * mahony_filter->q3;
    float q2q3 = mahony_filter->q2 * mahony_filter->q3;

    mahony_filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony_filter->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    mahony_filter->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    mahony_filter->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    mahony_filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony_filter->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    mahony_filter->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    mahony_filter->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    mahony_filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/**
 * @brief  Nạp giá trị đo tức thời từ con quay và gia tốc kế vào bộ lọc
 * @param  mahony_filter: Con trỏ cấu trúc bộ lọc
 * @param  gyro:          Vận tốc góc đo được (rad/s)
 * @param  acc:           Gia tốc đo được (m/s^2)
 */
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc)
{
    mahony_filter->gyro = gyro;
    mahony_filter->acc = acc;
}

/**
 * @brief  Thực thi một bước lặp giải thuật Mahony AHRS (bù lỗi PI + tích phân Quaternion)
 * @param  mahony_filter: Con trỏ cấu trúc bộ lọc
 */
void mahony_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float normalise;
    float ex, ey, ez;
       
    /* 1. Chuẩn hóa vector gia tốc kế trọng trường */
    normalise = sqrtf(mahony_filter->acc.x * mahony_filter->acc.x +
                      mahony_filter->acc.y * mahony_filter->acc.y +
                      mahony_filter->acc.z * mahony_filter->acc.z);
    if (normalise > 0.0f)
    {
        mahony_filter->acc.x /= normalise;
        mahony_filter->acc.y /= normalise;   
        mahony_filter->acc.z /= normalise;

        /* 2. Tính tích có hướng giữa hướng trọng trường ước lượng và gia tốc đo được */
        ex = (mahony_filter->acc.y * mahony_filter->rMat[2][2] - mahony_filter->acc.z * mahony_filter->rMat[2][1]);
        ey = (mahony_filter->acc.z * mahony_filter->rMat[2][0] - mahony_filter->acc.x * mahony_filter->rMat[2][2]);
        ez = (mahony_filter->acc.x * mahony_filter->rMat[2][1] - mahony_filter->acc.y * mahony_filter->rMat[2][0]);
        
        /* 3. Tích phân sai số bù trôi con quay (Gyro Bias Integral) */
        mahony_filter->exInt += mahony_filter->Ki * ex * mahony_filter->dt;  
        mahony_filter->eyInt += mahony_filter->Ki * ey * mahony_filter->dt;
        mahony_filter->ezInt += mahony_filter->Ki * ez * mahony_filter->dt;
        
        /* 4. Điều chỉnh vận tốc góc bằng bộ phản hồi PI */
        mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->exInt;
        mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->eyInt;
        mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ezInt;
    }
    
    /* 5. Tích phân động học vi phân Quaternion Runge-Kutta bậc 1 */
    float q0Last = mahony_filter->q0;
    float q1Last = mahony_filter->q1;
    float q2Last = mahony_filter->q2;
    float q3Last = mahony_filter->q3;
    float halfT = mahony_filter->dt * 0.5f;

    mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
    mahony_filter->q1 += ( q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
    mahony_filter->q2 += ( q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
    mahony_filter->q3 += ( q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;
    
    /* 6. Chuẩn hoá lại Quaternion đơn vị */
    normalise = sqrtf(mahony_filter->q0 * mahony_filter->q0 +
                      mahony_filter->q1 * mahony_filter->q1 +
                      mahony_filter->q2 * mahony_filter->q2 +
                      mahony_filter->q3 * mahony_filter->q3);
    if (normalise > 0.0f)
    {
        mahony_filter->q0 /= normalise;
        mahony_filter->q1 /= normalise;
        mahony_filter->q2 /= normalise;
        mahony_filter->q3 /= normalise;
    }
    
    /* 7. Cập nhật ma trận quay cho chu kỳ tiếp theo */
    mahony_filter->RotationMatrix_update(mahony_filter);
}

/**
 * @brief  Quy đổi ma trận quay sang các góc Euler (Pitch, Roll, Yaw) theo chuẩn ZYX
 * @param  mahony_filter: Con trỏ cấu trúc bộ lọc
 */
void mahony_output(struct MAHONY_FILTER_t *mahony_filter)
{
    mahony_filter->pitch = -asinf(mahony_filter->rMat[2][0]); 
    mahony_filter->roll  = atan2f(mahony_filter->rMat[2][1], mahony_filter->rMat[2][2]);
    mahony_filter->yaw   = atan2f(mahony_filter->rMat[1][0], mahony_filter->rMat[0][0]);
}

/**
 * @brief  Khởi tạo các tham số hệ số Kp, Ki và liên kết con trỏ hàm API
 * @param  mahony_filter: Con trỏ cấu trúc bộ lọc
 * @param  Kp:            Hệ số tỉ lệ
 * @param  Ki:            Hệ số tích phân
 * @param  dt:            Thời gian lấy mẫu (giây)
 */
void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt)
{
    mahony_filter->Kp = Kp;
    mahony_filter->Ki = Ki;
    mahony_filter->dt = dt;
    mahony_filter->q0 = 1.0f;
    mahony_filter->q1 = 0.0f;
    mahony_filter->q2 = 0.0f;
    mahony_filter->q3 = 0.0f;
    mahony_filter->exInt = 0.0f;
    mahony_filter->eyInt = 0.0f;
    mahony_filter->ezInt = 0.0f;
    mahony_filter->mahony_input = mahony_input;
    mahony_filter->mahony_update = mahony_update;
    mahony_filter->mahony_output = mahony_output;    
    mahony_filter->RotationMatrix_update = RotationMatrix_update;
}
