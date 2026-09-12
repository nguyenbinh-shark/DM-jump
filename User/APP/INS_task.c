/**
 ******************************************************************************
 * @file    INS_task.c
 * @brief   Hiện thực tác vụ thu thập cảm biến BMI088 và lọc Mahony AHRS ở tần số 1000Hz
 *          1000Hz IMU Acquisition and Mahony AHRS Task Implementation (Bilingual EN/VI)
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

#include "INS_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "mahony_filter.h"
#include "chassisR_task.h"
#include <stdio.h>
#include "app_uart.h"

INS_t INS;
extern chassis_t chassis_move;
extern vmc_leg_t left;
extern vmc_leg_t right;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro, Accel;
float gravity[3] = {0, 0, 9.81f};

/* Bù góc Pitch hiệu chuẩn phần cứng / Hardware calibration offset (in radians) */
#define PITCH_OFFSET_DEG (-2.0f)
#define PITCH_OFFSET_RAD (PITCH_OFFSET_DEG * 0.0174532925f)

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;
float Pitch_deg;
float Roll_deg;
float Yaw_deg;

/**
 * @brief  Khởi tạo các tham số bộ lọc tư thế Mahony AHRS
 *         Initialize Mahony AHRS filter parameters
 */
void INS_Init(void)
{ 
    mahony_init(&mahony, 1.0f, 0.0f, 0.001f);
    INS.AccelLPF = 0.0089f;
}

/**
 * @brief  Vòng lặp tác vụ thời gian thực INS_task chạy ở tần số 1000Hz (chu kỳ 1ms)
 *         High-priority 1000Hz real-time attitude estimation task
 */
void INS_task(void)
{
    INS_Init();
     
    while (1)
    {  
        ins_dt = DWT_GetDeltaT(&INS_DWT_Count);
        mahony.dt = ins_dt;

        /* 1. Đọc dữ liệu từ cảm biến BMI088 qua SPI / Read IMU data via SPI */
        BMI088_Read(&BMI088);

        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        Accel.x = BMI088.Accel[0];
        Accel.y = BMI088.Accel[1];
        Accel.z = BMI088.Accel[2];

        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];
        Gyro.x = BMI088.Gyro[0];
        Gyro.y = BMI088.Gyro[1];
        Gyro.z = BMI088.Gyro[2];

        /* 2. Cập nhật giải thuật lọc Mahony AHRS / Run Mahony AHRS algorithm */
        mahony_input(&mahony, Gyro, Accel);
        mahony_update(&mahony);
        mahony_output(&mahony);
        RotationMatrix_update(&mahony);
                
        INS.q[0] = mahony.q0;
        INS.q[1] = mahony.q1;
        INS.q[2] = mahony.q2;
        INS.q[3] = mahony.q3;
       
        /* 3. Khử trọng trường và chuyển đổi hệ toạ độ gia tốc / Remove gravity vector & transform to body frame */
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++)
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
                                   + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 
        }

        /* Chuyển đổi gia tốc từ hệ thân xe sang hệ quy chiếu thế giới / Body frame to Navigation frame */
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);
        
        /* 4. Áp dụng ngưỡng vùng chết lọc nhiễu gia tốc / Apply deadband thresholds */
        if (fabsf(INS.MotionAccel_n[0]) < 0.02f)
        {
            INS.MotionAccel_n[0] = 0.0f;
        }
        if (fabsf(INS.MotionAccel_n[1]) < 0.02f)
        {
            INS.MotionAccel_n[1] = 0.0f;
        }
        if (fabsf(INS.MotionAccel_n[2]) < 0.04f)
        {
            INS.MotionAccel_n[2] = 0.0f;
        }
 
        /* 5. Đợi bộ lọc hội tụ sau khoảng 3000 chu kỳ (~3 giây) / Wait for filter convergence */
        if (ins_time > 3000.0f)
        {
            INS.ins_flag = 1; /* Cờ báo INS sẵn sàng / INS ready flag */

            /* Trích xuất góc Euler (Pitch, Roll, Yaw) / Extract Euler angles */
            INS.Pitch = mahony.roll;
            INS.Roll  = mahony.pitch;
            INS.Yaw   = mahony.yaw;
            Pitch_deg = rad_to_deg(INS.Pitch);
            Roll_deg  = rad_to_deg(INS.Roll);
            Yaw_deg   = rad_to_deg(INS.Yaw);
            
            /* Xử lý vượt ngưỡng cuộn góc Yaw qua chu kỳ 2*PI / Handle Yaw angle wrap-around */
            if (INS.Yaw - INS.YawAngleLast > 3.14159265f)
            {
                INS.YawRoundCount--;
            }
            else if (INS.Yaw - INS.YawAngleLast < -3.14159265f)
            {
                INS.YawRoundCount++;
            }
            INS.YawTotalAngle = 6.2831853f * INS.YawRoundCount + INS.Yaw;
            INS.YawAngleLast = INS.Yaw;
        }
        else
        {
            ins_time++;
        }

        /* 6. Truyền thông tin giám sát qua UART ở tần số ~50Hz / Debug telemetry print via UART */
        static uint32_t last_print = 0;
        if (HAL_GetTick() - last_print >= 20)
        {
            char tx[200];
            snprintf(tx, sizeof(tx),
                     "P=%.2f R=%.2f Y=%.2f V=%.3f TL=%.2f TR=%.2f FL=%.1f FR=%.1f\r\n",
                     Pitch_deg,
                     Roll_deg,
                     Yaw_deg,
                     chassis_move.v_filter,
                     chassis_move.wheel_motor[1].para.tor,
                     chassis_move.wheel_motor[0].para.tor,
                     left.F0,
                     right.F0);
            uart_send_str(tx);
            last_print = HAL_GetTick();
        }

        osDelay(1);
    }
}

/**
 * @brief  Biến đổi vector 3 chiều từ hệ thân xe sang hệ thế giới qua Quaternion
 *         Transform 3D vector from Body Frame to Earth/Navigation Frame
 * @param  vecBF: Vector trong hệ thân xe / Vector in Body Frame
 * @param  vecEF: Vector trong hệ thế giới / Vector in Earth Frame
 * @param  q:     Quaternion định hướng / Orientation quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief  Biến đổi vector 3 chiều từ hệ thế giới sang hệ thân xe qua Quaternion
 *         Transform 3D vector from Earth/Navigation Frame to Body Frame
 * @param  vecEF: Vector trong hệ thế giới / Vector in Earth Frame
 * @param  vecBF: Vector trong hệ thân xe / Vector in Body Frame
 * @param  q:     Quaternion định hướng / Orientation quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief  Chuyển đổi góc từ đơn vị Radian sang Độ / Convert radians to degrees
 * @param  rad: Góc theo radian / Angle in radians
 * @retval Góc theo độ / Angle in degrees
 */
float rad_to_deg(float rad)
{
    return rad * 57.295779513f;
}
