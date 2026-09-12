/**
 ******************************************************************************
 * @file    observe_task.c
 * @brief   Hiện thực tác vụ quan sát ước lượng vận tốc và toạ độ robot qua lọc Kalman
 *          Kinematic Velocity and Position State Observer Implementation (Bilingual EN/VI)
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

#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
#include "app_uart.h"

KalmanFilter_t vaEstimateKF;       /*!< Bộ lọc Kalman ước lượng vận tốc và vị trí / Velocity-position Kalman Filter */

float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};    /*!< Ma trận chuyển trạng thái với dt=3ms / State transition matrix */

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    /*!< Hiệp phương sai sai số ban đầu / Initial error covariance */

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 0.5f};    /*!< Hiệp phương sai nhiễu quá trình / Process noise covariance */

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                           0.0f,  100.0f}; /*!< Hiệp phương sai nhiễu đo lường / Measurement noise covariance */
                                                        
float vaEstimateKF_K[4];
                                                     
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};   /*!< Ma trận đo lường đơn vị / Identity measurement matrix */
                                                                                                                     
extern INS_t INS;        
extern chassis_t chassis_move;                                                                                                                     
extern vmc_leg_t right;            
extern vmc_leg_t left;    

float vel_acc[2]; 
uint32_t OBSERVE_TIME = 3;  /*!< Chu kỳ lấy mẫu 3ms (~333Hz) / Sampling time 3ms */
static uint32_t feedback_counter = 0;
#define FEEDBACK_DIVIDER 10

/**
 * @brief  Vòng lặp tác vụ quan sát trạng thái chuyển động của robot
 *         FreeRTOS State Observer Task Loop
 */
void Observe_task(void)
{
    /* Đợi hệ thống định vị quán tính hội tụ / Wait for INS convergence */
    while (INS.ins_flag == 0)
    {
        osDelay(1);    
    }

    static float wr, wl = 0.0f;
    static float vrb, vlb = 0.0f;
    static float aver_v = 0.0f;
        
    xvEstimateKF_Init(&vaEstimateKF);
    
    while (1)
    {  
        /* 1. Tính vận tốc góc và vận tốc dài của bánh phải trong hệ thân xe / Right wheel linear velocity */
        wr  = -chassis_move.wheel_motor[0].para.vel - INS.Gyro[0] + right.d_alpha;
        vrb = wr * 0.0603f + right.L0 * right.d_theta * arm_cos_f32(right.theta) + right.d_L0 * arm_sin_f32(right.theta);
        
        /* 2. Tính vận tốc góc và vận tốc dài của bánh trái trong hệ thân xe / Left wheel linear velocity */
        wl  = -chassis_move.wheel_motor[1].para.vel + INS.Gyro[0] + left.d_alpha;
        vlb = wl * 0.0603f + left.L0 * left.d_theta * arm_cos_f32(left.theta) + left.d_L0 * arm_sin_f32(left.theta);
        
        /* 3. Vận tốc tịnh tiến trung bình của thân xe / Average forward linear velocity */
        aver_v = (vrb - vlb) / 2.0f;

        /* 4. Cập nhật bộ lọc Kalman kết hợp gia tốc quán tính và vận tốc đo bánh xe / Kalman update */
        xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_n[1], aver_v);
        
        /* 5. Tích phân xác định vị trí di chuyển / Integrate linear position */
        chassis_move.v_filter = vel_acc[0];
        chassis_move.x_filter = chassis_move.x_filter + chassis_move.v_filter * ((float)OBSERVE_TIME / 1000.0f);
        
        feedback_counter++;
        if (feedback_counter >= FEEDBACK_DIVIDER)
        {
            feedback_counter = 0;
        }
        
        osDelay(OBSERVE_TIME);
    }
}

/**
 * @brief  Khởi tạo các ma trận cho bộ lọc Kalman quan sát vị trí và vận tốc
 *         Initialize position/velocity Kalman Filter matrices
 */
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);
    
    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

/**
 * @brief  Cập nhật phép đo gia tốc và vận tốc vào bộ lọc Kalman
 *         Update Kalman filter with latest acceleration and velocity measurements
 * @param  EstimateKF: Con trỏ cấu trúc Kalman Filter
 * @param  acc:        Gia tốc tuyến tính đo được (m/s^2)
 * @param  vel:        Vận tốc tuyến tính đo được (m/s)
 */
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{       
    EstimateKF->MeasuredVector[0] = vel;
    EstimateKF->MeasuredVector[1] = acc;
            
    Kalman_Filter_Update(EstimateKF);

    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}
