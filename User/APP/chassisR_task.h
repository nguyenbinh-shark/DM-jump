/**
 ******************************************************************************
 * @file    chassisR_task.h
 * @brief   Tác vụ điều khiển động cơ chân PHẢI, điều phối tư thế và nhảy (Bilingual EN/VI)
 *          Right Leg Actuation, Chassis Coordination & Jump Machine Task Header
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

#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

/* --- Tham số PID bù lực vuông góc Tp / Anti-splay Tp PID --- */
#define TP_PID_KP           10.0f
#define TP_PID_KI           0.0f 
#define TP_PID_KD           0.1f
#define TP_PID_MAX_OUT      2.0f
#define TP_PID_MAX_IOUT     0.0f

/* --- Tham số PID điều khiển góc quay hướng Yaw / Yaw Steering PID --- */
#define TURN_PID_KP         2.0f
#define TURN_PID_KI         0.0f 
#define TURN_PID_KD         0.2f
#define TURN_PID_MAX_OUT    1.0f
#define TURN_PID_MAX_IOUT   0.0f

/* --- Tham số PID cân bằng góc Roll / Roll Balance PID --- */
#define ROLL_PID_KP         100.0f
#define ROLL_PID_KI         0.0f 
#define ROLL_PID_KD         0.0f
#define ROLL_PID_MAX_OUT    100.0f
#define ROLL_PID_MAX_IOUT   0.0f

#define Mg                  17.658f     /*!< Trọng lực robot: m * g (N) / Gravity force */

/**
 * @brief Cấu trúc tổng thể điều phối chuyển động khung gầm robot / Main chassis coordinate structure
 */
typedef struct
{
    Joint_Motor_t joint_motor[4];   /*!< 4 động cơ khớp DM4310: [0,1] chân phải, [2,3] chân trái */
    Wheel_Motor_t wheel_motor[2];   /*!< 2 động cơ bánh xe DM6215: [0] phải, [1] trái */
    
    float v_set;                    /*!< Vận tốc đặt / Desired forward velocity (m/s) */
    float target_v;
    float x_set;                    /*!< Vị trí đặt / Desired linear position (m) */
    float turn_set;                 /*!< Góc quay đặt / Desired yaw angle (rad) */
    float target_turn;
    float leg_set;                  /*!< Chiều cao chân đặt / Desired leg length (m) */
    float leg_lx_set;
    float target_leg_lx_set;
    float leg_left_set;             /*!< Chiều dài đặt chân trái / Left leg setpoint */
    float leg_right_set;            /*!< Chiều dài đặt chân phải / Right leg setpoint */
    float last_leg_set;
    float last_leg_left_set;
    float last_leg_right_set;
    float roll_set;                 /*!< Góc nghiêng đặt / Desired roll angle (rad) */
    float roll_target;
    float now_roll_set;

    float v_filter;                 /*!< Vận tốc dài lọc Kalman / Filtered linear velocity (m/s) */
    float x_filter;                 /*!< Vị trí di chuyển lọc / Filtered linear position (m) */
    
    float myPithR;                  /*!< Góc pitch chân phải / Right pitch state */
    float myPithGyroR;
    float myPithL;                  /*!< Góc pitch chân trái / Left pitch state */
    float myPithGyroL;
    float roll;                     /*!< Góc roll thân xe / Body roll angle */
    float total_yaw;                /*!< Góc yaw tích lũy / Continuous yaw */
    float theta_err;                /*!< Sai lệch góc chân / Leg angle divergence error */
        
    float turn_T;                   /*!< Mô-men bù xoay hướng Yaw / Yaw steering torque compensation */
    float leg_tp;                   /*!< Mô-men bù chống choạc chân / Anti-splay compensation torque */
    
    uint8_t start_flag;             /*!< Cờ cho phép robot chạy / Enable robot motion flag */
    uint8_t recover_flag;           /*!< Cờ tự đứng dậy sau khi ngã / Self-righting recovery flag */
    uint32_t count_key;
    
    /* Máy trạng thái nhảy / Jumping state machine variables */
    uint8_t jump_flag;              /*!< Cờ kích hoạt chu trình nhảy / Jump trigger flag */
    float jump_leg;
    uint32_t jump_time_r;
    uint32_t jump_time_l;
    uint8_t jump_status_r;
    uint8_t jump_status_l;

    uint8_t control_mode;           /*!< 0 = Tay cầm PS2, 1 = Lệnh UART / Control mode */
} chassis_t;

/* --- Các hàm API điều khiển chân phải và khung gầm / Public APIs --- */
void ChassisR_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legr);
void ChassisR_task(void);
void Pensation_init(PidTypeDef *Tp, PidTypeDef *turn);
void mySaturate(float *in, float min, float max);
void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins);
void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcr, INS_t *ins, float *LQR_K, PidTypeDef *leg);
void roll_pid_init(PidTypeDef *roll_pid);
void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcr, PidTypeDef *leg);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSISR_TASK_H */
