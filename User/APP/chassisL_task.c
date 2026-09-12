/**
 ******************************************************************************
 * @file    chassisL_task.c
 * @brief   Hiện thực tác vụ điều khiển động cơ và cân bằng động chân TRÁI qua CAN2
 *          Left Leg Motor Actuation & Dynamic Balancing Control (Bilingual EN/VI)
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

#include "chassisL_task.h"
#include "fdcan.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "pid.h"

vmc_leg_t left;

/* Vector hệ số LQR chân trái / Left leg LQR feedback gain vector */
float LQR_K_L[12] = { 
   -5.11661395587923f, -0.696218875811963f, -2.21758581294137f, -1.99627529082690f,  1.58419592732791f, 0.204733182421384f,
    1.05342099102585f,  0.125737489379163f,  0.641533951717394f,  0.517183568761030f, 20.0319440884663f, 0.635996438625759f
};

extern float Poly_Coefficient[12][4];
extern chassis_t chassis_move;
PidTypeDef LegL_Pid;
extern INS_t INS;

uint32_t CHASSL_TIME = 1;

/**
 * @brief  Vòng lặp tác vụ điều khiển chân trái / Left chassis control task loop
 */
void ChassisL_task(void)
{
    while (INS.ins_flag == 0)
    {
        /* Đợi INS hội tụ ổn định / Wait for INS convergence */
        osDelay(1);    
    }    
    ChassisL_init(&chassis_move, &left, &LegL_Pid);
    
    while (1)
    {    
        chassisL_feedback_update(&chassis_move, &left, &INS);
        chassisL_control_loop(&chassis_move, &left, &INS, LQR_K_L, &LegL_Pid);
           
        if (chassis_move.start_flag == 1)    
        {
            /* Gửi lệnh mô-men điều khiển qua CAN2 / Send control torque commands via FDCAN2 */
            mit_ctrl(&hfdcan2, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, left.torque_set[1]);
            osDelay(CHASSL_TIME);
            mit_ctrl(&hfdcan2, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, left.torque_set[0]);
            osDelay(CHASSL_TIME);
            mit_ctrl2(&hfdcan2, chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_move.wheel_motor[1].wheel_T);
            osDelay(CHASSL_TIME);
        }
        else if (chassis_move.start_flag == 0)    
        {
            /* Chế độ dừng/thả lỏng động cơ / Motor disable/damping mode */
            mit_ctrl(&hfdcan2, chassis_move.joint_motor[3].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSL_TIME);
            mit_ctrl(&hfdcan2, chassis_move.joint_motor[2].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSL_TIME);
            mit_ctrl2(&hfdcan2, chassis_move.wheel_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSL_TIME);
        }
    }
}

/**
 * @brief  Khởi tạo ID và cấu hình động cơ chân trái / Initialize left leg motors & kinematics
 */
void ChassisL_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legl)
{
    const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

    joint_motor_init(&chassis->joint_motor[2], 6, MIT_MODE); /* Khớp sau TX ID: 6 */
    joint_motor_init(&chassis->joint_motor[3], 8, MIT_MODE); /* Khớp trước TX ID: 8 */
    wheel_motor_init(&chassis->wheel_motor[1], 1, MIT_MODE); /* Bánh xe TX ID: 1 */
    
    VMC_init(vmc);
    PID_init(legl, PID_POSITION, legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    /* Kích hoạt động cơ qua CAN2 / Enable motors on CAN2 */
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan2, chassis->joint_motor[3].para.id, chassis->joint_motor[3].mode);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan2, chassis->joint_motor[2].para.id, chassis->joint_motor[2].mode);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan2, chassis->wheel_motor[1].para.id, chassis->wheel_motor[1].mode);
        osDelay(1);
    }
}

/**
 * @brief  Cập nhật trạng thái góc khớp và tư thế IMU chân trái / Update feedback states
 */
void chassisL_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
    vmc->phi1 = pi / 2.0f + chassis->joint_motor[2].para.pos;
    vmc->phi4 = pi / 2.0f + chassis->joint_motor[3].para.pos;
        
    chassis->myPithL     = 0.0f - ins->Pitch;
    chassis->myPithGyroL = 0.0f - ins->Gyro[0];
}

extern uint8_t right_flag;
uint8_t left_flag;

/**
 * @brief  Tính toán vòng lặp điều khiển LQR & VMC chân trái / Left leg control loop computation
 */
void chassisL_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
    VMC_calc_1_left(vmcl, ins, ((float)CHASSL_TIME) * 3.0f / 1000.0f);
    
    /* Nội suy hệ số LQR theo chiều dài chân L0 / Interpolate LQR gains */
    for (int i = 0; i < 12; i++)
    {
        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcl->L0);    
    }
            
    /* Mô-men xoắn bánh xe / Left wheel motor LQR torque */
    chassis->wheel_motor[1].wheel_T = (LQR_K[0] * (vmcl->theta - 0.0f)
                                     + LQR_K[1] * (vmcl->d_theta - 0.0f)
                                     + LQR_K[2] * (chassis->x_set - chassis->x_filter)
                                     + LQR_K[3] * (chassis->v_set - chassis->v_filter)
                                     + LQR_K[4] * (chassis->myPithL - 0.0f)
                                     + LQR_K[5] * (chassis->myPithGyroL - 0.0f));
    
    /* Mô-men ảo vuông góc trục chân / Virtual tangential torque Tp */            
    vmcl->Tp = (LQR_K[6] * (vmcl->theta - 0.0f)
              + LQR_K[7] * (vmcl->d_theta - 0.0f)
              + LQR_K[8] * (chassis->x_set - chassis->x_filter)
              + LQR_K[9] * (chassis->v_set - chassis->v_filter)
              + LQR_K[10] * (chassis->myPithL - 0.0f)
              + LQR_K[11] * (chassis->myPithGyroL - 0.0f));
            
    chassis->wheel_motor[1].wheel_T = chassis->wheel_motor[1].wheel_T - chassis->turn_T;
    mySaturate(&chassis->wheel_motor[1].wheel_T, -2.0f, 2.0f);    
    
    vmcl->Tp = vmcl->Tp + chassis->leg_tp;

    jump_loop_l(chassis, vmcl, leg);     

    left_flag = ground_detectionL(vmcl, ins);
    
    if (chassis->recover_flag == 0)    
    {
        if (left_flag == 1 && right_flag == 1 && vmcl->leg_flag == 0)
        {
            /* Đang bay trên không / Both legs airborne: suppress wheel torque */
            chassis->wheel_motor[1].wheel_T = 0.0f;
            vmcl->Tp = LQR_K[6] * (vmcl->theta - 0.0f) + LQR_K[7] * (vmcl->d_theta - 0.0f);
            
            chassis->x_filter = 0.0f;
            chassis->x_set = chassis->x_filter;
            chassis->turn_set = chassis->total_yaw;
            vmcl->Tp = vmcl->Tp + chassis->leg_tp;        
        }
        else
        {
            vmcl->leg_flag = 0;            
        }
    }
    else if (chassis->recover_flag == 1)
    {
        vmcl->Tp = 0.0f;
    }
    
    mySaturate(&vmcl->F0, -100.0f, 100.0f);
    
    /* Ánh xạ sang mô-men khớp / Map to joint torques via Jacobian J^T */
    VMC_calc_2(vmcl);
    
    mySaturate(&vmcl->torque_set[1], -7.0f, 7.0f);    
    mySaturate(&vmcl->torque_set[0], -7.0f, 7.0f);    
}

/**
 * @brief  Máy trạng thái nhảy cho chân trái / Left leg jumping state machine
 * @note   Giai đoạn 0: Co chân nén lò xo (Crouch) -> Giai đoạn 1: Đẩy tối đa (Thrust) -> Giai đoạn 2: Thu chân (Flight) -> Tiếp đất
 */
void jump_loop_l(chassis_t *chassis, vmc_leg_t *vmcl, PidTypeDef *leg)
{
    if (chassis->jump_flag == 1)
    {
        if (chassis->jump_status_l == 0)
        {
            /* Giai đoạn nén chân chuẩn bị nhảy / Crouch phase: L = 0.08m */
            vmcl->F0 = Mg / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, 0.08f);
            if (vmcl->L0 < 0.10f)
            {
                chassis->jump_time_l++;
            }
        }
        else if (chassis->jump_status_l == 1)
        {
            /* Giai đoạn đạp chân hết công suất / Maximum thrust phase: L = 0.21m */
            vmcl->F0 = Mg / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, 0.21f);
            if (vmcl->L0 > 0.18f)
            {
                chassis->jump_time_l++;
            }
        }
        else if (chassis->jump_status_l == 2)
        {
            /* Giai đoạn thu chân trên không / Airborne retract phase */
            vmcl->F0 = Mg / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_right_set);
            if (vmcl->L0 < (chassis->leg_right_set + 0.01f))
            {
                chassis->jump_time_l++;
            }
        }
        else
        {
            vmcl->F0 = Mg / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_left_set);
        }
    }
    else
    {
        /* Trạng thái cân bằng bình thường / Normal standing balance */
        vmcl->F0 = Mg / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_left_set) + chassis->now_roll_set;
    }
}
