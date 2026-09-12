/**
 ******************************************************************************
 * @file    chassisR_task.c
 * @brief   Hiện thực tác vụ điều khiển động cơ chân PHẢI và máy trạng thái nhảy qua CAN1
 *          Right Leg Motor Control & Jump State Machine Implementation (Bilingual EN/VI)
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

#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

/* Vector hệ số LQR chân phải / Right leg LQR feedback gain vector */
float LQR_K_R[12] = { 
   -5.11661395587923f, -0.696218875811963f, -2.21758581294137f, -1.99627529082690f,  1.58419592732791f, 0.204733182421384f,
    1.05342099102585f,  0.125737489379163f,  0.641533951717394f,  0.517183568761030f, 20.0319440884663f, 0.635996438625759f
};

/* Ma trận hệ số đa thức nội suy LQR bậc 3 theo chiều dài chân L0 / LQR Gain Polynomial Table */
float Poly_Coefficient[12][4] = {
    {-161.259861257070f, 109.536849434539f, -41.158450389303f, -0.317093938410f},
    {   2.378801114906f,  -2.041764288258f,  -3.284889190676f, -0.006700613564f},
    {-169.255081329858f,  90.015210428789f, -16.206399365452f, -1.212411773750f},
    { -87.407564519454f,  46.784956583810f,  -9.591053978779f, -1.257240855724f},
    {-815.326088560862f, 492.689732072248f, -109.029233242912f, 10.434322873478f},
    { -18.580455294741f,  13.391605893996f,  -3.764584564372f,  0.594544483498f},
    { -94.379503773934f,  72.751171405232f, -21.372256720865f,  3.417208515707f},
    {  -5.906969017545f,   4.743882223006f,  -1.479937927643f,  0.308086768436f},
    {-442.425497045011f, 266.496891626985f, -58.446703969453f,  5.314260650995f},
    {-462.161539531547f, 267.190926527127f, -55.513156813776f,  4.725333258092f},
    {1758.555751801603f,-937.419487883650f, 169.683061508561f,  9.469338375809f},
    {  69.876296668616f, -39.577598374806f,   7.923421244298f,  0.133008226300f}
};

vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;                                                                
chassis_t chassis_move;
                                                            
PidTypeDef LegR_Pid;
PidTypeDef Tp_Pid;
PidTypeDef Turn_Pid;
PidTypeDef RollR_Pid;

uint32_t CHASSR_TIME = 1;

/**
 * @brief  Vòng lặp tác vụ điều khiển chân phải / Right chassis control task loop
 */
void ChassisR_task(void)
{
    while (INS.ins_flag == 0)
    {
        osDelay(1);    
    }

    ChassisR_init(&chassis_move, &right, &LegR_Pid);
    Pensation_init(&Tp_Pid, &Turn_Pid);
    roll_pid_init(&RollR_Pid);

    while (1)
    {    
        chassisR_feedback_update(&chassis_move, &right, &INS);
        chassisR_control_loop(&chassis_move, &right, &INS, LQR_K_R, &LegR_Pid);
   
        if (chassis_move.start_flag == 1)    
        {
            /* Gửi lệnh qua FDCAN1 cho chân phải / Send commands over FDCAN1 */
            mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[1]);
            osDelay(CHASSR_TIME);
            mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[0]);
            osDelay(CHASSR_TIME);
            mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);
            osDelay(CHASSR_TIME);
        }
        else if (chassis_move.start_flag == 0)    
        {
            mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSR_TIME);
            mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSR_TIME);
            mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            osDelay(CHASSR_TIME);
        }
    }
}

/**
 * @brief  Khởi tạo ID và cấu hình động cơ chân phải / Initialize right leg motors & kinematics
 */
void ChassisR_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legr)
{
    const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

    joint_motor_init(&chassis->joint_motor[0], 6, MIT_MODE); /* Khớp sau TX ID: 6 */
    joint_motor_init(&chassis->joint_motor[1], 8, MIT_MODE); /* Khớp trước TX ID: 8 */
    wheel_motor_init(&chassis->wheel_motor[0], 0, MIT_MODE); /* Bánh xe TX ID: 0 (hoặc 1) */
    
    VMC_init(vmc);
    PID_init(legr, PID_POSITION, legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

    /* Kích hoạt động cơ qua CAN1 / Enable motors on CAN1 */
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, chassis->wheel_motor[0].para.id, chassis->wheel_motor[0].mode);
        osDelay(1);
    }
}

/**
 * @brief  Khởi tạo các bộ PID bù chống choạc chân và hướng / Initialize compensation PIDs
 */
void Pensation_init(PidTypeDef *Tp, PidTypeDef *turn)
{
    const static float tp_pid[3]   = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
    const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
    
    PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);
    PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

/**
 * @brief  Khởi tạo bộ PID nghiêng thân Roll / Initialize Roll PID
 */
void roll_pid_init(PidTypeDef *roll_pid)
{
    const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
    PID_init(roll_pid, PID_POSITION, roll, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
}

/**
 * @brief  Cập nhật dữ liệu góc khớp và tư thế IMU chân phải / Update right feedback states
 */
void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
    vmc->phi1 = pi / 2.0f + chassis->joint_motor[0].para.pos;
    vmc->phi4 = pi / 2.0f + chassis->joint_motor[1].para.pos;
        
    chassis->myPithR     = ins->Pitch;
    chassis->myPithGyroR = ins->Gyro[0];
    
    chassis->total_yaw = ins->YawTotalAngle;
    chassis->roll      = ins->Roll;
    chassis->theta_err = 0.0f - (vmc->theta + left.theta);
    
    /* Tự động xóa cờ ngã nếu góc pitch trở lại vùng an toàn (±30 độ) / Reset recovery flag */
    if (ins->Pitch < (3.14159265f / 6.0f) && ins->Pitch > (-3.14159265f / 6.0f))
    {
        chassis->recover_flag = 0;
    }
}

uint32_t count_roll = 0; 
uint8_t right_flag = 0;
extern uint8_t left_flag;

/**
 * @brief  Tính toán vòng lặp điều khiển LQR & VMC chân phải / Right leg control loop computation
 */
void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcr, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
    VMC_calc_1_right(vmcr, ins, ((float)CHASSR_TIME) * 3.0f / 1000.0f);
    
    for (int i = 0; i < 12; i++)
    {
        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0);    
    }
        
    /* Tính bù xoay hướng Yaw bằng PD trực tiếp với con quay / Direct gyro PD for stable yaw */
    chassis->turn_T = Turn_Pid.Kp * (chassis->turn_set - chassis->total_yaw) - Turn_Pid.Kd * ins->Gyro[2];

    /* Tính bù lực chống choạc chân / Anti-splay compensation */
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f);
    
    /* Mô-men bánh xe chân phải / Right wheel motor torque */
    chassis->wheel_motor[0].wheel_T = (LQR_K[0] * (vmcr->theta - 0.0f)
                                     + LQR_K[1] * (vmcr->d_theta - 0.0f)
                                     + LQR_K[2] * (chassis->x_filter - chassis->x_set)
                                     + LQR_K[3] * (chassis->v_filter - chassis->v_set)
                                     + LQR_K[4] * (chassis->myPithR - 0.0f)
                                     + LQR_K[5] * (chassis->myPithGyroR - 0.0f));
    
    /* Mô-men ảo vuông góc trục chân / Virtual tangential torque Tp */                
    vmcr->Tp = (LQR_K[6] * (vmcr->theta - 0.0f)
              + LQR_K[7] * (vmcr->d_theta - 0.0f)
              + LQR_K[8] * (chassis->x_filter - chassis->x_set)
              + LQR_K[9] * (chassis->v_filter - chassis->v_set)
              + LQR_K[10] * (chassis->myPithR - 0.0f)
              + LQR_K[11] * (chassis->myPithGyroR - 0.0f));
                
    chassis->wheel_motor[0].wheel_T = chassis->wheel_motor[0].wheel_T - chassis->turn_T;
    mySaturate(&chassis->wheel_motor[0].wheel_T, -2.0f, 2.0f);    
    
    vmcr->Tp = vmcr->Tp + chassis->leg_tp;

    chassis->now_roll_set = PID_Calc(&RollR_Pid, chassis->roll, chassis->roll_set);

    jump_loop_r(chassis, vmcr, leg);
        
    right_flag = ground_detectionR(vmcr, ins);
     
    if (chassis->recover_flag == 0)        
    {
        if (right_flag == 1 && left_flag == 1 && vmcr->leg_flag == 0)
        {
            chassis->wheel_motor[0].wheel_T = 0.0f;
            vmcr->Tp = LQR_K[6] * (vmcr->theta - 0.0f) + LQR_K[7] * (vmcr->d_theta - 0.0f);

            chassis->x_filter = 0.0f;
            chassis->x_set = chassis->x_filter;
            chassis->turn_set = chassis->total_yaw;
            vmcr->Tp = vmcr->Tp + chassis->leg_tp;        
        }
        else
        {
            vmcr->leg_flag = 0;
        }
    }
    else if (chassis->recover_flag == 1)
    {
        vmcr->Tp = 0.0f;
    }     
     
    mySaturate(&vmcr->F0, -100.0f, 100.0f);
    
    VMC_calc_2(vmcr);

    mySaturate(&vmcr->torque_set[1], -7.0f, 7.0f);    
    mySaturate(&vmcr->torque_set[0], -7.0f, 7.0f);        
}

/**
 * @brief  Hàm giới hạn bão hòa biến thực / Slew saturation clamp
 */
void mySaturate(float *in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

/**
 * @brief  Máy trạng thái nhảy cho chân phải và đồng bộ toàn thân
 *         Right leg jumping state machine & whole-body jump synchronization
 * @note   Giai đoạn 0: Crouch -> Giai đoạn 1: Thrust -> Giai đoạn 2: Flight Retract -> Reset
 */
void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcr, PidTypeDef *leg)
{
    if (chassis->jump_flag == 1)
    {
        if (chassis->jump_status_r == 0)
        {
            /* Giai đoạn nén chân chuẩn bị nhảy / Crouch phase */
            vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.08f);
            if (vmcr->L0 < 0.10f)
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 1;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 1;
            }
        }
        else if (chassis->jump_status_r == 1)
        {
            /* Giai đoạn duỗi chân hết công suất / Maximum thrust phase */
            vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.21f);
            if (vmcr->L0 > 0.18f)
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 2;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 2;
            }
        }
        else if (chassis->jump_status_r == 2)
        {
            /* Giai đoạn thu chân trên không chuẩn bị tiếp đất / Flight retract phase */
            vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set);
            if (vmcr->L0 < (chassis->leg_right_set + 0.01f))
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 3;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 3;
            }
        }
        else
        {
            vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set);
        }

        /* Khi cả hai chân hoàn tất tiếp đất / Both legs landed: exit jump state */
        if (chassis->jump_status_r == 3 && chassis->jump_status_l == 3)
        {
            chassis->jump_flag = 0;
            chassis->jump_time_r = 0;
            chassis->jump_status_r = 0;
            chassis->jump_time_l = 0;
            chassis->jump_status_l = 0;
        }
    }
    else
    {
        /* Cân bằng tư thế bình thường / Normal standing balance */
        vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set) - chassis->now_roll_set;
    }
}
