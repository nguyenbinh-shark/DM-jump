/**
 ******************************************************************************
 * @file    VMC_calc.c
 * @brief   Thuật toán động học thuận cơ cấu 5 khâu và điều khiển lực ảo (Virtual Model Control)
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

#include "VMC_calc.h"
#include <math.h>

/**
 * @brief  Khởi tạo kích thước hình học tiêu chuẩn cho cơ cấu chân 5 khâu đối xứng
 * @param  vmc: Con trỏ cấu trúc dữ liệu chân robot
 */
void VMC_init(vmc_leg_t *vmc)
{
    vmc->l5 = 0.08f;   /* Khoảng cách giữa 2 trục động cơ AE = 80mm */
    vmc->l1 = 0.075f;  /* Chiều dài đốt đùi trước AB = 75mm */
    vmc->l2 = 0.14f;   /* Chiều dài đốt bắp chân trước BC = 140mm */
    vmc->l3 = 0.14f;   /* Chiều dài đốt bắp chân sau DC = 140mm */
    vmc->l4 = 0.075f;  /* Chiều dài đốt đùi sau ED = 75mm */
}

/**
 * @brief  Giải động học thuận cho chân PHẢI (Forward Kinematics & State Calculation)
 * @note   Tính toán toạ độ mắt cá C, chiều dài chân L0, góc phi0, biến trạng thái LQR (theta, d_theta)
 * @param  vmc: Con trỏ cấu trúc chân phải
 * @param  ins: Con trỏ cấu trúc dữ liệu tư thế IMU
 * @param  dt:  Khoảng thời gian lấy mẫu (giây)
 */
void VMC_calc_1_right(vmc_leg_t *vmc, INS_t *ins, float dt)
{		
    static float PitchR = 0.0f;
    static float PithGyroR = 0.0f;
    PitchR = ins->Pitch;
    PithGyroR = ins->Gyro[0];

    /* 1. Toạ độ các khớp đầu gối B và D từ góc quay động cơ phi1 và phi4 */
    vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4);
    vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1);
    vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4);
    vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1);
        
    /* Khoảng cách chéo giữa hai khớp B và D */
    vmc->lBD = sqrtf((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

    /* 2. Giải phương trình hình học khâu đóng để tìm góc phi2 và phi3 */
    vmc->A0 = 2.0f * vmc->l2 * (vmc->XD - vmc->XB);
    vmc->B0 = 2.0f * vmc->l2 * (vmc->YD - vmc->YB);
    vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
    vmc->phi2 = 2.0f * atan2f((vmc->B0 + sqrtf(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), 
                              vmc->A0 + vmc->C0);			
    vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2), 
                       vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));

    /* 3. Toạ độ điểm tiếp đất C (mắt cá chân / tâm trục bánh xe) */
    vmc->XC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->YC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);

    /* Chiều dài chân ảo L0 và góc cực phi0 */
    vmc->L0 = sqrtf((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);
    vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f));
    vmc->alpha = pi / 2.0f - vmc->phi0;
    
    if (vmc->first_flag == 0)
    {
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
    }
    vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;
    vmc->d_alpha = 0.0f - vmc->d_phi0;
    
    /* Biến trạng thái góc nghiêng và vận tốc góc phục vụ cân bằng động LQR */
    vmc->theta = pi / 2.0f - PitchR - vmc->phi0;
    vmc->d_theta = (-PithGyroR - vmc->d_phi0);
    
    vmc->last_phi0 = vmc->phi0;

    /* Đạo hàm bậc 1 và bậc 2 của chiều dài chân L0 */
    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;
    
    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;
    
    vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

/**
 * @brief  Giải động học thuận cho chân TRÁI (Forward Kinematics & State Calculation)
 * @param  vmc: Con trỏ cấu trúc chân trái
 * @param  ins: Con trỏ cấu trúc dữ liệu tư thế IMU
 * @param  dt:  Khoảng thời gian lấy mẫu (giây)
 */
void VMC_calc_1_left(vmc_leg_t *vmc, INS_t *ins, float dt)
{		
    static float PitchL = 0.0f;
    static float PithGyroL = 0.0f;
    PitchL = 0.0f - ins->Pitch;
    PithGyroL = 0.0f - ins->Gyro[0];

    /* 1. Toạ độ các khớp đầu gối B và D */
    vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4);
    vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1);
    vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4);
    vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1);
        
    vmc->lBD = sqrtf((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

    /* 2. Giải phương trình hình học khâu đóng */
    vmc->A0 = 2.0f * vmc->l2 * (vmc->XD - vmc->XB);
    vmc->B0 = 2.0f * vmc->l2 * (vmc->YD - vmc->YB);
    vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
    vmc->phi2 = 2.0f * atan2f((vmc->B0 + sqrtf(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), 
                              vmc->A0 + vmc->C0);			
    vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2), 
                       vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));

    /* 3. Toạ độ điểm C và các biến toạ độ cực chân trái */
    vmc->XC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->YC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);
    vmc->L0 = sqrtf((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);
        
    vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f));		
    vmc->alpha = pi / 2.0f - vmc->phi0;
    
    if (vmc->first_flag == 0)
    {
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
    }
    vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;
    vmc->d_alpha = 0.0f - vmc->d_phi0;
    
    vmc->theta = pi / 2.0f - PitchL - vmc->phi0;
    vmc->d_theta = (-PithGyroL - vmc->d_phi0);
    
    vmc->last_phi0 = vmc->phi0;

    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;
    
    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;
    
    vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

/**
 * @brief  Tính toán ma trận Jacobian chuyển vị và quy đổi lực ảo (F0, Tp) thành mô-men khớp
 * @note   Công thức: [tau1, tau4]^T = J^T * [F0, Tp]^T
 * @param  vmc: Con trỏ cấu trúc dữ liệu chân robot
 */
void VMC_calc_2(vmc_leg_t *vmc)
{
    /* Tính các phần tử ma trận Jacobian chuyển vị */
    vmc->j11 = (vmc->l1 * arm_sin_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) / arm_sin_f32(vmc->phi3 - vmc->phi2);
    vmc->j12 = (vmc->l1 * arm_cos_f32(vmc->phi0 - vmc->phi3) * arm_sin_f32(vmc->phi1 - vmc->phi2)) / (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));
    vmc->j21 = (vmc->l4 * arm_sin_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) / arm_sin_f32(vmc->phi3 - vmc->phi2);
    vmc->j22 = (vmc->l4 * arm_cos_f32(vmc->phi0 - vmc->phi2) * arm_sin_f32(vmc->phi3 - vmc->phi4)) / (vmc->L0 * arm_sin_f32(vmc->phi3 - vmc->phi2));

    /* Mô-men xoắn cần tác động lên 2 động cơ khớp: Đùi trước và Đùi sau */
    vmc->torque_set[0] = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp;
    vmc->torque_set[1] = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp;
}

/**
 * @brief  Thuật toán phát hiện tiếp xúc mặt đất cho chân PHẢI (Ground Contact Detection)
 * @param  vmc: Con trỏ cấu trúc chân phải
 * @param  ins: Con trỏ dữ liệu IMU
 * @retval 1 nếu đang bay trên không (Airborne / Rời đất), 0 nếu đang tiếp đất
 */
uint8_t ground_detectionR(vmc_leg_t *vmc, INS_t *ins)
{
    /* Ước lượng phản lực pháp tuyến mặt đất FN */
    vmc->FN = vmc->F0 * arm_cos_f32(vmc->theta) + vmc->Tp * arm_sin_f32(vmc->theta) / vmc->L0 + 6.0f;
 
    if (vmc->FN < 5.0f)
    {
        return 1; /* Đang ở trên không */
    }
    else
    {
        return 0; /* Đang tiếp đất */
    }
}

/**
 * @brief  Thuật toán phát hiện tiếp xúc mặt đất cho chân TRÁI (Ground Contact Detection)
 * @param  vmc: Con trỏ cấu trúc chân trái
 * @param  ins: Con trỏ dữ liệu IMU
 * @retval 1 nếu đang bay trên không (Airborne / Rời đất), 0 nếu đang tiếp đất
 */
uint8_t ground_detectionL(vmc_leg_t *vmc, INS_t *ins)
{
    vmc->FN = vmc->F0 * arm_cos_f32(vmc->theta) + vmc->Tp * arm_sin_f32(vmc->theta) / vmc->L0 + 6.0f;
 
    if (vmc->FN < 5.0f)
    {
        return 1; /* Đang ở trên không */
    }
    else
    {
        return 0; /* Đang tiếp đất */
    }
}

/**
 * @brief  Nội suy đa thức bậc 3 cho hệ số ma trận phản hồi LQR theo chiều dài chân L0
 * @param  coe: Mảng 4 hệ số đa thức [a3, a2, a1, a0]
 * @param  len: Chiều dài chân tức thời L0 (m)
 * @retval Giá trị hệ số độ lợi LQR thu được
 */
float LQR_K_calc(float *coe, float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}
