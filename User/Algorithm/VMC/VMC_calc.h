/**
 ******************************************************************************
 * @file    VMC_calc.h
 * @brief   Tính toán động học cơ cấu 5 khâu và điều khiển lực ảo (Virtual Model Control)
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

#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "INS_task.h"

#define pi                  3.141592653589793f

/* Tham số bộ điều khiển PID chiều dài chân robot */
#define LEG_PID_KP          700.0f
#define LEG_PID_KI          0.0f
#define LEG_PID_KD          2500.0f
#define LEG_PID_MAX_OUT     130.0f  /*!< Giới hạn mô-men xoắn lớn nhất (130 N.m) */
#define LEG_PID_MAX_IOUT    0.0f

/**
 * @brief Cấu trúc dữ liệu quản lý động học và lực cơ cấu 5 khâu cho một chân robot
 */
typedef struct
{
    /* Kích thước hình học cố định của các thanh đòn (m) */
    float l5;               /*!< Chiều dài đoạn cơ sở gắn trên thân robot AE (m) */
    float l1;               /*!< Chiều dài đùi trước AB (m) */
    float l2;               /*!< Chiều dài bắp chân trước BC (m) */
    float l3;               /*!< Chiều dài bắp chân sau DC (m) */
    float l4;               /*!< Chiều dài đùi sau ED (m) */
    
    /* Toạ độ các khớp động học */
    float XB, YB;           /*!< Toạ độ khớp B (khớp gối trước) */
    float XD, YD;           /*!< Toạ độ khớp D (khớp gối sau) */
    float XC, YC;           /*!< Toạ độ điểm C (khớp mắt cá / tâm bánh xe) */
    
    /* Biến toạ độ cực ảo của chân robot */
    float L0;               /*!< Chiều dài chân ảo nối từ tâm đế đến điểm C (m) */
    float phi0;             /*!< Góc nghiêng chân ảo so với phương ngang (rad) */
    float alpha;            /*!< Góc bù alpha = pi/2 - phi0 */
    float d_alpha;          /*!< Vận tốc góc d_alpha/dt */
    
    float lBD;              /*!< Khoảng cách chéo giữa khớp B và D */
    
    float d_phi0;           /*!< Tốc độ thay đổi góc chân ảo (rad/s) */
    float last_phi0;        /*!< Giá trị góc phi0 ở chu kỳ trước */

    /* Các hệ số trung gian giải phương trình lượng giác khâu đóng */
    float A0, B0, C0;
    float phi2;             /*!< Góc nghiêng thanh truyền bắp chân trước BC */
    float phi3;             /*!< Góc nghiêng thanh truyền bắp chân sau DC */
    float phi1;             /*!< Góc quay động cơ đùi trước */
    float phi4;             /*!< Góc quay động cơ đùi sau */
    
    /* Các phần tử của ma trận chuyển vị Jacobian (J^T) */
    float j11, j12;
    float j21, j22;
    float torque_set[2];    /*!< Mô-men mục tiêu tính được cho 2 động cơ khớp: [Khớp trước, Khớp sau] (N.m) */

    /* Lực ảo trong không gian tác vụ Cartesian */
    float F0;               /*!< Lực ảo dọc trục chân (Virtual Normal Force) (N) */
    float Tp;               /*!< Mô-men ảo vuông góc trục chân (Virtual Tangential Torque) (N.m) */
    
    /* Biến trạng thái cân bằng động dùng cho LQR */
    float theta;            /*!< Góc pitch tổng hợp của chân đối với phương thẳng đứng */
    float d_theta;          /*!< Vận tốc góc d_theta/dt */
    float last_d_theta;
    float dd_theta;         /*!< Gia tốc góc d2_theta/dt2 */
    
    float d_L0;             /*!< Vận tốc co duỗi chiều dài chân dL0/dt (m/s) */
    float dd_L0;            /*!< Gia tốc co duỗi chiều dài chân d2L0/dt2 (m/s^2) */
    float last_L0;
    float last_d_L0;
    
    float FN;               /*!< Ước lượng phản lực pháp tuyến mặt đất (Ground Reaction Force) (N) */
    
    uint8_t first_flag;     /*!< Cờ đánh dấu chu kỳ tính toán đầu tiên */
    uint8_t leg_flag;       /*!< Cờ trạng thái chuyển động của chân */
} vmc_leg_t;

/* --- Khởi tạo và tính toán động học VMC --- */
void VMC_init(vmc_leg_t *vmc);

/* Giải bài toán thuận động học cho chân phải và chân trái */
void VMC_calc_1_right(vmc_leg_t *vmc, INS_t *ins, float dt);
void VMC_calc_1_left(vmc_leg_t *vmc, INS_t *ins, float dt);

/* Ánh xạ lực ảo (F0, Tp) sang mô-men khớp động cơ qua Jacobian J^T */
void VMC_calc_2(vmc_leg_t *vmc);

/* Thuật toán phát hiện tiếp đất / rời đất (Ground Contact Detection) */
uint8_t ground_detectionR(vmc_leg_t *vmc, INS_t *ins);
uint8_t ground_detectionL(vmc_leg_t *vmc, INS_t *ins);

/* Nội suy hệ số ma trận phản hồi LQR theo chiều dài chân L0 */
float LQR_K_calc(float *coe, float len);

#ifdef __cplusplus
}
#endif

#endif /* __VMC_CALC_H */
