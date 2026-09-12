/**
 ******************************************************************************
 * @file    controller.h
 * @brief   Bộ thư viện điều khiển nâng cao: Fuzzy PID, Feedforward, LDOB, TD (Bilingual EN/VI)
 *          Advanced Control Library: Fuzzy PID, Feedforward, LDOB, Tracking Differentiator
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

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* ==============================================================================
 * 1. FUZZY PID - BỘ ĐIỀU KHIỂN PID MỜ THÍCH NGHI / FUZZY ADAPTIVE PID
 * ============================================================================== */
#define NB -3   /*!< Negative Big (Âm lớn) */
#define NM -2   /*!< Negative Medium (Âm vừa) */
#define NS -1   /*!< Negative Small (Âm nhỏ) */
#define ZE  0   /*!< Zero (Không) */
#define PS  1   /*!< Positive Small (Dương nhỏ) */
#define PM  2   /*!< Positive Medium (Dương vừa) */
#define PB  3   /*!< Positive Big (Dương lớn) */

/**
 * @brief Cấu trúc luật mờ hiệu chỉnh tham số Kp, Ki, Kd / Fuzzy rule structure
 */
typedef __packed struct
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/* ==============================================================================
 * 2. PID CONTROL - BỘ ĐIỀU KHIỂN PID ĐA TÍNH NĂNG / ADVANCED PID CONTROLLER
 * ============================================================================== */
typedef enum pid_Improvement_e
{
    NONE                        = 0x00, /*!< PID tiêu chuẩn / Standard PID */
    Integral_Limit              = 0x01, /*!< Giới hạn tích phân chống bão hòa / Integral anti-windup */
    Derivative_On_Measurement   = 0x02, /*!< Đạo hàm theo biến đo tránh giật / Derivative on measurement */
    Trapezoid_Intergral         = 0x04, /*!< Tích phân hình thang / Trapezoidal integration */
    Proportional_On_Measurement = 0x08, /*!< Tỉ lệ theo biến đo / Proportional on measurement */
    OutputFilter                = 0x10, /*!< Lọc thông thấp đầu ra / Output lowpass filter */
    ChangingIntegrationRate     = 0x20, /*!< Tích phân biến thiên theo sai số / Variable integration rate */
    DerivativeFilter            = 0x40, /*!< Lọc thông thấp đạo hàm / Derivative lowpass filter */
    ErrorHandle                 = 0x80  /*!< Giám sát và xử lý sự cố kẹt động cơ / Motor stall error handling */
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked  = 0x01U
} ErrorType_e;

typedef __packed struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/**
 * @brief Cấu trúc đối tượng PID mở rộng / Extended PID controller structure
 */
typedef __packed struct pid_t
{
    float Ref;                  /*!< Giá trị đặt / Target reference */
    float Kp;                   /*!< Hệ số tỉ lệ / Proportional gain */
    float Ki;                   /*!< Hệ số tích phân / Integral gain */
    float Kd;                   /*!< Hệ số vi phân / Derivative gain */

    float Measure;              /*!< Giá trị đo / Measurement */
    float Last_Measure;
    float Err;                  /*!< Sai số e / Error */
    float Last_Err;
    float Last_ITerm;

    float Pout;                 /*!< Ngõ ra tỉ lệ P / Proportional output */
    float Iout;                 /*!< Ngõ ra tích phân I / Integral output */
    float Dout;                 /*!< Ngõ ra vi phân D / Derivative output */
    float ITerm;

    float Output;               /*!< Tín hiệu điều khiển tổng hợp / Total control output */
    float Last_Output;
    float Last_Dout;

    float MaxOut;               /*!< Giới hạn ngõ ra lớn nhất / Maximum output saturation */
    float IntegralLimit;        /*!< Giới hạn tích phân / Integral limit */
    float DeadBand;             /*!< Vùng chết / Deadband */
    float ControlPeriod;
    float CoefA;                /*!< Hệ số tích phân biến thiên A */
    float CoefB;                /*!< Hệ số tích phân biến thiên B */
    float Output_LPF_RC;        /*!< Hằng số thời gian lọc RC đầu ra / Output LPF RC */
    float Derivative_LPF_RC;    /*!< Hằng số thời gian lọc RC đạo hàm / Derivative LPF RC */

    uint16_t OLS_Order;
    Ordinary_Least_Squares_t OLS;

    uint32_t DWT_CNT;
    float dt;

    FuzzyRule_t *FuzzyRule;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*User_Func1_f)(struct pid_t *pid);
    void (*User_Func2_f)(struct pid_t *pid);
} PID_t;

void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,
    float kp,
    float ki,
    float kd,
    float A,
    float B,
    float output_lpf_rc,
    float derivative_lpf_rc,
    uint16_t ols_order,
    uint8_t improve);

float PID_Calculate(PID_t *pid, float measure, float ref);

/* ==============================================================================
 * 3. FEEDFORWARD CONTROL - BỘ ĐIỀU KHIỂN BÙ TIẾN / FEEDFORWARD CONTROLLER
 * ============================================================================== */
typedef __packed struct
{
    float c[3]; /*!< Hệ số hàm truyền: G(s) = 1/(c2*s^2 + c1*s + c0) */

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC;

    float Ref_dot;
    float Ref_ddot;
    float Last_Ref_dot;

    uint16_t Ref_dot_OLS_Order;
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;
} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/* ==============================================================================
 * 4. LINEAR DISTURBANCE OBSERVER - BỘ QUAN SÁT NHIỄU TUYẾN TÍNH (LDOB)
 * ============================================================================== */
typedef __packed struct
{
    float c[3]; /*!< Mô hình danh định đối tượng / Nominal plant inverse coefficients */

    float Measure;
    float Last_Measure;

    float u;    /*!< Tín hiệu điều khiển ngõ vào hệ thống / Plant input */

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC;

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;      /*!< Ước lượng nhiễu ngoại lực tác động / Estimated disturbance */
    float Output;           /*!< Tín hiệu bù nhiễu ngõ ra / Compensation output */
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/* ==============================================================================
 * 5. TRACKING DIFFERENTIATOR - BỘ ĐẠO HÀM BÁM MỀM HÀN KINH THANH (TD)
 * ============================================================================== */
typedef __packed struct
{
    float Input;

    float h0;       /*!< Bước lọc vi phân / Filter step parameter */
    float r;        /*!< Tham số tốc độ bám tín hiệu / Tracking speed factor */

    float x;        /*!< Tín hiệu bám làm mượt / Filtered signal */
    float dx;       /*!< Đạo hàm bậc 1 / First-order derivative */
    float ddx;      /*!< Đạo hàm bậc 2 / Second-order derivative */

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);

#ifdef __cplusplus
}
#endif

#endif /* _CONTROLLER_H */
