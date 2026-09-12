/**
 ******************************************************************************
 * @file    controller.c
 * @brief   Hiện thực thuật toán Fuzzy PID, Feedforward, LDOB và Tracking Differentiator
 *          Advanced Control Algorithms Implementation (Bilingual EN/VI)
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

#include "controller.h"

/* ==============================================================================
 * 1. FUZZY PID CONTROL - BẢNG LUẬT MỜ VÀ NỘI SUY THÍCH NGHI / FUZZY RULES & INTERPOLATION
 * ============================================================================== */

/* Bảng luật mờ hiệu chỉnh độ lợi Kp / Fuzzy Rule Base for Kp */
static float FuzzyRuleKpRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, PS,
    PM, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PM,
    PS, ZE, PS, PM, PM, PM, PB,
    ZE, ZE, PM, PM, PM, PB, PB
};

/* Bảng luật mờ hiệu chỉnh độ lợi Ki / Fuzzy Rule Base for Ki */
static float FuzzyRuleKiRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, ZE,
    PB, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PB,
    ZE, ZE, PS, PS, PM, PB, PB,
    ZE, ZE, PS, PM, PM, PB, PB
};

/* Bảng luật mờ hiệu chỉnh độ lợi Kd / Fuzzy Rule Base for Kd */
static float FuzzyRuleKdRAW[7][7] = {
    PS, PS, PB, PB, PB, PM, PS,
    PS, PS, PB, PM, PM, PS, ZE,
    ZE, PS, PM, PM, PS, PS, ZE,
    ZE, PS, PS, PS, PS, PS, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PB, PS, PS, PS, PS, PS, PB
};

/**
 * @brief  Khởi tạo bộ tham số mờ thích nghi / Initialize fuzzy adaptive PID rules
 */
void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    if (fuzzyRuleKp == NULL)
        fuzzyRule->FuzzyRuleKp = FuzzyRuleKpRAW;
    else
        fuzzyRule->FuzzyRuleKp = fuzzyRuleKp;

    if (fuzzyRuleKi == NULL)
        fuzzyRule->FuzzyRuleKi = FuzzyRuleKiRAW;
    else
        fuzzyRule->FuzzyRuleKi = fuzzyRuleKi;

    if (fuzzyRuleKd == NULL)
        fuzzyRule->FuzzyRuleKd = FuzzyRuleKdRAW;
    else
        fuzzyRule->FuzzyRuleKd = fuzzyRuleKd;

    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;

    fuzzyRule->DWT_CNT = 0;
}

/**
 * @brief  Nội suy giải mờ tính toán delta Kp, Ki, Kd / Defuzzification and gain interpolation
 */
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp;
    float eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex;
    int eRightIndex, ecRightIndex;

    fuzzyRule->dt = DWT_GetDeltaT((void *)fuzzyRule->DWT_CNT);

    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    /* Xác định chỉ số mờ hóa / Membership interval indices */
    eLeftIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2)));
    eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3)));
    ecLeftIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
    ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

    /* Trọng số thành viên tam giác / Triangular membership degree */
    eLeftTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
    eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
    ecLeftTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
    ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

    /* Giải mờ tính delta Kp, Ki, Kd theo trọng số / Weighted average defuzzification */
    fuzzyRule->KpFuzzy = (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] +
                          eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] +
                          eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] +
                          eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex]) * fuzzyRule->KpRatio;

    fuzzyRule->KiFuzzy = (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] +
                          eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] +
                          eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] +
                          eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex]) * fuzzyRule->KiRatio;

    fuzzyRule->KdFuzzy = (eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] +
                          eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] +
                          eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] +
                          eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex]) * fuzzyRule->KdRatio;
}

/* ==============================================================================
 * 2. PID CONTROL IMPLEMENTATION / HIỆN THỰC BỘ ĐIỀU KHIỂN PID
 * ============================================================================== */
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief  Khởi tạo bộ PID mở rộng / Initialize extended PID controller
 */
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
    uint8_t improve)
{
    pid->MaxOut = max_out;
    pid->IntegralLimit = intergral_limit;
    pid->DeadBand = deadband;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;
    pid->Derivative_LPF_RC = derivative_lpf_rc;

    pid->OLS_Order = ols_order;
    if (ols_order > 2)
        OLS_Init(&pid->OLS, ols_order);

    pid->Improve = improve;

    pid->DWT_CNT = 0;

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
}

/**
 * @brief  Tính toán ngõ ra PID theo chu kỳ thời gian thực / Compute PID output in real-time
 * @param  pid:     Con trỏ cấu trúc PID / Pointer to PID controller
 * @param  measure: Giá trị đo thực tế / Current measurement
 * @param  ref:     Giá trị đặt mục tiêu / Target reference setpoint
 * @retval Tín hiệu điều khiển đầu ra / Calculated control output
 */
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
    {
        f_PID_ErrorHandle(pid);
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0;
        }
    }

    pid->dt = DWT_GetDeltaT((void *)&pid->DWT_CNT);

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (abs(pid->Err) > pid->DeadBand)
    {
        if (pid->FuzzyRule != NULL)
        {
            Fuzzy_Rule_Implementation(pid->FuzzyRule, measure, ref);
        }

        if (pid->User_Func1_f != NULL)
            pid->User_Func1_f(pid);

        if (pid->FuzzyRule == NULL)
        {
            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }
        else
        {
            pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
            pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
        }

        if (pid->User_Func2_f != NULL)
            pid->User_Func2_f(pid);

        /* 1. Tích phân hình thang / Trapezoid Integral */
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);

        /* 2. Tích phân biến thiên theo sai số / Variable integration rate */
        if (pid->Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);

        /* 3. Vi phân theo biến đo / Derivative on measurement */
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);

        /* 4. Lọc thông thấp thành phần vi phân / Derivative LPF */
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);

        /* 5. Giới hạn tích phân chống bão hòa / Anti-windup limit */
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        /* 6. Lọc thông thấp tín hiệu đầu ra / Output LPF */
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        /* 7. Giới hạn độ lớn đầu ra / Output saturation */
        f_Output_Limit(pid);

        /* 8. Giới hạn thành phần tỉ lệ P / Proportion limit */
        f_Proportion_Limit(pid);
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

static void f_Trapezoid_Intergral(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        /* Cùng chiều: giảm hệ số tích phân khi sai số lớn / Same direction: reduce Ki */
        if (abs(pid->Err) <= pid->CoefB)
            return;
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
    {
        if (pid->OLS_Order > 2)
            pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        if (pid->OLS_Order > 2)
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

static void f_PID_ErrorHandle(PID_t *pid)
{
    if (pid->Output >= pid->MaxOut)
    {
        if (abs(pid->Ref - pid->Measure) >= pid->CoefA)
        {
            pid->ERRORHandler.ERRORCount++;
        }
        else
        {
            pid->ERRORHandler.ERRORCount = 0;
        }

        if (pid->ERRORHandler.ERRORCount > 1000)
        {
            pid->ERRORHandler.ERRORType = Motor_Blocked;
        }
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }
}

/* ==============================================================================
 * 3. FEEDFORWARD CONTROL IMPLEMENTATION / HIỆN THỰC ĐIỀU KHIỂN BÙ TIẾN
 * ============================================================================== */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    ffc->LPF_RC = lpf_rc;

    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = 0;
    ffc->Output = 0;
}

float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
    ffc->dt = DWT_GetDeltaT((void *)&ffc->DWT_CNT);

    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);

    /* Đạo hàm bậc 1 / First derivative */
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

    /* Đạo hàm bậc 2 / Second derivative */
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

    /* Tính đầu ra bù tiến / Feedforward control output */
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;
    ffc->Output = float_constrain(ffc->Output, -ffc->MaxOut, ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;
    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}

/* ==============================================================================
 * 4. LINEAR DISTURBANCE OBSERVER (LDOB) IMPLEMENTATION / BỘ QUAN SÁT NHIỄU
 * ============================================================================== */
void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order)
{
    ldob->Max_Disturbance = max_d;
    ldob->DeadBand = deadband;

    if (c != NULL && ldob != NULL)
    {
        ldob->c[0] = c[0];
        ldob->c[1] = c[1];
        ldob->c[2] = c[2];
    }
    else
    {
        ldob->c[0] = 0;
        ldob->c[1] = 0;
        ldob->c[2] = 0;
        ldob->Max_Disturbance = 0;
    }

    ldob->LPF_RC = lpf_rc;

    ldob->Measure_dot_OLS_Order = measure_dot_ols_order;
    ldob->Measure_ddot_OLS_Order = measure_ddot_ols_order;
    if (measure_dot_ols_order > 2)
        OLS_Init(&ldob->Measure_dot_OLS, measure_dot_ols_order);
    if (measure_ddot_ols_order > 2)
        OLS_Init(&ldob->Measure_ddot_OLS, measure_ddot_ols_order);

    ldob->DWT_CNT = 0;
    ldob->Disturbance = 0;
}

float LDOB_Calculate(LDOB_t *ldob, float measure, float u)
{
    ldob->dt = DWT_GetDeltaT((void *)&ldob->DWT_CNT);

    ldob->Measure = measure;
    ldob->u = u;

    /* Đạo hàm bậc 1 biến đo / First derivative of measurement */
    if (ldob->Measure_dot_OLS_Order > 2)
        ldob->Measure_dot = OLS_Derivative(&ldob->Measure_dot_OLS, ldob->dt, ldob->Measure);
    else
        ldob->Measure_dot = (ldob->Measure - ldob->Last_Measure) / ldob->dt;

    /* Đạo hàm bậc 2 biến đo / Second derivative of measurement */
    if (ldob->Measure_ddot_OLS_Order > 2)
        ldob->Measure_ddot = OLS_Derivative(&ldob->Measure_ddot_OLS, ldob->dt, ldob->Measure_dot);
    else
        ldob->Measure_ddot = (ldob->Measure_dot - ldob->Last_Measure_dot) / ldob->dt;

    /* Ước lượng nhiễu ngoại lực / Disturbance estimation: d = G^-1 * y - u */
    ldob->Disturbance = ldob->c[0] * ldob->Measure + ldob->c[1] * ldob->Measure_dot + ldob->c[2] * ldob->Measure_ddot - ldob->u;
    ldob->Disturbance = ldob->Disturbance * ldob->dt / (ldob->LPF_RC + ldob->dt) +
                        ldob->Last_Disturbance * ldob->LPF_RC / (ldob->LPF_RC + ldob->dt);

    ldob->Disturbance = float_constrain(ldob->Disturbance, -ldob->Max_Disturbance, ldob->Max_Disturbance);

    /* Vùng chết ngõ ra / Deadband handling */
    if (abs(ldob->Disturbance) > ldob->DeadBand * ldob->Max_Disturbance)
        ldob->Output = ldob->Disturbance;
    else
        ldob->Output = 0;

    ldob->Last_Measure = ldob->Measure;
    ldob->Last_Measure_dot = ldob->Measure_dot;
    ldob->Last_Disturbance = ldob->Disturbance;

    return ldob->Output;
}

/* ==============================================================================
 * 5. TRACKING DIFFERENTIATOR (TD) IMPLEMENTATION / BỘ ĐẠO HÀM BÁM MỀM
 * ============================================================================== */
void TD_Init(TD_t *td, float r, float h0)
{
    td->r = r;
    td->h0 = h0;

    td->x = 0;
    td->dx = 0;
    td->ddx = 0;
    td->last_dx = 0;
    td->last_ddx = 0;
}

float TD_Calculate(TD_t *td, float input)
{
    static float d, a0, y, a1, a2, a, fhan;

    td->dt = DWT_GetDeltaT((void *)&td->DWT_CNT);

    if (td->dt > 0.5f)
        return 0;

    td->Input = input;

    /* Giải thuật fhan của GS. Hàn Kinh Thanh / Han's fhan non-linear discrete tracking function */
    d = td->r * td->h0 * td->h0;
    a0 = td->dx * td->h0;
    y = td->x - td->Input + a0;
    a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
    a2 = a0 + sign(y) * (a1 - d) / 2.0f;
    a = (a0 + y) * (sign(y + d) - sign(y - d)) / 2.0f + a2 * (1.0f - (sign(y + d) - sign(y - d)) / 2.0f);
    fhan = -td->r * a / d * (sign(a + d) - sign(a - d)) / 2.0f -
           td->r * sign(a) * (1.0f - (sign(a + d) - sign(a - d)) / 2.0f);

    td->ddx = fhan;
    td->dx += (td->ddx + td->last_ddx) * td->dt / 2.0f;
    td->x += (td->dx + td->last_dx) * td->dt / 2.0f;

    td->last_ddx = td->ddx;
    td->last_dx = td->dx;

    return td->x;
}
