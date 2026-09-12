/**
 ******************************************************************************
 * @file    user_lib.c
 * @brief   Hiện thực các thuật toán toán học phụ trợ, bộ lọc dốc và lọc OLS
 *          Math Utilities, Ramp Generator, and OLS Filter Implementation (Bilingual EN/VI)
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

#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

uint8_t GlobalDebugMode = 7;

/**
 * @brief  Thuật toán xấp xỉ khai căn bậc hai Newton-Raphson
 *         Fast square root approximation using Newton-Raphson iteration
 * @param  x: Số đầu vào / Input value
 * @retval Căn bậc hai xấp xỉ / Approximate square root
 */
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    y = x / 2.0f;
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2.0f * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

/**
 * @brief  Khởi tạo bộ tạo hàm dốc / Initialize ramp function generator
 * @param  ramp_source_type: Con trỏ cấu trúc ramp / Pointer to ramp structure
 * @param  frame_period:     Chu kỳ thời gian thực thi / Execution frame period
 * @param  max:              Giới hạn cực đại / Maximum limit
 * @param  min:              Giới hạn cực tiểu / Minimum limit
 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
 * @brief  Tính toán giá trị dốc tăng tốc dần theo chu kỳ / Compute ramp output per cycle
 * @param  ramp_source_type: Con trỏ cấu trúc ramp / Pointer to ramp structure
 * @param  input:            Giá trị gia tốc mong muốn / Acceleration input
 * @retval Giá trị đầu ra hàm dốc / Ramp output value
 */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    return ramp_source_type->out;
}

/**
 * @brief  Giới hạn biên độ tuyệt đối của số thực / Clamp absolute value within limit
 * @param  num:   Giá trị đầu vào / Input value
 * @param  Limit: Giới hạn tuyệt đối / Absolute threshold limit
 * @retval Giá trị sau giới hạn / Clamped output
 */
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

/**
 * @brief  Hàm lấy dấu của số thực / Signum function
 * @param  value: Giá trị đầu vào / Input value
 * @retval 1.0f nếu dương, -1.0f nếu âm / 1.0f for non-negative, -1.0f for negative
 */
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/**
 * @brief  Vùng chết số thực / Float deadband filter
 * @param  Value:    Giá trị kiểm tra / Input value
 * @param  minValue: Ngưỡng cận dưới / Lower threshold
 * @param  maxValue: Ngưỡng cận trên / Upper threshold
 * @retval 0 nếu nằm trong vùng chết, ngược lại giữ nguyên / 0 inside deadband, else original
 */
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

/**
 * @brief  Vùng chết số nguyên 16-bit / Integer 16-bit deadband
 */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

/**
 * @brief  Khống chế giá trị số thực trong dải [minValue, maxValue] / Clamp float within range
 */
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief  Khống chế giá trị số nguyên trong dải [minValue, maxValue] / Clamp int16 within range
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief  Giới hạn tuần hoàn số thực trong dải chu kỳ / Circular wrap-around constrain
 */
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/**
 * @brief  Chuẩn hóa góc trong dải [-180, 180] độ / Normalize angle in degrees to [-180, 180]
 */
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

/**
 * @brief  Làm tròn số thực thành số nguyên gần nhất / Round float to nearest integer
 */
int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

/**
 * @brief  Khởi tạo bộ lọc hồi quy tuyến tính bình phương bé nhất (OLS)
 *         Initialize Ordinary Least Squares (OLS) filter
 * @param  OLS:   Con trỏ đối tượng OLS / Pointer to OLS structure
 * @param  order: Số lượng mẫu hồi quy / Filter order / window size
 */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);
    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
 * @brief  Cập nhật mẫu dữ liệu mới vào bộ lọc OLS / Update OLS filter sample
 * @param  OLS:    Con trỏ cấu trúc OLS / Pointer to OLS instance
 * @param  deltax: Bước nhảy thời gian / Time step delta x
 * @param  y:      Giá trị đo / Measured value y
 */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }
    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;
}

/**
 * @brief  Tính toán đạo hàm lọc OLS / Calculate derivative via OLS linear regression
 */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k;
}

/**
 * @brief  Lấy giá trị đạo hàm đã tính của OLS / Get cached OLS derivative slope
 */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k;
}

/**
 * @brief  Tính giá trị làm mượt tín hiệu qua OLS / Calculate smoothed value via OLS
 */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    static float temp = 0;
    temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset((void *)OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

    OLS->StandardDeviation = 0;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
 * @brief  Lấy giá trị làm mượt OLS / Get cached OLS smoothed value
 */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
 * @brief  Hàm bám dốc giới hạn gia tốc chuyển động / Slew-rate slope tracking limiter
 * @param  target: Giá trị đích mong muốn / Target reference
 * @param  set:    Giá trị tức thời được điều chỉnh / Current setpoint to ramp
 * @param  acc:    Bước nhảy gia tốc tối đa mỗi chu kỳ / Maximum acceleration step
 */
void slope_following(float *target, float *set, float acc)
{
    if (*target > *set)
    {
        *set = *set + acc;
        if (*set >= *target)
            *set = *target;
    }
    else if (*target < *set)
    {
        *set = *set - acc;
        if (*set <= *target)
            *set = *target;
    }
}

/* End of file user_lib.c */
