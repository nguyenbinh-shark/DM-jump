/**
 ******************************************************************************
 * @file    user_lib.h
 * @brief   Thư viện các hàm toán học, bộ lọc dốc ramp, OLS và hàm giới hạn (Bilingual EN/VI)
 *          General Math Utilities, Ramp Filter, OLS Regression, and Saturation Limits
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

#ifndef _USER_LIB_H
#define _USER_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Các hằng số toán học / Mathematical constants */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.29577951308232f
#endif

#ifndef PI
#define PI 3.141592653589793f
#endif

/* Macro giới hạn khoảng giá trị / Clamp macro */
#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief Cấu trúc tạo hàm dốc tăng tốc mượt / Ramp function generator structure
 */
typedef struct
{
    float input;        /*!< Giá trị mục tiêu đầu vào / Target input value */
    float out;          /*!< Giá trị dốc đầu ra tức thời / Ramp output value */
    float min_value;    /*!< Giá trị nhỏ nhất / Minimum allowed value */
    float max_value;    /*!< Giá trị lớn nhất / Maximum allowed value */
    float frame_period; /*!< Chu kỳ thực thi / Execution period */
} ramp_function_source_t;

/**
 * @brief Cấu trúc lọc bình phương bé nhất (OLS) / Ordinary Least Squares filter structure
 */
typedef __packed struct
{
    uint16_t Order;     /*!< Bậc của bộ lọc / Regression filter order */
    uint32_t Count;

    float *x;
    float *y;

    float k;            /*!< Hệ số góc (đạo hàm) / Slope (derivative) */
    float b;            /*!< Hệ số tự do (giá trị làm mượt) / Intercept (smoothed value) */

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

/* --- Các hàm tiện ích toán học / Math utility functions --- */
float Sqrt(float x);

/* Hàm dốc / Ramp function */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

/* Hàm bão hòa và dấu / Saturation and sign functions */
float abs_limit(float num, float Limit);
float sign(float value);

/* Vùng chết / Deadband */
float float_deadband(float Value, float minValue, float maxValue);
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

/* Khống chế giá trị trong khoảng / Constrain functions */
float float_constrain(float Value, float minValue, float maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
float loop_float_constrain(float Input, float minValue, float maxValue);

/* Chuẩn hóa góc / Angle formatting */
float theta_format(float Ang);
int float_rounding(float raw);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

/* Bộ lọc bình phương tối thiểu OLS / Ordinary Least Squares functions */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

/* Hàm bám dốc giới hạn gia tốc / Slew-rate slope following */
void slope_following(float *target, float *set, float acc);

#ifdef __cplusplus
}
#endif

#endif /* _USER_LIB_H */
