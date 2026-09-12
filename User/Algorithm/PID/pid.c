/**
 ******************************************************************************
 * @file    pid.c
 * @brief   Hiện thực thuật toán PID vị trí và PID số gia (Bilingual EN/VI)
 *          Position and Incremental PID Controller Implementation
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

#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
 * @brief  Khởi tạo bộ điều khiển PID / Initialize PID controller instance
 * @param  pid:      Con trỏ đối tượng PID / Pointer to PID instance
 * @param  mode:     Chế độ (PID_POSITION hoặc PID_DELTA) / Control mode
 * @param  PID:      Mảng hệ số [Kp, Ki, Kd] / Array containing Kp, Ki, Kd
 * @param  max_out:  Giới hạn bão hòa đầu ra / Maximum output saturation limit
 * @param  max_iout: Giới hạn chống bão hòa tích phân / Anti-windup integral limit
 */
void PID_init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief  Tính toán ngõ ra PID theo chu kỳ / Compute PID control output per cycle
 * @param  pid: Con trỏ đối tượng PID / Pointer to PID instance
 * @param  ref: Giá trị đo phản hồi thực tế / Actual feedback measurement
 * @param  set: Giá trị đặt mục tiêu / Target setpoint reference
 * @retval Tín hiệu điều khiển đầu ra đã bão hòa / Saturated control output
 */
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        /* --- Chế độ PID Vị trí / Position PID Mode --- */
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        /* Khống chế tích phân chống bão hòa / Anti-windup clamp */
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        /* --- Chế độ PID Số gia / Incremental Velocity PID Mode --- */
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
 * @brief  Xóa bộ nhớ đệm và reset trạng thái PID / Clear buffers and reset PID state
 * @param  pid: Con trỏ đối tượng PID / Pointer to PID instance
 */
void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
