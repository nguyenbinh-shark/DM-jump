/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @brief   Hiện thực bộ đếm chu kỳ xung nhịp Cortex-M DWT độ phân giải micro-giây
 *          High-Resolution Cortex-M DWT Cycle Counter Implementation (Bilingual EN/VI)
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

#include "bsp_dwt.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
uint64_t CYCCNT64;
static void DWT_CNT_Update(void);

/**
 * @brief  Khởi tạo khối DWT và kích hoạt thanh ghi đếm chu kỳ CYCCNT
 *         Initialize DWT cycle counter peripheral
 * @param  CPU_Freq_mHz: Tần số xung nhịp CPU theo MHz (ví dụ: 550 cho STM32H723)
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* Bật khối gỡ lỗi và theo dõi DWT / Enable DWT tracing unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Đặt lại thanh ghi đếm CYCCNT / Reset DWT CYCCNT counter register */
    DWT->CYCCNT = (uint32_t)0u;

    /* Kích hoạt bộ đếm CYCCNT / Enable Cortex-M DWT CYCCNT counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
}

/**
 * @brief  Tính toán khoảng thời gian trôi qua dt (float) tính bằng giây
 *         Get elapsed time delta dt (float) in seconds
 * @param  cnt_last: Con trỏ lưu giá trị tick trước đó / Pointer to last tick count
 * @retval Thời gian trôi qua delta t (giây) / Elapsed time in seconds
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief  Tính toán khoảng thời gian dt (double) độ chính xác 64-bit
 *         Get elapsed time delta dt (double) in seconds with 64-bit precision
 * @param  cnt_last: Con trỏ lưu giá trị tick trước đó / Pointer to last tick count
 * @retval Thời gian trôi qua delta t (giây) / Elapsed time in seconds
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 * @brief  Cập nhật cấu trúc thời gian hệ thống SysTime (giây, mili-giây, micro-giây)
 *         Update global high-resolution system time structure
 */
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/**
 * @brief  Lấy mốc thời gian hệ thống theo giây / Get total system timeline in seconds
 * @retval Thời gian hệ thống (giây) / Timeline in seconds
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
}

/**
 * @brief  Lấy mốc thời gian hệ thống theo mili-giây / Get total system timeline in milliseconds
 * @retval Thời gian hệ thống (mili-giây) / Timeline in milliseconds
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000.0f + SysTime.ms + SysTime.us * 0.001f;
}

/**
 * @brief  Lấy mốc thời gian hệ thống theo micro-giây / Get total system timeline in microseconds
 * @retval Thời gian hệ thống 64-bit (micro-giây) / Timeline in microseconds
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return (uint64_t)SysTime.s * 1000000 + (uint64_t)SysTime.ms * 1000 + SysTime.us;
}

/**
 * @brief  Cập nhật trạng thái tràn số của bộ đếm 32-bit CYCCNT
 *         Handle 32-bit cycle counter overflow
 */
static void DWT_CNT_Update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST)
    {
        CYCCNT_RountCount++; /* Xử lý tràn bộ đếm 32-bit / Handle 32-bit overflow */
    }

    CYCCNT_LAST = cnt_now;
}

/**
 * @brief  Hàm trì hoãn chính xác micro-giây sử dụng chu kỳ CPU (Blocking Delay)
 *         Blocking delay in seconds using cycle counter
 * @param  Delay: Thời gian trì hoãn tính bằng giây / Delay duration in seconds
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}
