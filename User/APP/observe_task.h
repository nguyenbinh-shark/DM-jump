/**
 ******************************************************************************
 * @file    observe_task.h
 * @brief   Tác vụ quan sát và ước lượng trạng thái chuyển động của robot (Bilingual EN/VI)
 *          Robot State Observer & Kinematic Velocity Estimation Task Header
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

#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ins_task.h"
#include "chassisL_task.h"
#include "main.h"

/* --- Các hàm API công khai / Public API functions --- */
void Observe_task(void);
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel);

#ifdef __cplusplus
}
#endif

#endif /* __OBSERVE_TASK_H */
