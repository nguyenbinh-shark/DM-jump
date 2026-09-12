/**
 ******************************************************************************
 * @file    chassisL_task.h
 * @brief   Tác vụ điều khiển động cơ và cân bằng động chân TRÁI (Bilingual EN/VI)
 *          Left Leg Actuation & Dynamic Balancing Task Header
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

#ifndef __CHASSISL_TASK_H
#define __CHASSISL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "dm4310_drv.h"
#include "chassisR_task.h"

/* --- Các hàm API điều khiển chân trái / Left Leg APIs --- */
void ChassisL_task(void);
void ChassisL_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legl);
void chassisL_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins);
void chassisL_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, INS_t *ins, float *LQR_K, PidTypeDef *leg);
void jump_loop_l(chassis_t *chassis, vmc_leg_t *vmcl, PidTypeDef *leg);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSISL_TASK_H */
