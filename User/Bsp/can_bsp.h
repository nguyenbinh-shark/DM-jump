/**
 ******************************************************************************
 * @file    can_bsp.h
 * @brief   Cấu hình bộ lọc phần cứng và giao tiếp FDCAN1 & FDCAN2 (Bilingual EN/VI)
 *          FDCAN1 & FDCAN2 Hardware Filter and Interface BSP Header
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

#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

/* --- Các hàm cấu hình phần cứng FDCAN / FDCAN Configuration APIs --- */
void FDCAN1_Config(void);
void FDCAN2_Config(void);

/**
 * @brief  Gửi khung tin qua giao thức CAN / Send frame over CAN bus
 * @param  hcan: Con trỏ cấu trúc FDCAN / Pointer to FDCAN_HandleTypeDef
 * @param  id:   CAN ID định danh gói tin / Frame identifier
 * @param  data: Con trỏ mảng byte dữ liệu cần gửi / Pointer to data payload
 * @param  len:  Độ dài dữ liệu tính bằng byte (1..64) / Length in bytes
 * @retval 0 nếu thành công / 0 on success
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* _CAN_BSP_H */
