/**
 ******************************************************************************
 * @file    ps2_task.h
 * @brief   Tác vụ giao tiếp tay cầm không dây PS2 và ánh xạ điều khiển robot (Bilingual EN/VI)
 *          PS2 Wireless Controller Interface & Teleoperation Mapping Task Header
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

#ifndef __PSTWO_TASK_H
#define __PSTWO_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "chassisR_task.h"
#include "INS_task.h"

/* --- Mã định danh các nút bấm số trên tay cầm PS2 / PS2 Digital Buttons --- */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13  /*!< Nút Tam Giác (Triangle) */
#define PSB_RED         14  /*!< Nút Tròn (Circle) */
#define PSB_BLUE        15  /*!< Nút X (Cross) */
#define PSB_PINK        16  /*!< Nút Vuông (Square) */

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

/* --- Chỉ số các trục Joystick tương tự / PS2 Analog Stick Axes --- */
#define PSS_RX          5   /*!< Trục X cần gạt phải / Right stick X */
#define PSS_RY          6   /*!< Trục Y cần gạt phải / Right stick Y */
#define PSS_LX          7   /*!< Trục X cần gạt trái / Left stick X */
#define PSS_LY          8   /*!< Trục Y cần gạt trái / Left stick Y */

/**
 * @brief Cấu trúc dữ liệu trạng thái tay cầm PS2 / PS2 controller state data
 */
typedef struct
{
    int16_t key;            /*!< Trạng thái nút bấm hiện tại / Current digital button mask */
    int16_t last_key;       /*!< Trạng thái nút bấm chu kỳ trước / Previous digital button mask */
    
    int16_t lx;             /*!< Vị trí trục X cần trái [0..255] (128 là giữa) / Left stick X */
    int16_t ly;             /*!< Vị trí trục Y cần trái [0..255] / Left stick Y */
    int16_t rx;             /*!< Vị trí trục X cần phải [0..255] / Right stick X */
    int16_t ry;             /*!< Vị trí trục Y cần phải [0..255] / Right stick Y */
} ps2data_t;

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

/* --- Các hàm xử lý dữ liệu tay cầm / PS2 Data processing APIs --- */
void PS2_data_read(ps2data_t *data);
void PS2_data_process(ps2data_t *data, chassis_t *chassis, float dt);
void jump_key(chassis_t *chassis, ps2data_t *data);
void PS2_mode_switch(ps2data_t *data, chassis_t *chassis);

/* --- Các hàm giao tiếp tầng vật lý PS2 (Bit-banging) / Low-level PS2 helpers --- */
uint8_t PS2_RedLight(void);
void PS2_ReadData(void);
void PS2_Cmd(uint8_t CMD);
uint8_t PS2_DataKey(void);
uint8_t PS2_AnologData(uint8_t button);
void PS2_ClearData(void);
void PS2_Vibration(uint8_t motor1, uint8_t motor2);

/* --- Cấu hình chế độ tay cầm / Controller configuration helpers --- */
void PS2_EnterConfing(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
void PS2_ExitConfing(void);
void PS2_SetInit(void);

void pstwo_task(void);

#ifdef __cplusplus
}
#endif

#endif /* __PSTWO_TASK_H */
