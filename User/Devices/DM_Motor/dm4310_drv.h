/**
 ******************************************************************************
 * @file    dm4310_drv.h
 * @brief   Driver giao tiếp và điều khiển động cơ Damiao DM4310 & DM6215 qua CAN bus
 * @author  Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
 * @date    2024 - 2026
 * @note    Wheeled-Bipedal Jumping Robot (DM-jump) Firmware
 *          Target MCU: STM32H723VGT6 | FreeRTOS | Keil MDK-ARM
 * @link    https://github.com/nguyenbinh-shark/DM-jump
 * @website https://nguyenbinh-shark.github.io/
 *
 * Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * Distributed under the MIT License.
 ******************************************************************************
 */

#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

/* --- Các chế độ điều khiển động cơ Damiao (Control Modes) --- */
#define MIT_MODE        0x000   /*!< Điều khiển hỗn hợp lực/vị trí/vận tốc (MIT Mode) */
#define POS_MODE        0x100   /*!< Điều khiển vị trí (Position Mode) */
#define SPEED_MODE      0x200   /*!< Điều khiển vận tốc (Velocity Mode) */

/* --- Giới hạn thông số cho động cơ khớp DM4310 (Joint Motors) --- */
#define P_MIN           -12.5f  /*!< Vị trí tối thiểu (rad) */
#define P_MAX           12.5f   /*!< Vị trí tối đa (rad) */
#define V_MIN           -30.0f  /*!< Vận tốc tối thiểu (rad/s) */
#define V_MAX           30.0f   /*!< Vận tốc tối đa (rad/s) */
#define KP_MIN          0.0f    /*!< Hệ số độ cứng Kp tối thiểu */
#define KP_MAX          500.0f  /*!< Hệ số độ cứng Kp tối đa */
#define KD_MIN          0.0f    /*!< Hệ số giảm chấn Kd tối thiểu */
#define KD_MAX          5.0f    /*!< Hệ số giảm chấn Kd tối đa */
#define T_MIN           -10.0f  /*!< Mô-men xoắn tối thiểu (N.m) */
#define T_MAX           10.0f   /*!< Mô-men xoắn tối đa (N.m) */

/* --- Giới hạn thông số cho động cơ bánh xe DM6215 (Wheel Motors) --- */
#define P_MIN2          -12.0f  /*!< Vị trí tối thiểu DM6215 (rad) */
#define P_MAX2          12.0f   /*!< Vị trí tối đa DM6215 (rad) */
#define V_MIN2          -45.0f  /*!< Vận tốc tối thiểu DM6215 (rad/s) */
#define V_MAX2          45.0f   /*!< Vận tốc tối đa DM6215 (rad/s) */
#define KP_MIN2         0.0f    /*!< Hệ số Kp tối thiểu DM6215 */
#define KP_MAX2         500.0f  /*!< Hệ số Kp tối đa DM6215 */
#define KD_MIN2         0.0f    /*!< Hệ số Kd tối thiểu DM6215 */
#define KD_MAX2         5.0f    /*!< Hệ số Kd tối đa DM6215 */
#define T_MIN2          -18.0f  /*!< Mô-men xoắn tối thiểu DM6215 (N.m) */
#define T_MAX2          18.0f   /*!< Mô-men xoắn tối đa DM6215 (N.m) */

/**
 * @brief Cấu trúc dữ liệu phản hồi trạng thái động cơ từ bus CAN
 */
typedef struct 
{
    uint16_t id;        /*!< ID của động cơ phản hồi */
    uint16_t state;     /*!< Mã trạng thái lỗi hoặc hoạt động */
    int p_int;          /*!< Dữ liệu vị trí thô dạng số nguyên */
    int v_int;          /*!< Dữ liệu vận tốc thô dạng số nguyên */
    int t_int;          /*!< Dữ liệu mô-men thô dạng số nguyên */
    int kp_int;         /*!< Kp thô */
    int kd_int;         /*!< Kd thô */
    float pos;          /*!< Góc vị trí quy đổi (rad) */
    float vel;          /*!< Vận tốc góc quy đổi (rad/s) */
    float tor;          /*!< Mô-men xoắn thực tế (N.m) */
    float Kp;           /*!< Hệ số Kp */
    float Kd;           /*!< Hệ số Kd */
    float Tmos;         /*!< Nhiệt độ mạch công suất MOSFET (°C) */
    float Tcoil;        /*!< Nhiệt độ cuộn dây động cơ (°C) */
} motor_fbpara_t;

/**
 * @brief Cấu trúc quản lý đối tượng động cơ khớp DM4310
 */
typedef struct
{
    uint16_t mode;          /*!< Chế độ hoạt động hiện tại */
    motor_fbpara_t para;    /*!< Dữ liệu phản hồi trạng thái */
} Joint_Motor_t;

/**
 * @brief Cấu trúc quản lý đối tượng động cơ bánh lăn DM6215
 */
typedef struct
{
    uint16_t mode;          /*!< Chế độ hoạt động hiện tại */
    float wheel_T;          /*!< Mô-men điều khiển bánh xe */
    motor_fbpara_t para;    /*!< Dữ liệu phản hồi trạng thái */
} Wheel_Motor_t;

/* --- Hàm khởi tạo đối tượng động cơ --- */
void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode);
void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode);

/* --- Hàm giải mã dữ liệu phản hồi từ bus CAN --- */
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);

/* --- Hàm bật / tắt chế độ hoạt động động cơ qua CAN --- */
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

/* --- Các hàm gửi lệnh điều khiển qua CAN --- */
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void pos_speed_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void speed_ctrl(hcan_t* hcan, uint16_t motor_id, float vel);

/* --- Các hàm chuyển đổi định dạng dữ liệu (Data compression) --- */
float Hex_To_Float(uint32_t *Byte, int num);
uint32_t FloatTohex(float HEX);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

#ifdef __cplusplus
}
#endif

#endif /* __DM4310_DRV_H__ */
