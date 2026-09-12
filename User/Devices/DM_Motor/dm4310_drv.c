/**
 ******************************************************************************
 * @file    dm4310_drv.c
 * @brief   Driver hiện thực giao tiếp và điều khiển động cơ Damiao DM4310 & DM6215
 * @author  Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
 * @date    2024 - 2026
 * @note    Wheeled-Bipedal Jumping Robot (DM-jump) Firmware
 *          Target MCU: STM32H723VGT6 | FreeRTOS | Keil MDK-ARM
 * @link    https://github.com/nguyenbinh-shark/DM-jump
 *
 * Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
 * Distributed under the MIT License.
 ******************************************************************************
 */

#include "dm4310_drv.h"
#include "fdcan.h"
#include "arm_math.h"

/**
 * @brief  Chuyển đổi mảng 4-byte dạng Hex sang giá trị float 32-bit
 * @param  Byte: Con trỏ tới vùng đệm dữ liệu 4 byte
 * @param  num:  Số lượng phần tử
 * @retval Giá trị số thực float
 */
float Hex_To_Float(uint32_t *Byte, int num)
{
    return *((float*)Byte);
}

/**
 * @brief  Chuyển đổi giá trị float 32-bit sang dạng nguyên 32-bit Hex
 * @param  HEX: Giá trị float đầu vào
 * @retval Giá trị uint32_t tương đương
 */
uint32_t FloatTohex(float HEX)
{
    return *(uint32_t *)&HEX;
}

/**
 * @brief  Nén số thực float thành số nguyên unsigned int theo dải giá trị và số bit
 * @param  x_float: Giá trị float đầu vào
 * @param  x_min:   Giá trị nhỏ nhất của dải
 * @param  x_max:   Giá trị lớn nhất của dải
 * @param  bits:    Số bit phân giải (e.g. 12, 16 bit)
 * @retval Số nguyên sau khi nén
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  Giải nén số nguyên unsigned int thành số thực float theo dải giá trị và số bit
 * @param  x_int: Giá trị nguyên cần giải nén
 * @param  x_min: Giá trị nhỏ nhất của dải
 * @param  x_max: Giá trị lớn nhất của dải
 * @param  bits:  Số bit phân giải (e.g. 12, 16 bit)
 * @retval Giá trị float thu được
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  Khởi tạo thông tin cho động cơ khớp DM4310
 * @param  motor: Con trỏ đối tượng động cơ khớp
 * @param  id:    ID của động cơ trên bus CAN
 * @param  mode:  Chế độ điều khiển khởi tạo
 */
void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode)
{
    motor->mode = mode;
    motor->para.id = id;
}

/**
 * @brief  Khởi tạo thông tin cho động cơ bánh lăn DM6215
 * @param  motor: Con trỏ đối tượng động cơ bánh lăn
 * @param  id:    ID của động cơ trên bus CAN
 * @param  mode:  Chế độ điều khiển khởi tạo
 */
void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode)
{
    motor->mode = mode;
    motor->para.id = id;
}

/**
 * @brief  Giải mã khung tin 8-byte phản hồi từ động cơ khớp DM4310
 * @param  motor:    Con trỏ đối tượng động cơ khớp cần lưu dữ liệu
 * @param  rx_data:  Bộ đệm nhận 8 byte từ CAN
 * @param  data_len: Độ dài gói tin (chuẩn 8 byte)
 */
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{ 
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
        motor->para.pos   = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);  /* Dải vị trí [-12.5, 12.5] rad */
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);  /* Dải vận tốc [-30.0, 30.0] rad/s */
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  /* Dải mô-men [-10.0, 10.0] N.m */
        motor->para.Tmos  = (float)(rx_data[6]);                                  /* Nhiệt độ MOSFET (°C) */
        motor->para.Tcoil = (float)(rx_data[7]);                                  /* Nhiệt độ cuộn cảm (°C) */
    }
}

/**
 * @brief  Giải mã khung tin 8-byte phản hồi từ động cơ bánh lăn DM6215
 * @param  motor:    Con trỏ đối tượng động cơ bánh lăn
 * @param  rx_data:  Bộ đệm nhận 8 byte từ CAN
 * @param  data_len: Độ dài gói tin (chuẩn 8 byte)
 */
void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{ 
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
        motor->para.pos   = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16); /* Dải vị trí [-12.0, 12.0] rad */
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12); /* Dải vận tốc [-45.0, 45.0] rad/s */
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12); /* Dải mô-men [-18.0, 18.0] N.m */
        motor->para.Tmos  = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}

/**
 * @brief  Gửi lệnh bật (Enable) chế độ điều khiển cho động cơ qua CAN
 * @param  hcan:      Con trỏ đối tượng giao tiếp CAN (FDCAN1 hoặc FDCAN2)
 * @param  motor_id:  ID của động cơ cần kích hoạt
 * @param  mode_id:   Chế độ (MIT_MODE, POS_MODE, SPEED_MODE)
 */
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC; /* Lệnh bật động cơ Damiao */
    
    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Gửi lệnh ngắt (Disable) chế độ điều khiển cho động cơ qua CAN
 * @param  hcan:      Con trỏ đối tượng giao tiếp CAN
 * @param  motor_id:  ID của động cơ cần ngắt
 * @param  mode_id:   Chế độ đang chạy
 */
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD; /* Lệnh tắt động cơ Damiao */
    
    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Điều khiển động cơ khớp DM4310 ở chế độ hỗn hợp MIT Control Mode
 * @param  hcan:      Con trỏ cấu trúc CAN
 * @param  motor_id:  ID động cơ nhận lệnh
 * @param  pos:       Góc vị trí mục tiêu (rad)
 * @param  vel:       Vận tốc góc mục tiêu (rad/s)
 * @param  kp:        Hệ số độ cứng ảo Kp
 * @param  kd:        Hệ số giảm chấn ảo Kd
 * @param  torq:      Mô-men xoắn bù trước Feedforward Torque (N.m)
 */
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
    kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    
    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Điều khiển động cơ bánh lăn DM6215 ở chế độ hỗn hợp MIT Control Mode
 * @param  hcan:      Con trỏ cấu trúc CAN
 * @param  motor_id:  ID động cơ nhận lệnh
 * @param  pos:       Góc vị trí mục tiêu (rad)
 * @param  vel:       Vận tốc góc mục tiêu (rad/s)
 * @param  kp:        Hệ số độ cứng ảo Kp
 * @param  kd:        Hệ số giảm chấn ảo Kd
 * @param  torq:      Mô-men xoắn bù trước Feedforward Torque (N.m)
 */
void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos,  P_MIN2,  P_MAX2,  16);
    vel_tmp = float_to_uint(vel,  V_MIN2,  V_MAX2,  12);
    kp_tmp  = float_to_uint(kp,   KP_MIN2, KP_MAX2, 12);
    kd_tmp  = float_to_uint(kd,   KD_MIN2, KD_MAX2, 12);
    tor_tmp = float_to_uint(torq, T_MIN2,  T_MAX2,  12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    
    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Điều khiển động cơ ở chế độ Vị trí kèm giới hạn Vận tốc (Position-Speed Mode)
 * @param  hcan:      Con trỏ cấu trúc CAN
 * @param  motor_id:  ID động cơ
 * @param  pos:       Vị trí mục tiêu (rad)
 * @param  vel:       Giới hạn vận tốc cực đại (rad/s)
 */
void pos_speed_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];
    
    id = motor_id + POS_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;
    
    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);
    
    canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Điều khiển động cơ ở chế độ Vận tốc thuần túy (Speed Mode)
 * @param  hcan:      Con trỏ cấu trúc CAN
 * @param  motor_id:  ID động cơ
 * @param  vel:       Vận tốc góc mục tiêu (rad/s)
 */
void speed_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];
    
    id = motor_id + SPEED_MODE;
    vbuf = (uint8_t*)&vel;
    
    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);
    
    canx_send_data(hcan, id, data, 4);
}
