/**
 ******************************************************************************
 * @file    ps2_task.c
 * @brief   Hiện thực tác vụ đọc tay cầm PS2 và ánh xạ lệnh điều khiển khung gầm
 *          PS2 Controller Task: Stick/Button Decoding & Chassis Command Mapping (Bilingual EN/VI)
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

#include "ps2_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "tim.h"
#include "bsp_PWM.h"

ps2data_t ps2data;

uint16_t Handkey;
uint8_t Comd[2] = {0x01, 0x42};
uint8_t Data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t MASK[] = {
    PSB_SELECT,
    PSB_L3,
    PSB_R3,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};

extern chassis_t chassis_move;
extern INS_t INS;
uint32_t PS2_TIME = 10; /* Chu kỳ đọc tay cầm 10ms (100Hz) / 100Hz polling rate */

/**
 * @brief  Vòng lặp tác vụ đọc tay cầm PS2 / PS2 Controller Task Loop
 */
void pstwo_task(void)
{		
    PS2_SetInit();
    while (1)
    {
        if (Data[1] != 0x73)
        {
            /* Nếu mất kết nối Analog mode, khởi tạo lại / Re-init if not in analog red mode */
            PS2_SetInit();
        }

        PS2_data_read(&ps2data);
        PS2_data_process(&ps2data, &chassis_move, (float)PS2_TIME / 1000.0f);
        osDelay(PS2_TIME);
    }
}

/**
 * @brief  Gửi 1 byte lệnh tới tay cầm và đọc phản hồi qua chân GPIO (Bit-banging)
 *         Transmit one command byte to PS2 controller and receive response byte
 */
void PS2_Cmd(uint8_t CMD)
{
    volatile uint16_t ref = 0x01;
    Data[1] = 0;
    for (ref = 0x01; ref < 0x0100; ref <<= 1)
    {
        if (ref & CMD)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   /* DO_H */
        }
        else 
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); /* DO_L */
        }

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);      /* CLK_H */
        DWT_Delay(0.000005f);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);    /* CLK_L */
        DWT_Delay(0.000005f);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);      /* CLK_H */
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
            Data[1] = ref | Data[1];
    }
    DWT_Delay(0.000016f); 
}

uint8_t reve_flag = 0;

/**
 * @brief  Đọc trạng thái các nút bấm và 2 cần gạt Joystick / Read buttons & analog sticks
 */
void PS2_data_read(ps2data_t *data)
{
    data->key = PS2_DataKey();

    data->lx = PS2_AnologData(PSS_LX);
    data->ly = PS2_AnologData(PSS_LY);
    data->rx = PS2_AnologData(PSS_RX);
    data->ry = PS2_AnologData(PSS_RY);

    /* Vùng chết phần cứng cần Joystick / Analog stick center deadbands */
    if ((data->ry <= 255 && data->ry > 192) || (data->ry < 64 && data->ry >= 0))
    {
        data->rx = 127;
    }
    if ((data->rx <= 255 && data->rx > 192) || (data->rx < 64 && data->rx >= 0))
    {
        data->ry = 128;
    }
}

extern vmc_leg_t right;			
extern vmc_leg_t left;	
float acc_test = 0.005f;

/**
 * @brief  Ánh xạ dữ liệu điều khiển từ tay cầm PS2 sang các mục tiêu vận tốc, góc và độ cao chân
 *         Map PS2 inputs to chassis velocity, turning rate, roll, and leg height targets
 */
void PS2_data_process(ps2data_t *data, chassis_t *chassis, float dt)
{
    /* Nút Tam Giác chuyển đổi chế độ PS2 / UART / Toggle control mode (PS2 or UART) */
    PS2_mode_switch(data, chassis);
    chassis->start_flag = 1;

    if (chassis->control_mode != 0)
    {
        /* Khi ở chế độ UART, bỏ qua cần gạt PS2 để tránh xung đột / Hold pose when in UART mode */
        chassis->target_v = 0.0f;
        chassis->v_set = 0.0f;
        chassis->x_set = chassis->x_filter;
        chassis->turn_set = chassis->total_yaw;
        chassis->roll_target = 0.0f;
        slope_following(&chassis->roll_target, &chassis->roll_set, 0.0075f);

        data->last_key = data->key;
        return;
    }

    /* Bật/tắt robot bằng nút START / Toggle robot arming with START button */
    if (data->last_key != 4 && data->key == 4 && chassis->start_flag == 0) 
    {
        chassis->start_flag = 1;
        if (chassis->recover_flag == 0
            && ((chassis->myPithR < ((-3.1415926f) / 4.0f) && chassis->myPithR > ((-3.1415926f) / 2.0f))
            || (chassis->myPithR > (3.1415926f / 4.0f) && chassis->myPithR < (3.1415926f / 2.0f))))
        {
            chassis->recover_flag = 1; /* Tự đứng dậy sau ngã / Needs recovery */
        }
    }
    else if (data->last_key != 4 && data->key == 4 && chassis->start_flag == 1) 
    {
        chassis->start_flag = 0;
        chassis->recover_flag = 0;
    }

    /* Nút L2 thay đổi cấp tốc độ (0.5x, 1.0x, 1.5x) / Slew speed level toggle */
    static float speed_mult = 1.0f;
    static uint8_t speed_level = 1;
    static uint8_t beep_count = 0;
    static uint8_t beep_state = 0;
    static uint32_t level_beep_timer = 0;
    
    if (data->last_key != 9 && data->key == 9)
    {
        speed_level++;
        if (speed_level > 2) speed_level = 0;
        
        if (speed_level == 0) speed_mult = 0.5f;
        else if (speed_level == 1) speed_mult = 1.0f;
        else if (speed_level == 2) speed_mult = 1.5f;
        
        /* Còi báo: 1 tiếng cho 0.5x, 2 tiếng cho 1.0x, 3 tiếng cho 1.5x / Beep feedback */
        beep_count = speed_level + 1; 
        beep_state = 1;
        level_beep_timer = HAL_GetTick();
        TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    }

    /* Máy trạng thái còi báo không chặn / Non-blocking buzzer state machine */
    if (beep_state > 0)
    {
        uint32_t now = HAL_GetTick();
        if (beep_state == 1 && (now - level_beep_timer) >= 100)
        {
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
            level_beep_timer = now;
            beep_count--;
            
            if (beep_count > 0) beep_state = 2;
            else beep_state = 0;
        }
        else if (beep_state == 2 && (now - level_beep_timer) >= 100)
        {
            TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            level_beep_timer = now;
            beep_state = 1;
        }
    }

    if (chassis->start_flag == 1)
    {
        /* 1. Điều khiển vận tốc tiến lùi bằng cần trái Y / Forward/Backward velocity */
        if (data->ly < 120)
        {
            chassis->target_v = ((float)(128 - data->ly) / 128.0f) * 1.5f * speed_mult;
        }
        else if (data->ly > 136)
        {
            chassis->target_v = ((float)(128 - data->ly) / 128.0f) * 1.5f * speed_mult;
        }
        else
        {
            chassis->target_v = 0.0f;
        }
        slope_following(&chassis->target_v, &chassis->v_set, acc_test);
        chassis->x_set = chassis->x_set + chassis->v_set * dt;

        /* 2. Điều khiển góc xoay hướng Yaw bằng cần phải X / Yaw steering rate */
        if (data->rx > 136)
        {
            chassis->target_turn = chassis->target_turn - ((float)(data->rx - 128) / 128.0f) * 1.2f * dt;
        }
        else if (data->rx < 120)
        {
            chassis->target_turn = chassis->target_turn - ((float)(data->rx - 128) / 128.0f) * 1.2f * dt;
        }
        slope_following(&chassis->target_turn, &chassis->turn_set, 0.05f);

        /* 3. Điều khiển nghiêng thân Roll / Roll tilt angle */
        if (data->lx < 120)
        {
            chassis->roll_target = -0.15f * ((float)(128 - data->lx) / 128.0f);
        }
        else if (data->lx > 136)
        {
            chassis->roll_target = 0.15f * ((float)(data->lx - 128) / 128.0f);
        }
        else
        {
            chassis->roll_target = 0.0f;
        }
        slope_following(&chassis->roll_target, &chassis->roll_set, 0.0075f);

        /* 4. Điều khiển chiều cao chân bằng cần phải Y / Leg height */
        if (data->ry < 120)
        {
            chassis->target_leg_lx_set = chassis->target_leg_lx_set + ((float)(128 - data->ry) / 128.0f) * 0.001f;
        }
        else if (data->ry > 136)
        {
            chassis->target_leg_lx_set = chassis->target_leg_lx_set + ((float)(128 - data->ry) / 128.0f) * 0.001f;
        }
        mySaturate(&chassis->target_leg_lx_set, 0.08f, 0.21f);
        slope_following(&chassis->target_leg_lx_set, &chassis->leg_set, 0.001f);

        jump_key(chassis, data);

        chassis->leg_left_set = chassis->leg_set;
        chassis->leg_right_set = chassis->leg_set;

        mySaturate(&chassis->leg_left_set, 0.060f, 0.21f);
        mySaturate(&chassis->leg_right_set, 0.060f, 0.21f);

        if (fabsf(chassis->last_leg_left_set - chassis->leg_left_set) > 0.0001f || 
            fabsf(chassis->last_leg_right_set - chassis->leg_right_set) > 0.0001f)
        {
            right.leg_flag = 1;
            left.leg_flag = 1;                    
        }
        chassis->last_leg_set = chassis->leg_set;
        chassis->last_leg_left_set = chassis->leg_left_set;
        chassis->last_leg_right_set = chassis->leg_right_set;
    }
    else if (chassis->start_flag == 0)
    {
        chassis->v_set = 0.0f;
        chassis->x_set = chassis->x_filter;
        chassis->turn_set = chassis->total_yaw;
        chassis->leg_set = 0.08f;
    }

    data->last_key = data->key;
}

/**
 * @brief  Kiểm tra nút R1 (key 12) để kích hoạt nhảy / Jump trigger check
 */
void jump_key(chassis_t *chassis, ps2data_t *data)
{
    if (data->key == 12)
    {
        if (++chassis->count_key > 10)
        {
            if (chassis->jump_flag == 0)
            {
                chassis->jump_flag = 1;
                chassis->jump_leg = chassis->leg_set;
            }
        }
    }
    else
    {
        chassis->count_key = 0;
    }
}

static uint8_t last_mode_btn_state = 0;
static uint32_t last_mode_switch_time = 0;
static uint32_t beep_timer = 0;
static uint8_t beep_stage = 0;

#define PS2_TOGGLE_KEY PSB_GREEN

/**
 * @brief  Nút Tam Giác chuyển đổi chế độ điều khiển PS2 / UART kèm phản hồi âm thanh còi
 *         Toggle PS2 / UART control mode with buzzer feedback
 */
void PS2_mode_switch(ps2data_t *data, chassis_t *chassis)
{
    uint32_t now = HAL_GetTick();
    
    if (beep_stage > 0)
    {
        if (beep_stage == 1 && (now - beep_timer) >= 100)
        {
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
            beep_timer = now;
            
            if (chassis->control_mode == 0)
            {
                beep_stage = 0; /* 1 tiếng bíp cho PS2 mode / Single beep for PS2 */
            }
            else
            {
                beep_stage = 2; /* 2 tiếng bíp cho UART mode / Double beep for UART */
            }
        }
        else if (beep_stage == 2 && (now - beep_timer) >= 100)
        {
            TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            beep_timer = now;
            beep_stage = 3;
        }
        else if (beep_stage == 3 && (now - beep_timer) >= 100)
        {
            HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
            beep_stage = 0;
        }
    }
    
    if (last_mode_btn_state == 0 && data->key == PS2_TOGGLE_KEY && (now - last_mode_switch_time > 500))
    {
        last_mode_switch_time = now;
        chassis->control_mode = (chassis->control_mode == 0) ? 1 : 0;
        
        TIM_Set_PWM(&htim12, TIM_CHANNEL_2, 500);
        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
        beep_timer = HAL_GetTick();
        beep_stage = 1;
    }
    
    last_mode_btn_state = (data->key == PS2_TOGGLE_KEY) ? 1 : 0;
}

uint8_t PS2_RedLight(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    PS2_Cmd(Comd[0]);
    PS2_Cmd(Comd[1]);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    if (Data[1] == 0x73) return 0;
    else return 1;
}

void PS2_ReadData(void)
{
    volatile uint8_t byte = 0;
    volatile uint16_t ref = 0x01;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    PS2_Cmd(Comd[0]);
    PS2_Cmd(Comd[1]);
    for (byte = 2; byte < 9; byte++)
    {
        for (ref = 0x01; ref < 0x100; ref <<= 1)
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
            DWT_Delay(0.000005f);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
            DWT_Delay(0.000005f);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
                Data[byte] = ref | Data[byte];
        }
        DWT_Delay(0.000016f); 
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

uint8_t PS2_DataKey()
{
    uint8_t index;

    PS2_ClearData();
    PS2_ReadData();

    Handkey = (Data[4] << 8) | Data[3];
    for (index = 0; index < 16; index++)
    {
        if ((Handkey & (1 << (MASK[index] - 1))) == 0)
            return index + 1;
    }
    return 0;
}

uint8_t PS2_AnologData(uint8_t button)
{
    return Data[button];
}

void PS2_ClearData()
{
    uint8_t a;
    for (a = 0; a < 9; a++)
        Data[a] = 0x00;
}

void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);
    PS2_Cmd(0x42);
    PS2_Cmd(0x00);
    PS2_Cmd(motor1);
    PS2_Cmd(motor2);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f);  
}

void PS2_ShortPoll(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);  
    PS2_Cmd(0x42);  
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f); 	
}

void PS2_EnterConfing(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0x00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f); 
}

void PS2_TurnOnAnalogMode(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);  
    PS2_Cmd(0x44);  
    PS2_Cmd(0x00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x03);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f); 
}

void PS2_VibrationMode(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);  
    PS2_Cmd(0x4D);  
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x01);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f); 
}

void PS2_ExitConfing(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    DWT_Delay(0.000016f); 
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay(0.000016f); 
}

void PS2_SetInit(void)
{
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfing();
    PS2_TurnOnAnalogMode();
    PS2_ExitConfing();
}
