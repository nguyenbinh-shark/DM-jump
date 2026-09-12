/**
 ******************************************************************************
 * @file    can_bsp.c
 * @brief   Hiện thực bộ lọc phần cứng FDCAN và ngắt nhận dữ liệu động cơ DM4310
 *          FDCAN Filter Configuration & Motor RX Callback Implementation (Bilingual EN/VI)
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

#include "can_bsp.h"
#include "fdcan.h"
#include "dm4310_drv.h"
#include "string.h"
#include "chassisR_task.h"

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

extern chassis_t chassis_move;

/**
 * @brief  Cấu hình bộ lọc phần cứng FDCAN1 (chân phải robot)
 *         Configure FDCAN1 hardware filter & activate notifications (Right leg)
 */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* Cấu hình bộ lọc FIFO0 / Configure Rx filter for FIFO0 */ 	
    sFilterConfig.IdType = FDCAN_STANDARD_ID;       /* Sử dụng Standard ID (11-bit) */
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
        
    /* Cấu hình lọc toàn cục: Từ chối gói tin không khớp / Configure global filter */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Kích hoạt ngắt khi có tin nhắn mới tại RX FIFO0 / Enable RX FIFO0 new message interrupt */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Khởi động ngoại vi FDCAN1 / Start FDCAN1 peripheral */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Cấu hình bộ lọc phần cứng FDCAN2 (chân trái robot)
 *         Configure FDCAN2 hardware filter & activate notifications (Left leg)
 */
void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* Cấu hình bộ lọc FIFO1 / Configure Rx filter for FIFO1 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Kích hoạt ngắt nhận tin nhắn tại RX FIFO1 / Enable RX FIFO1 interrupt */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Khởi động ngoại vi FDCAN2 / Start FDCAN2 peripheral */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Đóng gói và gửi khung tin qua bus CAN / Pack and transmit frame over CAN
 * @param  hcan: Con trỏ cấu trúc FDCAN / Pointer to FDCAN handle
 * @param  id:   CAN ID định danh / Frame identifier
 * @param  data: Con trỏ mảng byte / Payload buffer
 * @param  len:  Độ dài dữ liệu (byte) / Data length in bytes
 * @retval 0 nếu thành công / 0 on success
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;        
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
    if (len <= 8)    
    {
        TxHeader.DataLength = len << 16;
    }
    else if (len == 12)    
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    }
    else if (len == 16)    
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    }
    else if (len == 20)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    }        
    else if (len == 24)    
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_24;    
    }
    else if (len == 48)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    }
    else if (len == 64)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }
                                            
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     /* Sử dụng Classic CAN 1Mbps / Classic CAN, no BRS */
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;      /* Khung tin Classic CAN / Classic CAN format */
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;  
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
    return 0;
}

/**
 * @brief  Callback ngắt nhận tin nhắn mới từ FDCAN1 (Chân phải robot)
 *         FDCAN1 RX FIFO0 interrupt callback (Right leg motors)
 * @param  hfdcan:     Con trỏ FDCAN handle / Pointer to FDCAN handle
 * @param  RxFifo0ITs: Cờ ngắt ngõ vào / Interrupt flags
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
            
            switch (RxHeader1.Identifier)
            {
                case 3: /* Động cơ khớp sau chân phải / Right back joint motor */
                    dm4310_fbdata(&chassis_move.joint_motor[0], g_Can1RxData, RxHeader1.DataLength);
                    break;
                case 4: /* Động cơ khớp trước chân phải / Right front joint motor */
                    dm4310_fbdata(&chassis_move.joint_motor[1], g_Can1RxData, RxHeader1.DataLength);
                    break;                 
                case 0: /* Động cơ bánh lăn chân phải / Right wheel motor */
                    dm6215_fbdata(&chassis_move.wheel_motor[0], g_Can1RxData, RxHeader1.DataLength);
                    break;
                default: 
                    break;
            }            
        }
    }
}

/**
 * @brief  Callback ngắt nhận tin nhắn mới từ FDCAN2 (Chân trái robot)
 *         FDCAN2 RX FIFO1 interrupt callback (Left leg motors)
 * @param  hfdcan:     Con trỏ FDCAN handle / Pointer to FDCAN handle
 * @param  RxFifo1ITs: Cờ ngắt ngõ vào / Interrupt flags
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN2)
        {
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
            switch (RxHeader2.Identifier)
            {
                case 3: /* Động cơ khớp sau chân trái / Left back joint motor */
                    dm4310_fbdata(&chassis_move.joint_motor[2], g_Can2RxData, RxHeader2.DataLength);
                    break;
                case 4: /* Động cơ khớp trước chân trái / Left front joint motor */
                    dm4310_fbdata(&chassis_move.joint_motor[3], g_Can2RxData, RxHeader2.DataLength);
                    break;                 
                case 0: /* Động cơ bánh lăn chân trái / Left wheel motor */
                    dm6215_fbdata(&chassis_move.wheel_motor[1], g_Can2RxData, RxHeader2.DataLength);
                    break;
                default: 
                    break;
            }    
        }
    }
}
