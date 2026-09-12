/**
 ******************************************************************************
 * @file    BMI088driver.h
 * @brief   Driver điều khiển và cấu hình cảm biến IMU 6 trục BMI088 (SPI/I2C)
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

#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "main.h"

/* --- Hệ số chuyển đổi nhiệt độ BMI088 --- */
#define BMI088_TEMP_FACTOR              0.125f
#define BMI088_TEMP_OFFSET              23.0f

#define BMI088_WRITE_ACCEL_REG_NUM      6
#define BMI088_WRITE_GYRO_REG_NUM       6

#define BMI088_GYRO_DATA_READY_BIT      0
#define BMI088_ACCEL_DATA_READY_BIT     1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME          80
#define BMI088_COM_WAIT_SENSOR_TIME     150

#define BMI088_ACCEL_IIC_ADDRESSE       (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE        (0x68 << 1)

/* --- Hệ số độ nhạy cảm biến gia tốc kế (Accelerometer Sensitivity) --- */
#define BMI088_ACCEL_3G_SEN             0.0008974358974f    /*!< Dải ±3g: m/s^2 per LSB */
#define BMI088_ACCEL_6G_SEN             0.00179443359375f   /*!< Dải ±6g: m/s^2 per LSB */
#define BMI088_ACCEL_12G_SEN            0.0035888671875f    /*!< Dải ±12g: m/s^2 per LSB */
#define BMI088_ACCEL_24G_SEN            0.007177734375f     /*!< Dải ±24g: m/s^2 per LSB */

/* --- Hệ số độ nhạy con quay hồi chuyển (Gyroscope Sensitivity: rad/s per LSB) --- */
#define BMI088_GYRO_2000_SEN            0.00106526443603169529841533860381f      /*!< Dải ±2000 dps */
#define BMI088_GYRO_1000_SEN            0.00053263221801584764920766930190693f   /*!< Dải ±1000 dps */
#define BMI088_GYRO_500_SEN             0.00026631610900792382460383465095346f   /*!< Dải ±500 dps */
#define BMI088_GYRO_250_SEN             0.00013315805450396191230191732547673f   /*!< Dải ±250 dps */
#define BMI088_GYRO_125_SEN             0.000066579027251980956150958662738366f  /*!< Dải ±125 dps */

/* --- Giá trị sai lệch tĩnh (Zero-rate Bias Calibration Offsets) --- */
#define GxOFFSET    -0.000681414269f    /*!< Con quay Gyro X offset (rad/s) */
#define GyOFFSET    -0.00134240754f     /*!< Con quay Gyro Y offset (rad/s) */
#define GzOFFSET    -0.00143384014f     /*!< Con quay Gyro Z offset (rad/s) */
#define AxOFFSET    0.299675316f        /*!< Gia tốc Accel X offset (m/s^2) */
#define AyOFFSET    0.0720675737f       /*!< Gia tốc Accel Y offset (m/s^2) */
#define AzOFFSET    0.0f                /*!< Gia tốc Accel Z offset (m/s^2) */
#define gNORM       9.84484291f         /*!< Gia tốc trọng trường chuẩn g (m/s^2) */

/**
 * @brief Cấu trúc dữ liệu đo lường thu thập từ cảm biến BMI088
 */
typedef struct
{
    float Accel[3];         /*!< Gia tốc 3 trục [X, Y, Z] (m/s^2) */
    float Gyro[3];          /*!< Vận tốc góc 3 trục [X, Y, Z] (rad/s) */
    float TempWhenCali;     /*!< Nhiệt độ cảm biến tại thời điểm hiệu chuẩn (°C) */
    float Temperature;      /*!< Nhiệt độ tức thời của chip cảm biến (°C) */
    float AccelScale;       /*!< Hệ số tỉ lệ gia tốc kế */
    float GyroOffset[3];    /*!< Độ lệch tĩnh con quay 3 trục */
    float AccelOffset[3];   /*!< Độ lệch tĩnh gia tốc kế 3 trục */
    float gNorm;            /*!< Độ lớn vector trọng trường hiệu chuẩn */
} IMU_Data_t;

/**
 * @brief Bảng mã trạng thái lỗi khởi tạo và tự kiểm tra (Self-test) BMI088
 */
enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
};

/* --- Các hàm API công khai --- */
void BMI088_Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);
uint8_t BMI088_init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);

extern IMU_Data_t BMI088;

void BMI088_Read(IMU_Data_t *bmi088);

#ifdef __cplusplus
}
#endif

#endif /* BMI088DRIVER_H */
