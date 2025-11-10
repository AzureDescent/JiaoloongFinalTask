//
// Created by DrownFish on 2025/11/4.
//
#include "bmi088.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"
#include "spi.h"

#define BMI088_ACC_SOFTRESET_REG        0x7E
#define BMI088_GYRO_SOFTRESET_REG       0x14
#define BMI088_SOFTRESET_CMD            0xB6
#define BMI088_GYRO_RANGE_REG           0x0F
#define BMI088_GYRO_BANDWIDTH_REG       0x10
#define BMI088_ACC_PWR_CTRL_REG         0x7D
#define BMI088_ACC_CONF_REG             0x40
#define BMI088_ACC_RANGE_REG            0x41
#define BMI088_GYRO_RANGE_500DPS        0x02
#define BMI088_GYRO_ODR_100_BW_32       0x07
#define BMI088_ACC_PWR_NORMAL           0x04
#define BMI088_ACC_ODR_100              0x07
#define BMI088_ACC_RANGE_6G             0x01

void Bmi088AccelNsL()
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void Bmi088AccelNsH()
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void Bmi088GyroNsL()
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void Bmi088GyroNsH()
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

void Bmi088Init()
{
    Bmi088AccelNsL();
    Bmi088WriteReg(BMI088_ACC_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    Bmi088AccelNsH();
    HAL_Delay(2);

    Bmi088GyroNsL();
    Bmi088WriteReg(BMI088_GYRO_SOFTRESET_REG, BMI088_SOFTRESET_CMD);
    Bmi088GyroNsH();
    HAL_Delay(30);

    Bmi088GyroNsL();
    Bmi088WriteReg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_500DPS); // 0x0F, 0x02
    Bmi088WriteReg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_ODR_100_BW_32); // 0x10, 0x07
    Bmi088GyroNsH();

    Bmi088AccelNsL();
    Bmi088WriteReg(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_PWR_NORMAL); // 0x7D, 0x04
    Bmi088AccelNsH();
    HAL_Delay(2);

    Bmi088AccelNsL();
    Bmi088WriteReg(BMI088_ACC_CONF_REG, BMI088_ACC_ODR_100); // 0x40, 0x07
    Bmi088WriteReg(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_6G); // 0x41, 0x01
    Bmi088AccelNsH();
}

HAL_StatusTypeDef Bmi088AccelReadID(uint8_t* id)
{
    *id = 0; // 默认为 0
    // 调用已修复的 ReadReg 函数
    return Bmi088AccelReadReg(BMI088_ACC_CHIP_ID_REG, id, 1);
}

HAL_StatusTypeDef Bmi088WriteByte(const uint8_t tx_data)
{
    return HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100);
    // while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX) {}
}

HAL_StatusTypeDef Bmi088WriteReg(const uint8_t reg, const uint8_t data)
{
    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);
}

HAL_StatusTypeDef Bmi088ReadByte(uint8_t* rx_data, const uint8_t length)
{
    // 创建一个静态的（static）发送缓冲区，用于存放 Dummy Bytes
    // 静态缓冲区可以避免在 RTOS 任务中消耗栈空间，并且只需初始化一次
    // 我们假设一次最多读取 16 字节（例如读取6字节的陀螺仪数据）
    // 0xFF 是 SPI 通信中推荐的 Dummy Byte 值
    static uint8_t dummy_tx_buffer[16] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    // 安全检查，防止溢出
    if (length > 16)
    {
        return HAL_ERROR;
    }

    // 【核心】使用 TransmitReceive 函数：
    // 1. 发送 tx_dummy_buffer 中的 Dummy 数据 (产生 SCK 时钟)
    // 2. 同时接收传感器在 MISO 上的数据，存入 rx_data
    return HAL_SPI_TransmitReceive(&hspi1, dummy_tx_buffer, rx_data, length, 100);
}


// TODO: Check the dummy byte requirement for read operations
HAL_StatusTypeDef Bmi088AccelWriteSingleReg(const uint8_t reg, const uint8_t data)
{
    Bmi088GyroNsH();
    Bmi088AccelNsL();

    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);

    Bmi088AccelNsH();
}

HAL_StatusTypeDef Bmi088AccelReadReg(const uint8_t reg, uint8_t* rx_data, const uint8_t length)
{
    constexpr uint8_t dummy_tx = 0xFF;

    Bmi088GyroNsH();
    Bmi088AccelNsL();

    HAL_StatusTypeDef status;
    status = Bmi088WriteByte(reg | 0x80);
    // Bmi088WriteByte(dummy_tx);

    if (status != HAL_OK)
    {
        Bmi088AccelNsH(); // 传输失败，释放 CS
        return status;    // 返回错误状态
    }

    status = Bmi088ReadByte(rx_data, length); // 读取数据

    Bmi088AccelNsH();
    return status;
}

HAL_StatusTypeDef Bmi088GyroWriteSingleReg(const uint8_t reg, const uint8_t data)
{
    Bmi088AccelNsH();
    Bmi088GyroNsL();

    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);

    Bmi088GyroNsH();
}

HAL_StatusTypeDef Bmi088GyroReadReg(const uint8_t reg, uint8_t* return_data, const uint8_t length)
{
    Bmi088AccelNsH();
    Bmi088GyroNsL();

    HAL_StatusTypeDef status;
    status = Bmi088WriteByte(reg | 0x80); // 发送地址

    if (status != HAL_OK)
    {
        Bmi088GyroNsH(); // 传输失败，释放 CS
        return status;   // 返回错误状态
    }

    status = Bmi088ReadByte(return_data, length); // 读取数据

    Bmi088GyroNsH();
    return status;
}