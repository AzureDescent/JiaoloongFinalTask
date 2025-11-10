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
    // Soft Reset ACCEL
    Bmi088AccelNsL();
    Bmi088WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    Bmi088AccelNsH();

    // Soft Reset GYRO
    Bmi088GyroNsL();
    Bmi088WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    Bmi088GyroNsH();

    Bmi088GyroNsL();
    HAL_Delay(1);
    Bmi088WriteReg(0x0F, 0x02); // 设置GYRO_RANGE为500dps
    HAL_Delay(1);
    Bmi088GyroNsH();

    Bmi088GyroNsL();
    HAL_Delay(1);
    Bmi088WriteReg(0x10, 0x07); // 设置100Hz ODR, 100Hz滤波
    HAL_Delay(1);
    Bmi088GyroNsH();

    // Switch ACCEL to Normal Mode
    Bmi088AccelNsL();
    HAL_Delay(1);
    Bmi088WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    Bmi088AccelNsH();

    //Gemini
    Bmi088AccelNsL();
    HAL_Delay(1);
    Bmi088WriteReg(0x40, 0x07); // Write 0x07 to ACC_CONF(0x40) for 100Hz ODR
    HAL_Delay(1);
    Bmi088AccelNsH();

    // Set ACCEL Range (ACC_RANGE - 0x41)
    // Example: Set Range to +/- 6g (0x01)
    // 0x41: Range = 6g (0x01). Value: 0x01
    Bmi088AccelNsL();
    HAL_Delay(1);
    Bmi088WriteReg(0x41, 0x01); // Write 0x01 to ACC_RANGE(0x41) for +/- 6g
    HAL_Delay(1);
    Bmi088AccelNsH();
}

void Bmi088WriteByte(const uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

void Bmi088WriteReg(const uint8_t reg, const uint8_t data)
{
    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);
}

void Bmi088ReadByte(uint8_t* rx_data, const uint8_t length)
{
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

// TODO: Check the dummy byte requirement for read operations
void Bmi088AccelWriteSingleReg(const uint8_t reg, const uint8_t data)
{
    Bmi088GyroNsH();
    Bmi088AccelNsL();

    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);

    Bmi088AccelNsH();
}

void Bmi088AccelReadReg(const uint8_t reg, uint8_t* rx_data, const uint8_t length)
{
    constexpr uint8_t dummy_tx = 0xFF;

    Bmi088GyroNsH();
    Bmi088AccelNsL();

    Bmi088WriteByte(reg | 0x80);
    Bmi088WriteByte(dummy_tx);

    Bmi088ReadByte(rx_data, length);

    Bmi088AccelNsH();
}

void Bmi088GyroWriteSingleReg(const uint8_t reg, const uint8_t data)
{
    Bmi088AccelNsH();
    Bmi088GyroNsL();

    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);

    Bmi088GyroNsH();
}

void Bmi088GyroReadReg(const uint8_t reg, uint8_t* return_data, const uint8_t length)
{
    constexpr uint8_t dummy_tx = 0xFF;

    Bmi088AccelNsH();
    Bmi088GyroNsL();

    Bmi088WriteByte(reg | 0x80);
    Bmi088WriteByte(dummy_tx);

    Bmi088ReadByte(return_data, length);

    Bmi088GyroNsH();
}

/**
  * @brief  读取 BMI088 加速度计的芯片 ID
  */
uint8_t Bmi088CheckAccelId(void)
{
    uint8_t acc_id = 0;
    // 芯片 ID 寄存器地址为 0x00
    Bmi088AccelReadReg(0x00, &acc_id, 1);
    return acc_id;
}

/**
  * @brief  读取 BMI088 陀螺仪的芯片 ID
  */
uint8_t Bmi088CheckGyroId(void)
{
    uint8_t gyro_id = 0;
    // 芯片 ID 寄存器地址为 0x00
    Bmi088GyroReadReg(0x00, &gyro_id, 1);
    return gyro_id;
}