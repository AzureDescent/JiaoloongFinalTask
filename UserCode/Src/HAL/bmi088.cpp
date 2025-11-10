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

uint8_t Bmi088AccelReadID()
{
    uint8_t id = 0;
    // Bmi088AccelReadReg 负责处理片选和读写时序
    Bmi088AccelReadReg(BMI088_ACC_CHIP_ID_REG, &id, 1);
    return id;
}

void Bmi088WriteByte(const uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
    // while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX) {}
}

void Bmi088WriteReg(const uint8_t reg, const uint8_t data)
{
    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);
}

void Bmi088ReadByte(uint8_t* rx_data, const uint8_t length)
{
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    // while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX) {}
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
    // Bmi088WriteByte(dummy_tx);

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
    // Bmi088WriteByte(dummy_tx);

    Bmi088ReadByte(return_data, length);

    Bmi088GyroNsH();
}