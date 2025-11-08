//
// Created by DrownFish on 2025/11/4.
//
#include "bmi088.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"
#include "spi.h"

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

// TODO: Complete bmi088 init function
void Bmi088Init()
{

}

void Bmi088WriteByte(const uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX) {}
}

void Bmi088WriteReg(const uint8_t reg, const uint8_t data)
{
    Bmi088WriteByte(reg & 0x7F);
    Bmi088WriteByte(data);
}

void Bmi088ReadByte(uint8_t* rx_data, const uint8_t length)
{
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX) {}
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

void Bmi088AccelReadReg(uint8_t reg, uint8_t* rx_data, uint8_t length)
{
    uint8_t dummy_tx = 0xFF;

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
    uint8_t dummy_tx = 0xFF;

    Bmi088AccelNsH();
    Bmi088GyroNsL();

    Bmi088WriteByte(reg | 0x80);
    Bmi088WriteByte(dummy_tx);

    Bmi088ReadByte(return_data, length);

    Bmi088GyroNsH();
}