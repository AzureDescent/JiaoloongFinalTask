//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_BMI088_H
#define FINALTASK_BMI088_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

// chip selection
void Bmi088AccelNsL(void);
void Bmi088AccelNsH(void);
void Bmi088GyroNsL(void);
void Bmi088GyroNsH(void);

// bmi088 init
void Bmi088Init(void);

// bmi088 read/write
void Bmi088WriteByte(uint8_t tx_data);
void Bmi088WriteReg(uint8_t reg, uint8_t data);
void Bmi088ReadByte(uint8_t* rx_data, uint8_t length);

void Bmi088AccelWriteSingleReg(uint8_t reg, uint8_t data);
void Bmi088AccelReadReg(uint8_t reg, uint8_t* rx_data, uint8_t length);
void Bmi088GyroWriteSingleReg(uint8_t reg, uint8_t data);
void Bmi088GyroReadReg(uint8_t reg, uint8_t* return_data, uint8_t length);

/**
  * @brief  读取 BMI088 加速度计的芯片 ID (用于调试)
  * @retval uint8_t 芯片 ID (预期值: 0x1E 或 30)
  */
uint8_t Bmi088CheckAccelId(void);

/**
  * @brief  读取 BMI088 陀螺仪的芯片 ID (用于调试)
  * @retval uint8_t 芯片 ID (预期值: 0x0F 或 15)
  */
uint8_t Bmi088CheckGyroId(void);


#ifdef __cplusplus
}
#endif

#endif //FINALTASK_BMI088_H