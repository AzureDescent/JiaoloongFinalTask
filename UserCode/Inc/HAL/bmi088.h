//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_BMI088_H
#define FINALTASK_BMI088_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BMI088_ACC_CHIP_ID_REG      0x00
#define BMI088_ACC_CHIP_ID_VAL      0x1E

// chip selection
void Bmi088AccelNsL(void);
void Bmi088AccelNsH(void);
void Bmi088GyroNsL(void);
void Bmi088GyroNsH(void);

// bmi088 init
void Bmi088Init(void);

HAL_StatusTypeDef Bmi088AccelReadID(uint8_t* id);

// bmi088 read/write
HAL_StatusTypeDef Bmi088WriteByte(uint8_t tx_data);
HAL_StatusTypeDef Bmi088WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef Bmi088ReadByte(uint8_t* rx_data, uint8_t length);

HAL_StatusTypeDef Bmi088AccelWriteSingleReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef Bmi088AccelReadReg(uint8_t reg, uint8_t* rx_data, uint8_t length);
HAL_StatusTypeDef Bmi088GyroWriteSingleReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef Bmi088GyroReadReg(uint8_t reg, uint8_t* return_data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif //FINALTASK_BMI088_H