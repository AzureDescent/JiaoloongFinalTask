//
// Created by DrownFish on 2025/11/4.
//
#include "can.h"
#include "tim.h"

uint32_t count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM7)
    {
        count++;
        if (count >= 10000)
        {
            count = 0;
        }
    }
}