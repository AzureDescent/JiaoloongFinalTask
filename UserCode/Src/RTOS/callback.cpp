//
// Created by DrownFish on 2025/11/4.
//
#include "can.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "string.h"
#include "rtos.h"
#include "dma.h"

#include "FreeRTOS.h"
#include "task.h"
// FIXME: resolve the inclusion relationship of header files after the project is stable

// 声明外部 HAL 库函数（如果需要的话，但通常 tim.h 中已包含）
extern "C" void HAL_IncTick(void);

// 声明 TIM6 句柄（它在 stm32f4xx_hal_timebase_tim.c 中定义，但我们需要在此处引用）
// 通常更好的做法是在 tim.h 中 extern 声明它
extern TIM_HandleTypeDef htim6;


uint32_t count = 0;

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */

    // 关键修复：保留 HAL Timebase (TIM6) 的逻辑
    // 注意：您需要确认 HAL timebase 确实是 TIM6。
    // 如果您在 stm32f4xx_hal_timebase_tim.c 中看到 htim6，这里就用 TIM6。
    if (htim->Instance == TIM6) // 或者 htim->Instance == htim6.Instance
    {
        HAL_IncTick();
    }

    /* USER CODE BEGIN Callback 1 */

    // 您自己的 TIM7 逻辑
    if (htim->Instance == TIM7) //
    {
        count++;
        if (count >= 10000)
        {
            count = 0;
        }

        // TODO: 在这里添加 TIM7 需要执行的任何操作
        // 例如：发送信号量或通知 VImuTask
    }

    /* USER CODE END Callback 1 */
}