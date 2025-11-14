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

// 声明在 rtos.cpp 中定义的全局缓冲区
extern uint8_t rx_buf[18];
extern uint8_t rx_data[18];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        // 1. 首先将 DMA 缓冲区 (rx_buf) 的内容复制到数据处理缓冲区 (rx_data)
        //    这样可以防止在 VRcProcessTask 处理数据时，DMA 覆盖了旧数据
        memcpy(rx_data, rx_buf, 18);

        // 2. 重新启动 DMA 接收
        //    这是至关重要的，否则下一次数据将无法接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18);

        // 3. 释放信号量，通知 VRcProcessTask 任务有新数据需要处理
        osSemaphoreRelease(rc_data_ready_semaphore_handle);
    }
}

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

    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }

    /* USER CODE BEGIN Callback 1 */

    if (htim->Instance == TIM7) //
    {
        count++;
        if (count >= 10000)
        {
            count = 0;
        }

        // TODO: Add TIM7 interrupt handling code here
    }

    /* USER CODE END Callback 1 */
}