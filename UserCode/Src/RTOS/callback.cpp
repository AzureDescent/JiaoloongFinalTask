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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        // TODO: Receive data processing in Class Remote Control
        // URL: https://github.com/AzureDescent/C-Type_Board/blob/RemoteControl/Core/Src/callback.cpp

        // TODO： extern uint8_t rx_buf[18];
        // TODO： extern uint8_t rx_data[18];

        // TODO： First copy DMA buffer to local buffer to avoid data corruption
        // HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18);

        // TODO: Define rc_data_ready_semaphore_handle appropriately in RTOS setup
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