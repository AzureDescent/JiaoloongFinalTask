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

extern RemoteControl rc_controller;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        // 1. 立即重启 DMA，准备接收下一帧 (这是正确的)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18);

        // 2. 获取当前时间（使用 RTOS 的时钟）
        uint32_t current_tick = osKernelGetTickCount();

        // 3. 恢复 C 板的超时检查逻辑 (使用 500ms 作为阈值)
        if (current_tick - rc_controller.lastTick > 500)
        {
            rc_controller.is_connected = false;
            // 如果超时（刚启动或信号丢失），不处理数据（不释放信号量）
        }
        else // 4. 未超时，数据被认为是有效的
        {
            rc_controller.is_connected = true;

            // 复制有效数据
            memcpy(rx_data, rx_buf, 18);

            // 释放信号量，让 VRcProcessTask 任务去处理
            osSemaphoreRelease(rc_data_ready_semaphore_handle);
        }

        // 5. 始终更新 lastTick（模仿 C 板的逻辑）
        //    这样 IsOffline() 才能正确工作
        rc_controller.lastTick = current_tick;
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