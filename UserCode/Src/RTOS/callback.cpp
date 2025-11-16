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

extern uint8_t rx_buf[18];
extern uint8_t rx_data[18];

extern RemoteControl rc_controller;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18);

        uint32_t current_tick = osKernelGetTickCount();

        if (current_tick - rc_controller.lastTick > 500)
        {
            rc_controller.is_connected = false;
        }
        else
        {
            rc_controller.is_connected = true;

            memcpy(rx_data, rx_buf, 18);

            osSemaphoreRelease(rc_data_ready_semaphore_handle);
        }

        rc_controller.lastTick = current_tick;
    }
}

extern "C" void HAL_IncTick(void);

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
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
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

extern CAN_HandleTypeDef hcan1;
extern osMessageQueueId_t can_rx_queue_handle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
            memcpy(queue_message, &rx_header, sizeof(CAN_RxHeaderTypeDef));
            memcpy(queue_message + sizeof(CAN_RxHeaderTypeDef), rx_data, 8);

            osMessageQueuePut(can_rx_queue_handle, &queue_message, 0U, 0U);
        }
    }
}