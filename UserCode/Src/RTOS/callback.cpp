//
// Created by DrownFish on 2025/11/4.
//
#include "can.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "string.h"
#include "dma.h"
// FIXME: resolve the inclusion relationship of header files after the project is stable

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        // TODO: Receive data processing in Class Remote Control
        // URL: https://github.com/AzureDescent/C-Type_Board/blob/RemoteControl/Core/Src/callback.cpp

        // TODO: Define rcDataReadySemaphoreHandle appropriately in RTOS setup
        osSemaphoreRelease(rcDataReadySemaphoreHandle);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan -> Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
            // TODO: Process queue_message better as needed
            memcpy(queue_message, &rx_header, sizeof(CAN_RxHeaderTypeDef));
            memcpy(queue_message + sizeof(CAN_RxHeaderTypeDef), rx_data, 8);

            // TODO: Define canRxQueueHandle appropriately in RTOS setup
            osMessageQueuePut(canRxQueueHandle, &queue_message, 0, 0);
        }
    }
}

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