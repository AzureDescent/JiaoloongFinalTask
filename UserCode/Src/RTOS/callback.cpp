//
// Created by DrownFish on 2025/11/4.
//
#include "can.h"
#include "tim.h"
#include "cmsis_os2.h"
// FIXME: "cmsis_os2.h" is temporary for osMessageQueuePut, which may be removed after integrating with other head files

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan -> Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
            // TODO: Process queue_message as needed

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