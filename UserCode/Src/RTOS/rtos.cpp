//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"

Gimbal gimbal_controller;
IMU imu_sensor;
RemoteControl rc_controller;

osThreadId_t control_task_handle;
osThreadId_t imu_task_handle;
osThreadId_t can_send_task_handle;
osThreadId_t can_recv_task_handle;
osThreadId_t iwdg_task_handle;

osThreadId_t can_rx_queue_handle;

osSemaphoreId_t rc_data_ready_semaphore_handle;
osSemaphoreId_t imu_data_ready_semaphore_handle;

osMutexId_t gimbal_mutex_handle;

