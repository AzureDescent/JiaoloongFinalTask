//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RTOS_H
#define FINALTASK_RTOS_H
#include "cmsis_os2.h"
#include "Gimbal.h"

// TODO: Add necessary for RTOS setup here

// TODO: Add and FIX external declarations
extern Gimbal gimbal_controller;
extern IMU imu_sensor;
extern RemoteControl rc_controller;

// TODO: Add and FIX task handle declarations
extern osThreadId_t control_task_handle;
extern osThreadId_t imu_task_handle;
extern osThreadId_t can_send_task_handle;
extern osThreadId_t can_recv_task_handle;
extern osThreadId_t rc_process_task_handle;
extern osThreadId_t iwdg_task_handle;

extern osThreadId_t can_rx_queue_handle;

extern osSemaphoreId_t rc_data_ready_semaphore_handle;
extern osSemaphoreId_t imu_data_ready_semaphore_handle;

extern osMutexId_t gimbal_mutex_handle;

void VControlTask(void *argument);
void VImuTask(void *argument);
void VCanSendTask(void *argument);
void VCanRecvTask(void *argument);
void VRcProcessTask(void *argument);
void VIwdgTask(void *argument);


#endif //FINALTASK_RTOS_H