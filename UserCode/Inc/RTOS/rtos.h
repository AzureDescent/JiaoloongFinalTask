//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RTOS_H
#define FINALTASK_RTOS_H
#include "cmsis_os2.h"
#include "imu.h"

// TODO: Add and FIX task handle declarations
extern osThreadId_t imu_task_handle;
extern osSemaphoreId_t imu_data_ready_semaphore_handle;

#ifdef __cplusplus // 保持 extern "C" 结构
extern "C"
{
#endif

void VControlTask(void* argument);
[[noreturn]] void VImuTask(void* argument);
[[noreturn]] void VCanSendTask(void* argument);
[[noreturn]] void VCanRecvTask(void* argument);
[[noreturn]] void VRcProcessTask(void* argument);
[[noreturn]] void VIwdgTask(void* argument);

#ifdef __cplusplus
}
#endif

#endif //FINALTASK_RTOS_H