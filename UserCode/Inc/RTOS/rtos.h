//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RTOS_H
#define FINALTASK_RTOS_H
#include "cmsis_os2.h"
#include "Gimbal.h"

// TODO: Add and FIX external declarations

// TODO: Add and FIX task handle declarations
extern osThreadId_t imu_task_handle;

[[noreturn]] void VImuTask(void *argument);


#endif //FINALTASK_RTOS_H