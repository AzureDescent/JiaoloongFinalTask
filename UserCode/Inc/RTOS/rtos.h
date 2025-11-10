//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RTOS_H
#define FINALTASK_RTOS_H
#include "cmsis_os2.h"
#include "Gimbal.h"

#ifdef __cplusplus
extern "C" {
    #endif

    // TODO: Add and FIX external declarations

    // TODO: Add and FIX task handle declarations
    extern osThreadId_t imu_task_handle;

    extern volatile uint8_t g_accel_id;
    extern volatile uint8_t g_gyro_id;
    [[noreturn]] void VImuTask(void *argument);

    #ifdef __cplusplus
}
#endif

#endif //FINALTASK_RTOS_H