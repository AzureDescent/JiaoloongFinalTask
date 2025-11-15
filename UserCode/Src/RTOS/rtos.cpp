//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "iwdg.h"
#include "rc.h"

constexpr float dt = 0.001f;
constexpr float kg = 0.1f;
constexpr float g_threshold = 0.1f;
// TODO: Set correct gyro bias
constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

// Gimbal gimbal_controller;
IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);
RemoteControl rc_controller;

// 定义遥控器 DMA 接收缓冲区和数据处理缓冲区
// callback.cpp 将通过 extern 引用它们
uint8_t rx_buf[18];
uint8_t rx_data[18];

[[noreturn]] void VImuTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        imu_sensor.ReadSensor();

        imu_sensor.UpdateAttitude();

        // TODO: Sync with Control Task if necessary

        osDelayUntil(tick += 1);
    }
}

[[noreturn]] void VRcProcessTask(void* argument)
{
    for (;;)
    {
        osSemaphoreAcquire(rc_data_ready_semaphore_handle, osWaitForever);

        rc_controller.Handle(rx_data);

        if (rc_controller.IsOffline())
        {
            gimbal_controller.SetMode(Gimbal::GIMBAL_MODE_OFF);
        }
    }
}

[[noreturn]] void VCanRecvTask(void* argument)
{
    // uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
    // CAN_RxHeaderTypeDef rx_header;
    // uint8_t rx_data[8];

    for (;;)
    {
        // osMessageQueueGet(can_rx_queue_handle, &queue_message, nullptr, osWaitForever);
        //
        // memcpy(&rx_header, queue_message, sizeof(CAN_RxHeaderTypeDef));
        // memcpy(rx_data, queue_message + sizeof(CAN_RxHeaderTypeDef), 8);
        //
        // osMutexAcquire(gimbal_mutex_handle, osWaitForever);
        //
        // gimbal_controller.UpdateMotorFeedback(rx_header.StdId, rx_data);
        //
        // osMutexRelease(gimbal_mutex_handle);

        osDelay(1000); // 任务保留，但进入休眠
    }
}

void VControlTask(void* argument)
{
    // [DISABLED FOR TEST] 任务已在 main.c 禁用 (或未创建)，注释掉内部逻辑
    // uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        //     RemoteControl::ControlData rc_input = rc_controller.get_control_data();
        //
        //     EulerAngle_t imu_attitude = imu_sensor.GetAttitude();
        //
        //     Gimbal::Mode mode = gimbal_controller.DetermineMode(rc_input.switch_right);
        //
        //     osMutexAcquire(gimbal_mutex_handle, osWaitForever);
        //
        //     gimbal_controller.SetMode(mode);
        //     gimbal_controller.SetPIDTargets(rc_input.yaw_stick, rc_input.pitch_stick);
        //     gimbal_controller.SetImuFeedback(imu_attitude);
        //
        //     gimbal_controller.RunControlLoop();
        //
        //     osMutexRelease(gimbal_mutex_handle);
        //
        //     osDelayUntil(tick += 10);

        osDelay(1000); // 任务保留，但进入休眠
    }
}

[[noreturn]] void VCanSendTask(void* argument)
{
    // [DISABLED FOR TEST] 任务已在 main.c 禁用，注释掉内部逻辑
    // uint32_t tick = osKernelGetTickCount();
    // CAN_TxHeaderTypeDef tx_header;
    // uint8_t tx_data[8];
    // uint32_t tx_mailbox;

    for (;;)
    {
        //     osMutexAcquire(gimbal_mutex_handle, osWaitForever);
        //
        //     int16_t yaw_current = gimbal_controller.GetYawMotorCurrent();
        //     int16_t pitch_current = gimbal_controller.GetPitchMotorCurrent();
        //
        //     osMutexRelease(gimbal_mutex_handle);
        //
        //     // TODO: Package CAN datas for motor currents
        //
        //     HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
        //
        //     osDelayUntil(tick += 1);

        osDelay(1000); // 任务保留，但进入休眠
    }
}

[[noreturn]] void VIwdgTask(void* argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);

        osDelay(1000);
    }
}
extern "C" void IMU_Init_Wrapper()
{
    EulerAngle_t init_angle(0, 0, 0);
    imu_sensor.Init(init_angle);
}