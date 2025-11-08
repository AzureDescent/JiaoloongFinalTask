//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "iwdg.h"

void VImuTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        // TODO: Implement IMU data reading and attitude updating
        imu_sensor.ReadSensor();

        imu_sensor.UpdateAttitude();

        // TODO: Sync with Control Task if necessary

        osDelayUntil(tick += 2);
    }
}

void VRcProcessTask(void* argument)
{
    for (;;)
    {
        osSemaphoreAcquire(rc_data_ready_semaphore_handle, osWaitForever);

        rc_controller.processData();

        if (rc_controller.is_offline())
        {
            gimbal_controller.SetMode(Gimbal::GIMBAL_MODE_OFF);
        }
    }
}

void VCanRecvTask(void* argument)
{
    uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    for (;;)
    {
        osMessageQueueGet(can_rx_queue_handle, &queue_message, nullptr, osWaitForever);

        memcpy(&rx_header, queue_message, sizeof(CAN_RxHeaderTypeDef));
        memcpy(rx_data, queue_message + sizeof(CAN_RxHeaderTypeDef), 8);

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.UpdateMotorFeedback(rx_header.StdId, rx_data);

        osMutexRelease(gimbal_mutex_handle);
    }
}

void VControlTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        RemoteControl::ControlData rc_input = rc_controller.get_control_data();

        // TODO: Implement getting IMU attitude data
        IMU::AttitudeData imu_attitude = imu_sensor.get_attitude();

        Gimbal::Mode mode = gimbal_controller.DetermineMode(rc_input.switch_right);

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.SetMode(mode);
        gimbal_controller.SetPIDTargets(rc_input.yaw_stick, rc_input.pitch_stick);
        gimbal_controller.SetImuFeedback(imu_attitude);

        gimbal_controller.RunControlLoop();

        osMutexRelease(gimbal_mutex_handle);

        osDelayUntil(tick += 10);
    }
}

void VCanSendTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    for (;;)
    {
        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        int16_t yaw_current = gimbal_controller.GetYawMotorCurrent();
        int16_t pitch_current = gimbal_controller.GetPitchMotorCurrent();

        osMutexRelease(gimbal_mutex_handle);

        // TODO: Package CAN datas for motor currents

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

        osDelayUntil(tick += 1);
    }
}

void VIwdgTask(void* argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);

        osDelay(1000);
    }
}