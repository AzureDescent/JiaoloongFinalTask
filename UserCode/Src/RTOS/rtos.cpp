//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"

void VImuTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        // TODO: Implement IMU data reading and attitude updating
        imu_sensor.readData();

        imu_sensor.update_attitude();

        // TODO: Sync

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
            gimbal_controller.set_mode(GIMBAL_MODE_OFF);
        }
    }
}

void VCanRecvTask(void* argument)
{
    uint8_t queue_message[sizeof(CAN_RxHeaderTypeDef) + 8];
    CAN_RxHeaderTypeDef rx_header;
    uint16_t rx_data[8];

    for (;;)
    {
        osMessageQueueGet(can_rx_queue_handle, &queue_message, nullptr, osWaitForever);

        memcpy(&rx_header, queue_message, sizeof(CAN_RxHeaderTypeDef));
        memcpy(rx_data, queue_message + sizeof(CAN_RxHeaderTypeDef), 8);

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.update_motor_feedback(rx_header.StdId, rx_data);

        osMutexRelease(gimbal_mutex_handle);
    }
}

void VControlTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        RemoteControl::ControlData rc_input = rc_controller.get_control_data();

        IMU::AttitudeData imu_attitude = imu_sensor.get_attitude();

        Gimbal::Mode mode = gimbal_controller.determine_mode(rc_input.switch_right);

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.set_mode(mode);
        gimbal_controller.set_targets(rc_input.yaw_stick, rc_input.pitch_stick);
        gimbal_controller.set_imu_feedback(imu_attitude);

        gimbal_controller.run_control_loop();

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

        int16_t yaw_current = gimbal_controller.get_yaw_motor_current();
        int16_t pitch_current = gimbal_controller.get_pitch_motor_current();

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
