//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "iwdg.h"
#include "rc.h"

constexpr float dt = 0.002f;
constexpr float kg = 0.1f;
constexpr float g_threshold = 0.5f;
// TODO: Set correct gyro bias
constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

Gimbal gimbal_controller;
IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);
RemoteControl rc_controller;

uint8_t rx_buf[18];
uint8_t rx_data[18];

// 阶跃幅度 (例如 10.0 度)
volatile float debug_step_amp = 10.0f;

// 阶跃周期 (例如 2000ms，即 1秒正 1秒负)
volatile uint32_t debug_step_period = 4000;

// 阶跃中心点 (例如 0 度)
volatile float debug_step_offset = 0.0f;

void FillMotorCurrent(const int id, const int16_t current, uint8_t* data_1fe, uint8_t* data_2fe)
{
    const uint8_t high_byte = (current >> 8) & 0xFF;
    const uint8_t low_byte = current & 0xFF;
    int offset;

    if (id >= 1 && id <= 4)
    {
        offset = (id - 1) * 2;
        data_1fe[offset] = high_byte;
        data_1fe[offset + 1] = low_byte;
    }
    else if (id >= 5 && id <= 7)
    {
        offset = (id - 5) * 2;
        data_2fe[offset] = high_byte;
        data_2fe[offset + 1] = low_byte;
    }
}

[[noreturn]] void VImuTask(void* argument)
{
    for (;;)
    {
        imu_sensor.ReadSensor();

        imu_sensor.UpdateAttitude();

        osDelay(2);
    }
}

[[noreturn]] void VRcProcessTask(void* argument)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 18) != HAL_OK)
    {
        Error_Handler();
    }

    uint8_t local_rx_data[18];

    for (;;)
    {
        osSemaphoreAcquire(rc_data_ready_semaphore_handle, osWaitForever);

        memcpy(local_rx_data, rx_data, 18);

        rc_controller.is_connected = true;
        rc_controller.Handle(rx_data);
    }
}

[[noreturn]] void VCanRecvTask(void* argument)
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
    gimbal_controller.Init();
    osDelay(500);

    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        if (rc_controller.IsOffline())
        {
            osMutexAcquire(gimbal_mutex_handle, osWaitForever);
            gimbal_controller.SetMode(Gimbal::GIMBAL_MODE_OFF);
            osMutexRelease(gimbal_mutex_handle);
        }

        RemoteControl::ControlData rc_input = rc_controller.get_control_data();

        EulerAngle_t imu_attitude = imu_sensor.GetAttitude();

        Gimbal::Mode mode = gimbal_controller.DetermineMode(rc_input.switch_right);

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.SetImuFeedback(imu_attitude);

        gimbal_controller.SetMode(mode);

        // ==================
        uint32_t current_time = osKernelGetTickCount();

        // 利用取余运算生成方波: 前半周期为正，后半周期为负
        if ((current_time % debug_step_period) < (debug_step_period / 2))
        {
            // 上半波: 中心 + 幅度
            // 如果是调 Pitch，就改 target_pitch_angle_
            gimbal_controller.target_pitch_angle_ = debug_step_offset + debug_step_amp;
        }
        else
        {
            // 下半波: 中心 - 幅度
            gimbal_controller.target_pitch_angle_ = debug_step_offset - debug_step_amp;
        }
        // ====================

        // gimbal_controller.SetPIDTargets(rc_input.yaw_stick, rc_input.pitch_stick);

        gimbal_controller.Handle();

        gimbal_controller.UpdateCurrentCommands();

        osMutexRelease(gimbal_mutex_handle);

        osDelayUntil(tick += 1);
    }
}

[[noreturn]] void VCanSendTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    uint32_t tx_mailbox;

    CAN_TxHeaderTypeDef tx_header_1_fe = {
        .StdId = 0x1FE,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };
    uint8_t tx_data_1_fe[8];

    CAN_TxHeaderTypeDef tx_header_2_fe = {
        .StdId = 0x2FE,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };
    uint8_t tx_data_2_fe[8];

    for (;;)
    {
        int16_t pitch_current = gimbal_controller.GetPitchCurrentToSend();
        int16_t yaw_current = gimbal_controller.GetYawCurrentToSend();

        int pitch_id = gimbal_controller.GetPitchMotorID() - 0x204;
        int yaw_id = gimbal_controller.GetYawMotorID() - 0x204;

        memset(tx_data_1_fe, 0, sizeof(tx_data_1_fe));
        memset(tx_data_2_fe, 0, sizeof(tx_data_2_fe));

        FillMotorCurrent(pitch_id, pitch_current, tx_data_1_fe, tx_data_2_fe);
        FillMotorCurrent(yaw_id, yaw_current, tx_data_1_fe, tx_data_2_fe);

        HAL_CAN_AddTxMessage(&hcan1, &tx_header_1_fe, tx_data_1_fe, &tx_mailbox);
        HAL_CAN_AddTxMessage(&hcan1, &tx_header_2_fe, tx_data_2_fe, &tx_mailbox);

        osDelayUntil(tick += 1);
    }
}

[[noreturn]] void VIwdgTask(void* argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);

        osDelay(500);
    }
}

extern "C" void ImuInitWrapper()
{
    EulerAngle_t init_angle(0, 0, 0);
    imu_sensor.Init(init_angle);
}