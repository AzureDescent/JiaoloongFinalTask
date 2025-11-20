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
constexpr float g_threshold = 0.1f;
// TODO: Set correct gyro bias
constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

Gimbal gimbal_controller;
IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);
RemoteControl rc_controller;

uint8_t rx_buf[18];
uint8_t rx_data[18];


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
    uint32_t tick = osKernelGetTickCount();
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

struct TestData {
    float angle;      // 实际角度 (X轴)
    int16_t current;  // 输出电流 (Y轴)
    float target;     // 目标角度 (参考用)
};

// 记录 20 组数据，调试时在 Live Watch 中展开这个数组查看
TestData feedforward_logs[20];
int log_index = 0;

void VControlTask(void* argument)
{
    // 1. 初始化与缓启动
    gimbal_controller.Init();
    osDelay(500);

    // 读取当前姿态作为初始目标，防止上电瞬间炸机
    EulerAngle_t start_attitude = imu_sensor.GetAttitude();
    gimbal_controller.SetImuFeedback(start_attitude);

    // 强制进入前馈测试模式 (输出 = PID计算值，无额外叠加)

    // 2. 定义测试序列 (涵盖水平、正向、负向)
    // 根据你的云台活动范围调整，一般测 -30 到 +30 度即可
    const float test_sequence[] = {
        0.0f,
        5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f,
        0.0f, // 回中
        -5.0f, -10.0f, -15.0f, -20.0f, -25.0f, -30.0f
    };

    const int sequence_len = sizeof(test_sequence) / sizeof(float);
    int seq_idx = 0;
    uint32_t state_timer = osKernelGetTickCount();
    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        // --- 自动测试逻辑 ---
        // 每 4000ms (4秒) 切换一次角度，给电机足够时间稳定
        if (osKernelGetTickCount() - state_timer > 4000)
        {
            // A. 在切换前，记录上一阶段稳定后的数据
            if (log_index < 20 && seq_idx < sequence_len)
            {
                feedforward_logs[log_index].angle = imu_sensor.GetAttitude().pitch;
                feedforward_logs[log_index].current = gimbal_controller.GetPitchCurrentToSend();
                feedforward_logs[log_index].target = test_sequence[seq_idx];
                log_index++;
            }

            // B. 切换到下一个目标
            seq_idx++;
            if (seq_idx >= sequence_len)
            {
                seq_idx = 0;   // 跑完一轮回到开头
                log_index = 0; // 重置记录，循环覆盖
            }

            state_timer = osKernelGetTickCount();
        }

        // 获取当前目标
        float current_target = test_sequence[seq_idx];

        // --- 控制循环 ---
        RemoteControl::ControlData rc_input = rc_controller.get_control_data();
        EulerAngle_t imu_attitude = imu_sensor.GetAttitude();

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.SetImuFeedback(imu_attitude);
        // ================== 修改开始 ==================

        // 1. 读取开关状态并设置模式
        // 根据 Gimbal::DetermineMode 的逻辑: UP->PID, MID->TEST, DOWN->OFF
        Gimbal::Mode target_mode = gimbal_controller.DetermineMode(rc_input.switch_right);
        gimbal_controller.SetMode(target_mode);

        // 2. 根据当前模式执行逻辑
        if (target_mode == Gimbal::GIMBAL_MODE_FEEDFORWARD_TEST)
        {
            // 中档：执行自动前馈测试逻辑，强制覆盖目标角度
            gimbal_controller.target_pitch_angle_ = current_target;
        }
        else if (target_mode == Gimbal::GIMBAL_MODE_PID)
        {
            // 上档：恢复遥控器控制 (PID模式)，用于手动复位或检查
            gimbal_controller.SetPIDTargets(rc_input.yaw_stick, rc_input.pitch_stick);
        }
        // 下档 (OFF)：Handle() 会自动输出 0 力矩，此处无需操作

        // ================== 修改结束 ==================

        gimbal_controller.Handle();
        gimbal_controller.UpdateCurrentCommands();

        osMutexRelease(gimbal_mutex_handle);

        osDelayUntil(tick += 10);
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