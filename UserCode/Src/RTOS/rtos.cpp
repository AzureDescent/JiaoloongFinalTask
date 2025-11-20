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
constexpr float g_threshold = 0.15f;
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

int16_t debug_target_pitch = 20; // 手动修改此值来设定目标角度
volatile uint8_t debug_trigger_save = 0;  // 置 1 以保存当前数据到 feedforward_logs

struct TestData {
    float angle;      // 实际角度 (X轴)
    int16_t current;  // 输出电流 (Y轴)
    float target;     // 目标角度 (参考用)
};

TestData feedforward_logs[20];
int log_index = 0;

void VControlTask(void* argument)
{
    // 1. 初始化与缓启动
    gimbal_controller.Init();
    debug_target_pitch = 20;
    osDelay(500);

    // 读取当前姿态作为初始目标，防止上电瞬间大幅度动作
    EulerAngle_t start_attitude = imu_sensor.GetAttitude();
    gimbal_controller.SetImuFeedback(start_attitude);

    // 初始化调试目标为当前角度
    debug_target_pitch = start_attitude.pitch;

    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        // --- 读取数据 ---
        RemoteControl::ControlData rc_input = rc_controller.get_control_data();
        EulerAngle_t imu_attitude = imu_sensor.GetAttitude();

        osMutexAcquire(gimbal_mutex_handle, osWaitForever);

        gimbal_controller.SetImuFeedback(imu_attitude);

        // --- 1. 模式判断 (安全逻辑) ---
        // UP: PID控制(摇杆), MID: 前馈调试(LiveWatch), DOWN: 关(急停)
        Gimbal::Mode mode = gimbal_controller.DetermineMode(rc_input.switch_right);
        gimbal_controller.SetMode(mode);

        // --- 2. 根据模式执行控制 ---
        if (mode == Gimbal::GIMBAL_MODE_FEEDFORWARD_TEST)
        {
            // [前馈调试模式]
            // 限制输入范围，防止手误输入过大角度损坏机械结构
            if (debug_target_pitch > 25.0f) debug_target_pitch = 25.0f;
            if (debug_target_pitch < -25.0f) debug_target_pitch = -25.0f;

            // 应用 Live Watch 设置的目标
            gimbal_controller.target_pitch_angle_ = 1.8f;

            // [手动数据记录]
            // 当你觉得云台稳定后，将 debug_trigger_save 改为 1 即可记录
            if (debug_trigger_save == 1)
            {
                if (log_index < 20)
                {
                    feedforward_logs[log_index].angle = imu_sensor.GetAttitude().pitch;
                    feedforward_logs[log_index].current = gimbal_controller.GetPitchCurrentToSend();
                    feedforward_logs[log_index].target = debug_target_pitch;
                    log_index++;
                }
                else
                {
                    log_index = 0; // 循环覆盖，或者你也可以选择不覆盖
                }
                debug_trigger_save = 0; // 自动复位，等待下一次触发
            }
        }
        else if (mode == Gimbal::GIMBAL_MODE_PID)
        {
            // [普通PID模式] 允许用摇杆调整姿态
            gimbal_controller.SetPIDTargets(rc_input.yaw_stick, rc_input.pitch_stick);

            // 当切换回调试模式时，同步当前的 Pitch 到调试变量，防止跳变
            debug_target_pitch = gimbal_controller.target_pitch_angle_;
        }
        // mode == OFF 时，Handle() 会自动输出 0 力矩

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