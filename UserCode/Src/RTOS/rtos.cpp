//
// Created by DrownFish on 2025/11/4.
//
#include "rtos.h"
#include "string.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "iwdg.h"

constexpr float dt = 0.001f;
constexpr float kg = 0.1f;
constexpr float g_threshold = 0.1f;
// TODO: Set correct gyro bias
constexpr float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };
constexpr float r_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

IMU imu_sensor(dt, kg, g_threshold, r_imu, gyro_bias);

[[noreturn]] void VImuTask(void* argument)
{
    uint32_t tick = osKernelGetTickCount();
    for (;;)
    {
        imu_sensor.ReadSensor();

        imu_sensor.UpdateAttitude();

        // TODO: Sync with Control Task if necessary

        osDelayUntil(tick += 2);
    }
}

extern "C" void IMU_Init_Wrapper()
{
    // C 文件中无法使用 C++ 构造函数，因此手动构造一个 C-compatible 的初始化角度
    EulerAngle_t init_angle(0, 0, 0);
    imu_sensor.Init(init_angle);
}