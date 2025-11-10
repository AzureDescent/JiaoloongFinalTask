//
// Created by DrownFish on 2025/11/4.
//
#include "imu.h"
#include "string.h"
#include "cmath"
#include "stm32f4xx_hal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


IMU::IMU(const float& dt,
         const float& kg,
         const float& g_threshold,
         const float R_imu[3][3],
         const float gyro_bias[3]):
    mahony_(dt, kg, g_threshold)
{
    memcpy(R_imu_, R_imu, 9 * sizeof(float));
    memcpy(gyro_bias_, gyro_bias, 3 * sizeof(float));
}

void IMU::Init(EulerAngle_t euler_deg_init)
{
    Bmi088Init();
    HAL_Delay(50);

    euler_deg_ = euler_deg_init;

    euler_rad_.yaw = euler_deg_init.yaw * M_PI / 180.0f;
    euler_rad_.pitch = euler_deg_init.pitch * M_PI / 180.0f;
    euler_rad_.roll = euler_deg_init.roll * M_PI / 180.0f;

    float cy = cosf(euler_rad_.yaw * 0.5f);
    float sy = sinf(euler_rad_.yaw * 0.5f);
    float cp = cosf(euler_rad_.pitch * 0.5f);
    float sp = sinf(euler_rad_.pitch * 0.5f);
    float cr = cosf(euler_rad_.roll * 0.5f);
    float sr = sinf(euler_rad_.roll * 0.5f);

    q_[0] = cr * cp * cy + sr * sp * sy; // w
    q_[1] = sr * cp * cy - cr * sp * sy; // x
    q_[2] = cr * sp * cy + sr * cp * sy; // y
    q_[3] = cr * cp * sy - sr * sp * cy; // z

    constexpr uint8_t accel_range_setting = 0x01;
    Bmi088AccelWriteSingleReg(BMI088_ACC_RANGE_REG, accel_range_setting);

    switch (accel_range_setting)
    {
        case 0:
            accel_scale_factor_ = 3.0f;
            break;
        case 1:
            accel_scale_factor_ = 6.0f;
            break;
        case 2:
            accel_scale_factor_ = 12.0f;
            break;
        case 3:
            accel_scale_factor_ = 24.0f;
            break;
        default:
            accel_scale_factor_ = 6.0f;
            break;;
    }

    constexpr uint8_t gyro_range_setting = 0x00;
    Bmi088GyroWriteSingleReg(BMI088_GYRO_RANGE_REG, gyro_range_setting);

    switch (gyro_range_setting)
    {
        case 0:
            gyro_scale_factor_ = 2000.0f;
            break;
        case 1:
            gyro_scale_factor_ = 1000.0f;
            break;
        case 2:
            gyro_scale_factor_ = 500.0f;
            break;
        case 3:
            gyro_scale_factor_ = 250.0f;
            break;
        case 4:
            gyro_scale_factor_ = 125.0f;
            break;
        default:
            gyro_scale_factor_ = 2000.0f;
            break;;
    }
}

void IMU::ReadSensor()
{
    int16_t raw_accel_data[3];
    Bmi088AccelReadReg(BMI088_ACC_X_LSB_REG, reinterpret_cast<uint8_t*>(raw_accel_data), 6);
    raw_data_.accel[0] = static_cast<float>(raw_accel_data[0]) / 32768.0f * accel_scale_factor_;
    raw_data_.accel[1] = static_cast<float>(raw_accel_data[1]) / 32768.0f * accel_scale_factor_;
    raw_data_.accel[2] = static_cast<float>(raw_accel_data[2]) / 32768.0f * accel_scale_factor_;

    int16_t raw_gyro_data[3];
    Bmi088GyroReadReg(BMI088_GYRO_X_LSB_REG, reinterpret_cast<uint8_t*>(raw_gyro_data), 6);
    raw_data_.gyro[0] = static_cast<float>(raw_gyro_data[0]) / 32768.0f * gyro_scale_factor_;
    raw_data_.gyro[1] = static_cast<float>(raw_gyro_data[1]) / 32768.0f * gyro_scale_factor_;
    raw_data_.gyro[2] = static_cast<float>(raw_gyro_data[2]) / 32768.0f * gyro_scale_factor_;
}

void IMU::UpdateAttitude()
{
    float temp_gyro_dps[3];
    float temp_accel_g[3];
    float temp_accel_m_s2[3];

    for (int i = 0; i < 3; i++)
    {
        temp_gyro_dps[i] = raw_data_.gyro[i] - gyro_bias_[i];
        temp_accel_g[i] = raw_data_.accel[i];
    }

    Mahony::Matrix33fMultVector3f(R_imu_, temp_gyro_dps, gyro_sensor_dps_);

    for (int i = 0; i < 3; i++)
    {
        gyro_sensor_[i] = gyro_sensor_dps_[i] * M_PI / 180.0f;
    }

    for (int i = 0; i < 3; i++)
    {
        temp_accel_m_s2[i] = temp_accel_g[i] * gravity_accel;
    }

    Mahony::Matrix33fMultVector3f(R_imu_, temp_accel_m_s2, accel_sensor_);

    // 检查加速度计的模长是否接近于 0
    float accel_norm = sqrtf(accel_sensor_[0] * accel_sensor_[0] +
                             accel_sensor_[1] * accel_sensor_[1] +
                             accel_sensor_[2] * accel_sensor_[2]);

    // 如果模长为 0 (或非常小)，则不更新姿态，以防止除零 (NaN)
    if (accel_norm < 0.001f)
    {
        // (可选) 在这里您可以设置一个错误标志
        // 保持上一次的姿态不变，仅返回
        return;
    }

    mahony_.Update(q_, gyro_sensor_, accel_sensor_);

    float sin_pitch = 2.0f * (q_[0] * q_[2] - q_[1] * q_[3]);

    if (std::isnan(sin_pitch) || std::isinf(sin_pitch))
    {
        // 如果 q_ 已经损坏，立即停止，防止 NaN 传播
        // 这也暗示 mahony.cpp 内部的归一化需要修复
        return; // 保持上一帧的姿态
    }

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    euler_rad_.pitch = asinf(sin_pitch);

    if (fabsf(sin_pitch) >= 0.9999f)
    {
        euler_rad_.roll = 0.0f;
        euler_rad_.yaw = atan2f(-2.0f * (q_[1] * q_[3] - q_[0] * q_[2]), 1.0f - 2.0f * (q_[1] * q_[1] + q_[3] * q_[3]));
    }
    else
    {
        euler_rad_.roll = atan2f(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]), 1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2]));
        euler_rad_.yaw = atan2f(2.0f * (q_[0] * q_[3] + q_[1] * q_[2]), 1.0f - 2.0f * (q_[2] * q_[2] + q_[3] * q_[3]));
    }

    euler_deg_.roll = euler_rad_.roll * (180.0f / M_PI);
    euler_deg_.pitch = euler_rad_.pitch * (180.0f / M_PI);
    euler_deg_.yaw = euler_rad_.yaw * (180.0f / M_PI);

    memcpy(gyro_world_, mahony_.ww_, 3 * sizeof(float));
    memcpy(accel_world_, mahony_.aw_, 3 * sizeof(float));

    for (int i = 0; i < 3; i++)
    {
        gyro_world_dps_[i] = gyro_world_[i] * (180.0f / M_PI);
    }
}

EulerAngle_t IMU::GetAttitude() const
{
    return euler_deg_;
}