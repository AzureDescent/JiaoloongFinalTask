//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_GIMBAL_H
#define FINALTASK_GIMBAL_H

#include "imu.h"
#include "rc.h"
#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
class Gimbal
{
public:
    Gimbal();
    void Init();
    void Handle(); // 主控制循环

    // 外部设置目标值
    void SetTargetAngle(float pitch, float yaw);

    // CAN 消息回调
    void PitchMotorCallback(const uint8_t* data);
    void YawMotorCallback(const uint8_t* data);

    // 获取要发送的电流
    int16_t GetPitchCurrentToSend();
    int16_t GetYawCurrentToSend();

private:
    Motor pitch_motor_;
    Motor yaw_motor_;

    PID pitch_angle_pid_;
    PID pitch_speed_pid_;
    PID yaw_angle_pid_;
    PID yaw_speed_pid_;

    float target_pitch_angle_ = 0.0f;
    float target_yaw_angle_ = 0.0f;

    float pitch_output_torque_ = 0.0f;
    float yaw_output_torque_ = 0.0f;

    const float M6020_RATIO = 1.0f;
    const float M6020_TORQUE_CONSTANT = 0.741f;
    const float M6020_MAX_CURRENT = 1.62f;
};
#endif

#endif //FINALTASK_GIMBAL_H