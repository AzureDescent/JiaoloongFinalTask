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
    enum Mode
    {
        GIMBAL_MODE_OFF,
        GIMBAL_MODE_PID,
        GIMBAL_MODE_FEEDFORWARD_TEST
    };

    Gimbal();
    void Init();
    void Handle();
    void UpdateCurrentCommands();

    void SetMode(Mode mode);
    Mode DetermineMode(uint8_t switch_state);
    void UpdateMotorFeedback(uint32_t std_id, const uint8_t* data);
    void SetPIDTargets(float yaw_stick, float pitch_stick);
    void SetImuFeedback(const EulerAngle_t& imu_attitude);

    void PitchMotorCallback(const uint8_t* data);
    void YawMotorCallback(const uint8_t* data);

    int16_t GetPitchCurrentToSend();
    int16_t GetYawCurrentToSend();

    uint32_t GetPitchMotorID() const
    {
        return PITCH_MOTOR_ID;
    }

    uint32_t GetYawMotorID() const
    {
        return YAW_MOTOR_ID;
    }

// private:
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

    Mode current_mode_ = GIMBAL_MODE_OFF;
    EulerAngle_t imu_feedback_{};

    // TODO: 确认电机的 CAN ID
    const uint32_t PITCH_MOTOR_ID = 0x208;
    const uint32_t YAW_MOTOR_ID = 0x205;

    const float PITCH_MAX_ANGLE = 40.0f;
    const float PITCH_MIN_ANGLE = -40.0f;

    // TODO: 确认遥控器参数
    const float RC_STICK_DEADZONE = 0.05f;
    const float RC_YAW_SPEED_SCALE = 180.0f;
    const float RC_PITCH_SPEED_SCALE = 90.0f;

    const float M6020_RATIO = 1.0f;
    const float M6020_TORQUE_CONSTANT = 0.741f;
    const float M6020_MAX_CURRENT = 1.62f;

    volatile int16_t pitch_current_to_send_ = 0;
    volatile int16_t yaw_current_to_send_ = 0;
};
#endif

#endif //FINALTASK_GIMBAL_H