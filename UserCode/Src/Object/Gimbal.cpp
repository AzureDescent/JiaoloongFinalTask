//
// Created by DrownFish on 2025/11/4.
//
#include "Gimbal.h"
#include <cmath>
#include "rc.h"


int16_t ConvertTorqueToCanCurrent(float torque, float torque_constant, float max_current)
{
    float current = torque / torque_constant;

    if (current > max_current)
    {
        current = max_current;
    }
    else if (current < -max_current)
    {
        current = -max_current;
    }

    return static_cast<int16_t>(current * 16384.0f / 2.0f);
}

float CalculateFeedforward(float current_angle)
{
    //TODO: 拟合前馈力矩
    // static float torque_tmp = CalcTorque(current_angle); // CalcTorque 未定义
    // return torque_tmp;
    (void)current_angle;
    return 0.0f;
}


Gimbal::Gimbal():
    pitch_motor_(1.0f),
    yaw_motor_(1.0f)
{}

void Gimbal::Init()
{
    //TODO: Verify the i_max, out_max

    pitch_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    pitch_angle_pid_ = PID(0.f, 0.0f, 0.f, 0.f, 0.f, 0.f);

    yaw_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    yaw_angle_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    current_mode_ = GIMBAL_MODE_OFF;
    imu_feedback_ = EulerAngle_t(0, 0, 0);
}

void Gimbal::PitchMotorCallback(const uint8_t* data)
{
    pitch_motor_.Decode(data);
}

void Gimbal::YawMotorCallback(const uint8_t* data)
{
    yaw_motor_.Decode(data);
}

void Gimbal::SetMode(Gimbal::Mode mode)
{
    if (current_mode_ == mode) return;

    current_mode_ = mode;

    pitch_angle_pid_.Reset();
    pitch_speed_pid_.Reset();
    yaw_angle_pid_.Reset();
    yaw_speed_pid_.Reset();

    if (mode == GIMBAL_MODE_OFF)
    {
        pitch_output_torque_ = 0.0f;
        yaw_output_torque_ = 0.0f;
    }
}

Gimbal::Mode Gimbal::DetermineMode(uint8_t switch_state)
{
    auto state = static_cast<RemoteControl::SwitchStatus>(switch_state);

    switch (state)
    {
        case RemoteControl::UP:
            return GIMBAL_MODE_PID;
        case RemoteControl::MID:
            return GIMBAL_MODE_FEEDFORWARD_TEST;
        case RemoteControl::DOWN:
        default:
            return GIMBAL_MODE_OFF;
    }
}

void Gimbal::UpdateMotorFeedback(uint32_t std_id, const uint8_t* data)
{
    if (std_id == PITCH_MOTOR_ID)
    {
        PitchMotorCallback(data);
    }
    else if (std_id == YAW_MOTOR_ID)
    {
        YawMotorCallback(data);
    }
}

// (新增) 实现 rtos.cpp 调用的函数
void Gimbal::SetImuFeedback(const EulerAngle_t& imu_attitude)
{
    imu_feedback_ = imu_attitude;
}

void Gimbal::SetPIDTargets(float yaw_stick, float pitch_stick)
{
    const float dt = 0.010f;

    if (current_mode_ == GIMBAL_MODE_PID || current_mode_ == GIMBAL_MODE_FEEDFORWARD_TEST)
    {
        float yaw_speed = 0.0f;
        if (std::abs(yaw_stick) > RC_STICK_DEADZONE)
        {
            yaw_speed = yaw_stick * RC_YAW_SPEED_SCALE;
        }
        target_yaw_angle_ += yaw_speed * dt;

        float pitch_speed = 0.0f;
        if (std::abs(pitch_stick) > RC_STICK_DEADZONE)
        {
            pitch_speed = pitch_stick * RC_PITCH_SPEED_SCALE;
        }
        target_pitch_angle_ += pitch_speed * dt;
    }
}


void Gimbal::Handle()
{
    switch (current_mode_)
    {
        case GIMBAL_MODE_OFF:
        {
            pitch_output_torque_ = 0.0f;
            yaw_output_torque_ = 0.0f;
            break;
        }

        case GIMBAL_MODE_PID:
        {
            float fdb_angle_pitch = imu_feedback_.pitch;
            float fdb_speed_pitch = pitch_motor_.GetSpeed();

            float target_speed_pitch = pitch_angle_pid_.Calc(target_pitch_angle_, fdb_angle_pitch);
            float output_torque_pitch = pitch_speed_pid_.Calc(target_speed_pitch, fdb_speed_pitch);
            float feedforward_torque_pitch = CalculateFeedforward(fdb_angle_pitch);

            pitch_output_torque_ = output_torque_pitch + feedforward_torque_pitch;

            float fdb_angle_yaw = yaw_motor_.GetAngle();
            float fdb_speed_yaw = yaw_motor_.GetSpeed();

            float target_speed_yaw = yaw_angle_pid_.Calc(target_yaw_angle_, fdb_angle_yaw);
            float output_torque_yaw = yaw_speed_pid_.Calc(target_speed_yaw, fdb_speed_yaw);

            yaw_output_torque_ = output_torque_yaw;
            break;
        }

        case GIMBAL_MODE_FEEDFORWARD_TEST:
        {
            float fdb_angle_pitch = imu_feedback_.pitch;
            pitch_output_torque_ = CalculateFeedforward(fdb_angle_pitch);

            yaw_output_torque_ = 0.0f;
            break;
        }
    }
}

int16_t Gimbal::GetPitchCurrentToSend()
{
    return ConvertTorqueToCanCurrent(pitch_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}

int16_t Gimbal::GetYawCurrentToSend()
{
    return ConvertTorqueToCanCurrent(yaw_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}