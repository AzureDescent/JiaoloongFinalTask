//
// Created by DrownFish on 2025/11/4.
//
#include "Gimbal.h"
#include <cmath>
#include "rc.h"


int16_t ConvertTorqueToCanCurrent(float torque, float torque_constant, float max_current)
{
    float current_val = torque;
    float limit = 20000.0f;

    if (current_val > limit)
    {
        current_val = limit;
    }
    else if (current_val < -limit)
    {
        current_val = -limit;
    }

    return static_cast<int16_t>(current_val);
}

float CalculateFeedforward(const float current_angle)
{
    const float angle_rad = current_angle * (3.1415926f / 180.0f);

    // 拟合公式：3500 * sin( 实际角度_rad - 29.2度对应的弧度 )
    const float a = 3500.0f; // 幅值
    const float b = 0.51f;    // 相位偏移 (平衡点弧度)

    // 返回所需的补偿电流值 (Raw Units)
    return a * sinf(angle_rad - b);
}


Gimbal::Gimbal():
    pitch_motor_(1.0f),
    yaw_motor_(1.0f) {}

void Gimbal::Init()
{
    //TODO: Verify the i_max, out_max

    pitch_angle_pid_ = PID(2.0f, 0.0f, 0.0f, 0.0f, 200.0f);
    pitch_speed_pid_ = PID(42.0f, 0.0f, 0.0f, 2000.0f, 20000.0f);


    yaw_angle_pid_ = PID(2.0f, 0.0f, 0.0f, 0.0f, 200.0f);
    yaw_speed_pid_ = PID(10.0f, 0.0f, 0.0f, 2000.0f, 20000.0f);

    target_pitch_angle_ = 0.0f;
    target_yaw_angle_ = 0.0f;

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
    if (current_mode_ == mode)
    {
        return;
    }

    if (current_mode_ == GIMBAL_MODE_OFF && (mode == GIMBAL_MODE_PID || mode == GIMBAL_MODE_FEEDFORWARD_TEST))
    {
        target_pitch_angle_ = imu_feedback_.pitch;
        target_yaw_angle_ = yaw_motor_.GetAngle();
    }
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

void Gimbal::SetImuFeedback(const EulerAngle_t& imu_attitude)
{
    imu_feedback_ = imu_attitude;
}

void Gimbal::SetPIDTargets(float yaw_stick, float pitch_stick)
{
    if (current_mode_ == GIMBAL_MODE_PID)
    {
        constexpr float dt = 0.010f;
        float yaw_speed = 0.0f;
        if (std::fabs(yaw_stick) > RC_STICK_DEADZONE)
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

        if (target_pitch_angle_ > PITCH_MAX_ANGLE)
        {
            target_pitch_angle_ = PITCH_MAX_ANGLE;
        }
        else if (target_pitch_angle_ < PITCH_MIN_ANGLE)
        {
            target_pitch_angle_ = PITCH_MIN_ANGLE;
        }
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
            float fdb_speed_pitch = pitch_motor_.GetSpeed();

            float target_speed_pitch = pitch_angle_pid_.Calc(target_pitch_angle_, fdb_angle_pitch);
            float output_torque_pitch = pitch_speed_pid_.Calc(target_speed_pitch, fdb_speed_pitch);
            float feedforward_torque_pitch = CalculateFeedforward(fdb_angle_pitch);
            pitch_output_torque_ = output_torque_pitch + feedforward_torque_pitch;

            yaw_output_torque_ = 0.0f;
            break;
        }
    }
}

void Gimbal::UpdateCurrentCommands()
{
    pitch_current_to_send_ = ConvertTorqueToCanCurrent(pitch_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
    yaw_current_to_send_ = ConvertTorqueToCanCurrent(yaw_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}

int16_t Gimbal::GetPitchCurrentToSend()
{
    return pitch_current_to_send_;
}

int16_t Gimbal::GetYawCurrentToSend()
{
    return yaw_current_to_send_;
}