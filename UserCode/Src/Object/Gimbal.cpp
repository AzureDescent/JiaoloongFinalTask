//
// Created by DrownFish on 2025/11/4.
//
#include "Gimbal.h"
#include <cmath>


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
    static float torque_tmp = CalcTorque(current_angle);
    return torque_tmp;
}


Gimbal::Gimbal():
    pitch_motor_(M6020_RATIO),
    // M6020 减速比 36:1
    yaw_motor_(M6020_RATIO)
{}

void Gimbal::Init()
{
    //TODO: Verify the i_max, out_max

    pitch_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    pitch_angle_pid_ = PID(0.f, 0.0f, 0.f, 0.f, 0.f, 0.f);

    yaw_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    yaw_angle_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

void Gimbal::PitchMotorCallback(const uint8_t* data)
{
    pitch_motor_.Decode(data);
}

void Gimbal::YawMotorCallback(const uint8_t* data)
{
    yaw_motor_.Decode(data);
}

void Gimbal::Handle()
{
    float fdb_angle = pitch_motor_.GetAngle();
    float fdb_speed = pitch_motor_.GetSpeed();

    float target_speed = pitch_angle_pid_.Calc(target_pitch_angle_, fdb_angle);

    float output_torque = pitch_speed_pid_.Calc(target_speed, fdb_speed);

    float feedforward_torque = CalculateFeedforward(fdb_angle);

    pitch_output_torque_ = output_torque + feedforward_torque;


    fdb_angle = yaw_motor_.GetAngle();
    fdb_speed = yaw_motor_.GetSpeed();

    target_speed = yaw_angle_pid_.Calc(target_yaw_angle_, fdb_angle);

    output_torque = yaw_speed_pid_.Calc(target_speed, fdb_speed);

    feedforward_torque = CalculateFeedforward(fdb_angle);

    yaw_output_torque_ = output_torque + feedforward_torque;
}

int16_t Gimbal::GetPitchCurrentToSend()
{
    return ConvertTorqueToCanCurrent(pitch_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}

int16_t Gimbal::GetYawCurrentToSend()
{
    return ConvertTorqueToCanCurrent(yaw_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}