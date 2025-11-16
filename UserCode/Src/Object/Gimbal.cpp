//
// Created by DrownFish on 2025/11/4.
//
#include "Gimbal.h"
#include <cmath>
#include "rc.h" // (新增) 需要包含 rc.h 以使用 SwitchState


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
    (void)current_angle; // 避免未使用参数的警告
    return 0.0f; // (修改) 暂时返回 0.0f 以便编译
}


Gimbal::Gimbal():
    pitch_motor_(M6020_RATIO),
    yaw_motor_(M6020_RATIO)
{}

void Gimbal::Init()
{
    //TODO: Verify the i_max, out_max

    pitch_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    pitch_angle_pid_ = PID(0.f, 0.0f, 0.f, 0.f, 0.f, 0.f);

    yaw_speed_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    yaw_angle_pid_ = PID(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);

    current_mode_ = GIMBAL_MODE_OFF;
    imu_feedback_ = EulerAngle_t(0, 0, 0); // 初始化
}

void Gimbal::PitchMotorCallback(const uint8_t* data)
{
    pitch_motor_.Decode(data);
}

void Gimbal::YawMotorCallback(const uint8_t* data)
{
    yaw_motor_.Decode(data);
}

// (新增) 实现 rtos.cpp 调用的函数
void Gimbal::SetMode(Gimbal::Mode mode)
{
    if (current_mode_ == mode) return;

    current_mode_ = mode;
    if (mode == GIMBAL_MODE_OFF)
    {
        // 如果关闭模式，重置PID和输出
        pitch_angle_pid_.Reset();
        pitch_speed_pid_.Reset();
        yaw_angle_pid_.Reset();
        yaw_speed_pid_.Reset();
        pitch_output_torque_ = 0.0f;
        yaw_output_torque_ = 0.0f;
    }
}

// (新增) 实现 rtos.cpp 调用的函数
Gimbal::Mode Gimbal::DetermineMode(uint8_t switch_state)
{
    // 假设 switch_state 是 RemoteControl::SwitchState
    auto state = static_cast<RemoteControl::SwitchState>(switch_state);

    switch (state)
    {
        case RemoteControl::SWITCH_UP:
            return GIMBAL_MODE_ABSOLUTE; // 假设上是绝对角度（RC控制）
        case RemoteControl::SWITCH_MID:
            return GIMBAL_MODE_NORMAL;   // 假设中是正常（跟随IMU）
        case RemoteControl::SWITCH_DOWN:
        default:
            return GIMBAL_MODE_OFF;      // 假设下是关闭
    }
}

// (新增) 实现 rtos.cpp 调用的函数
void Gimbal::UpdateMotorFeedback(uint32_t std_id, const uint8_t* data)
{
    // 注意：这里假设了电机的 CAN ID
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

// (新增) 实现 rtos.cpp 调用的函数
void Gimbal::SetPIDTargets(float yaw_stick, float pitch_stick)
{
    // rtos.cpp (VControlTask) 周期为 10ms
    const float dt = 0.010f;

    if (current_mode_ == GIMBAL_MODE_ABSOLUTE)
    {
        // 绝对角度模式（遥控器控制），使用增量（速度）控制
        float yaw_speed = 0.0f;
        if (std::abs(yaw_stick) > RC_STICK_DEADZONE)
        {
            // 假设 yaw_stick 是 -1.0 到 1.0 (需要根据 rc.cpp 确认)
            yaw_speed = yaw_stick * RC_YAW_SPEED_SCALE;
        }
        // 增量式更新目标角度 (TODO: 确认 yaw 轴是相对角度还是绝对角度)
        target_yaw_angle_ += yaw_speed * dt;

        float pitch_speed = 0.0f;
        if (std::abs(pitch_stick) > RC_STICK_DEADZONE)
        {
            pitch_speed = pitch_stick * RC_PITCH_SPEED_SCALE;
        }
        target_pitch_angle_ += pitch_speed * dt;

        // TODO: 需要对 target_pitch_angle_ 进行限位
    }
    else if (current_mode_ == GIMBAL_MODE_NORMAL)
    {
        target_yaw_angle_ = imu_feedback_.yaw;
        target_pitch_angle_ = 0.0f;
    }
}


void Gimbal::Handle()
{
    if (current_mode_ == GIMBAL_MODE_OFF)
    {
        pitch_output_torque_ = 0.0f;
        yaw_output_torque_ = 0.0f;
        return;
    }

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
}

int16_t Gimbal::GetPitchCurrentToSend()
{
    return ConvertTorqueToCanCurrent(pitch_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}

int16_t Gimbal::GetYawCurrentToSend()
{
    return ConvertTorqueToCanCurrent(yaw_output_torque_, M6020_TORQUE_CONSTANT, M6020_MAX_CURRENT);
}