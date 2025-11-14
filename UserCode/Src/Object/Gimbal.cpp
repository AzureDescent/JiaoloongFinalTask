//
// Created by DrownFish on 2025/11/4.
//
#include "Gimbal.h"

Gimbal::Gimbal() : current_mode(GIMBAL_MODE_OFF), yaw_current_out(0), pitch_current_out(0)
{
    // TODO: 在 Task 3 中，初始化 PID 控制器参数
    // 参考以往文件中的 PID 初始化代码
}

/**
 * @brief 根据遥控器右拨杆状态决定云台模式
 * @param rc_switch
 * @return Gimbal::Mode
 */
Gimbal::Mode Gimbal::DetermineMode(SwitchStatus rc_switch)
{
    switch (rc_switch)
    {
        case UP:
            return GIMBAL_MODE_START;
        case DOWN:
            return GIMBAL_MODE_OFF;
        case MID:
        default:
            return GIMBAL_MODE_OFF;
    }
}

/**
 * @brief 设置云台的当前模式
 * @param mode
 */
void Gimbal::SetMode(Mode mode)
{
    current_mode = mode;

    // TODO: 在 Task 3 中，添加Gimbal逻辑
    // 如果模式从 OFF 切换到 START，重置 PID 积分项
    if (mode == GIMBAL_MODE_OFF)
    {
        yaw_pos_pid_.Clear();
        yaw_vel_pid_.Clear();
        pitch_pos_pid_.Clear();
        pitch_vel_pid_.Clear();

        yaw_current_out = 0;
        pitch_current_out = 0;
    }
}

/**
 * @brief 设置 PID 控制的目标值
 * @param yaw_target_stick   [-1.0f, 1.0f]
 * @param pitch_target_stick [-1.0f, 1.0f]
 */
void Gimbal::SetPIDTargets(float yaw_target_stick, float pitch_target_stick)
{
    if (current_mode == GIMBAL_MODE_OFF)
    {
        return;
    }

    // --- Task 3: PID 控制逻辑 START ---
    // 这是你实现 Task 3 (PID) 的入口点。
    // 你需要将摇杆的输入（[-1, 1]）转换为角度或速度目标值。

    // S：
    // float yaw_target_speed = yaw_target_stick * MAX_YAW_SPEED;
    // yaw_speed_pid.SetTarget(yaw_target_speed);

    // P：
    // float yaw_target_angle_increment = yaw_target_stick * YAW_SENSITIVITY * dt;
    // yaw_angle_pid.AddTarget(yaw_target_angle_increment);
}

/**
 * @brief 设置 IMU 反馈数据
 * @param imu_attitude
 */
void Gimbal::SetImuFeedback(EulerAngle_t imu_attitude)
{
    if (current_mode == GIMBAL_MODE_OFF)
    {
        return;
    }

    // TODO: 在 Task 3 中，将 IMU 数据喂给 PID 控制器
    // e.g. yaw_angle_pid.SetFeedback(imu_attitude.yaw);
    // e.g. pitch_angle_pid.SetFeedback(imu_attitude.pitch);
}

/**
 * @brief 主控制循环，在此处运行 PID 计算
 */
void Gimbal::RunControlLoop()
{
    if (current_mode == GIMBAL_MODE_OFF)
    {
        yaw_current_out = 0;
        pitch_current_out = 0;
        return;
    }

    // TODO: 在 Task 3 中，运行 PID 计算
    // 示例：
    // 1. 计算 Pitch (双环)
    // pitch_angle_pid.Calculate();
    // pitch_speed_pid.SetTarget(pitch_angle_pid.GetOutput());
    // pitch_speed_pid.SetFeedback(pitch_motor.GetSpeed()); // 假设电机有速度反馈
    // pitch_speed_pid.Calculate();
    //
    // 2. 计算 Yaw (类似)
    // ...
    //
    // 3. (可选) 重力前馈 [cite: 66]
    // float pitch_feedforward = CalculateGravityFeedforward(pitch_motor.GetAngle());
    //
    // 4. 设置最终电流
    // int16_t pitch_current = (int16_t)pitch_speed_pid.GetOutput() + (int16_t)pitch_feedforward;
    // SetPitchMotorCurrent(pitch_current);
}

/**
 * @brief CAN 回调函数会调用此函数来更新电机反馈
 * @param motor_id
 * @param data
 */
void Gimbal::UpdateMotorFeedback(uint16_t motor_id, uint8_t* data)
{
    if (motor_id == YAW_MOTOR_ID)
    {
        yaw_motor.Decode(data);
    }
    else if (motor_id == PITCH_MOTOR_ID)
    {
        pitch_motor.Decode(data);
    }
}

/**
 * @brief VCanSendTask 会调用此函数获取要发送的电流
 * @return
 */
int16_t Gimbal::GetYawMotorCurrent()
{
    if (current_mode == GIMBAL_MODE_OFF) return 0;

    // TODO: 从 PID 计算结果中返回 Yaw 电机电流
    // return (int16_t)yaw_speed_pid.GetOutput();
    return 0; // 桩实现
}

/**
 * @brief VCanSendTask 会调用此函数获取要发送的电流
 * @return
 */
int16_t Gimbal::GetPitchMotorCurrent()
{
    if (current_mode == GIMBAL_MODE_OFF) return 0;

    // TODO: 从 PID 计算结果中返回 Pitch 电机电流
    // return (int16_t)pitch_speed_pid.GetOutput();
    return 0; // 桩实现
}