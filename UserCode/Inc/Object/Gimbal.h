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
        GIMBAL_MODE_CUSTOM,
        GIMBAL_MODE_START,
    };

    Gimbal();

    void SetMode(Mode mode);

    void SetPIDTargets(float yaw_target_stick, float pitch_target_stick);

    void SetImuFeedback(EulerAngle_t imu_attitude);

    Mode DetermineMode(SwitchStatus rc_switch);

    void RunControlLoop();

    void UpdateMotorFeedback(uint16_t motor_id, uint8_t* data);
    int16_t GetYawMotorCurrent();
    int16_t GetPitchMotorCurrent();

private:
    Mode current_mode;

    Motor yaw_motor;
    Motor pitch_motor;

    PID yaw_pos_pid;
    PID yaw_speed_pid;
    PID pitch_pos_pid;
    PID pitch_speed_pid;

    int16_t yaw_current_out;
    int16_t pitch_current_out;

    const uint16_t YAW_MOTOR_ID = 0x201;
    const uint16_t PITCH_MOTOR_ID = 0x202;
};
#endif

#endif //FINALTASK_GIMBAL_H