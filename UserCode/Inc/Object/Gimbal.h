//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_GIMBAL_H
#define FINALTASK_GIMBAL_H

#include "imu.h"

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
    // TODO: Verify the parameter types for SetPIDTargets
    void SetPIDTargets(/*    */);
    // TODO: Verify the parameter types for SetImuFeedback
    void SetImuFeedback(EulerAngle_t imu_attitude);
    // TODO: Verify the parameter type for DetermineMode
    Mode DetermineMode(/*    */);

    void RunControlLoop();
    // TODO: Verify the parameter types for UpdateMotorFeedback
    void UpdateMotorFeedback(uint16_t motor_id, uint8_t* data);
    int16_t GetYawMotorCurrent();
    int16_t GetPitchMotorCurrent();

private:
    Mode current_mode;
};

#endif //FINALTASK_GIMBAL_H