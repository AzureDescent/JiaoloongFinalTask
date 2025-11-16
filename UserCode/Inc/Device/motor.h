//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_MOTOR_H
#define FINALTASK_MOTOR_H

#include "main.h"

#ifdef __cplusplus

class Motor
{
public:
    // TODO: Check the ratio
    explicit Motor(float ratio);
    Motor(); // 默认构造函数

    void Decode(const uint8_t* data);

    float GetAngle() const { return angle_; }

    float GetSpeed() const { return rotate_speed_; }

    float GetCurrent() const { return current_; }

    float GetTemp() const { return temp_; }

    float GetEcdAngle() const { return ecd_angle_; }

private:
    static float LinearMapping(int in, int in_min, int in_max, float out_min, float out_max);

    float ratio_ = 1.0f;

    float angle_ = 0.0f;
    float delta_angle_ = 0.0f;
    float ecd_angle_ = 0.0f;
    float last_ecd_angle_ = 0.0f;
    float delta_ecd_angle_ = 0.0f;

    float rotate_speed_ = 0.0f;
    float current_ = 0.0f;
    float temp_ = 0.0f;
};

#endif

#endif //FINALTASK_MOTOR_H