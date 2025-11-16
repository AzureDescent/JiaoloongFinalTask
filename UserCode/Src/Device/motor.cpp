//
// Created by DrownFish on 2025/11/4.
//
#include "motor.h"

Motor::Motor() :
    ratio_(1.0f), // 默认比例
    angle_(0.0f), delta_angle_(0.0f),
    ecd_angle_(0.0f), last_ecd_angle_(0.0f), delta_ecd_angle_(0.0f),
    rotate_speed_(0.0f), current_(0.0f), temp_(0.0f)
{}

// 构造函数
Motor::Motor(float ratio) :
    ratio_(ratio),
    angle_(0.0f), delta_angle_(0.0f),
    ecd_angle_(0.0f), last_ecd_angle_(0.0f), delta_ecd_angle_(0.0f),
    rotate_speed_(0.0f), current_(0.0f), temp_(0.0f)
{}

void Motor::Decode(const uint8_t* data)
{
    const int16_t raw_ecd_angle = (data[0] << 8 | data[1]);
    ecd_angle_ = LinearMapping(raw_ecd_angle, 0, 8191, 0.0f, 360.0f); //
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    last_ecd_angle_ = ecd_angle_;

    if (delta_ecd_angle_ > 180.0f)
    {
        delta_ecd_angle_ -= 360.0f;
    }
    else if (delta_ecd_angle_ < -180.0f)
    {
        delta_ecd_angle_ += 360.0f;
    }

    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;

    int16_t raw_rotate_speed = (data[2] << 8 | data[3]);
    rotate_speed_ = static_cast<float>(raw_rotate_speed);

    int16_t raw_current = (data[4] << 8 | data[5]);

    current_ = LinearMapping(raw_current, -16384, 16384, -3.0f, 3.0f);

    temp_ = static_cast<float>(data[6]);
}


float Motor::LinearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    if (in_max == in_min) {
        return out_min;
    }
    const float output_mapped = out_min + (out_max - out_min) * (in - in_min) / (float)(in_max - in_min);
    return output_mapped;
}