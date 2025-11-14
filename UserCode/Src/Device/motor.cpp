//
// Created by DrownFish on 2025/11/4.
//
#include "motor.h"

Motor::Motor()
{
    feedback = {0, 0, 0, 0};
}

// TODO: Transplant and Check the Variables with their Definitions and Functions
void Motor::Decode(const uint8_t* data)
{

}

Motor::FeedbackData Motor::GetFeedback() const
{
    return feedback;
}

float Motor::GetSpeed() const
{
    return static_cast<float>(feedback.speed);
}

float Motor::GetAngle() const
{
    return static_cast<float>(feedback.angle);
}