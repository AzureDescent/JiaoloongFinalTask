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
    // TODO: Transplant and Check the Variables with their Definitions and Functions
    struct FeedbackData
    {
        int16_t angle;
        int16_t speed;
        int16_t current;
        uint8_t temperature;
    };

    Motor();

    FeedbackData GetFeedback() const;

    void Decode(const uint8_t* data);

    float GetSpeed() const;
    float GetAngle() const;

private:
    FeedbackData feedback;
};
#endif


#endif //FINALTASK_MOTOR_H