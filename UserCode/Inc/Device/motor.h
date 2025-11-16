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

    Motor();

    void Decode(uint8_t* data);

    float LinearMapping(int in, int in_min, int in_max, float out_min, float out_max);

private:
};
#endif


#endif //FINALTASK_MOTOR_H