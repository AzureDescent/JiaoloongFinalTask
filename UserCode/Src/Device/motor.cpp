//
// Created by DrownFish on 2025/11/4.
//
#include "motor.h"

Motor::Motor()
{

}

// TODO: Transplant and Check the Variables with their Definitions and Functions
void Motor::Decode(uint8_t* data)
{

}

float LinearMapping(int in, int in_min, int in_max, float out_min, float out_max)
{
    const float output_mapped = out_min + (out_max - out_min) * (in - in_min) / (in_max - in_min);
    return output_mapped;
}