//
// Created by DrownFish on 2025/11/4.
//
#include "rc.h"

void RemoteControl::Init()
{
    // TODO: Check Initialization
    is_connected = false;

    lastTick = 0;
    curTick = 0;

    ch0 = 0, ch1 = 0, ch2 = 0, ch3 = 0;
    s1 = 0, s2 = 0;
    Right_X = 0.0f, Left_X = 0.0f, Right_Y = 0.0f, Left_Y = 0.0f;

    switch_left = MID;
    switch_right = MID;
}

float RemoteControl::LinearMapping(int in_min, int in_max, int input, float out_min, float out_max)
{
    float output_mapped = out_min + (out_max - out_min) * (input - in_min) / (in_max - in_min);
    return output_mapped;
}

void RemoteControl::Handle(const uint8_t* Data)
{
    DataProcess(Data);
}

void RemoteControl::DataProcess(const uint8_t* pData)
{
    if (pData == nullptr)
    {
        return;
    }
    ch0 = (static_cast<int16_t>(pData[0]) | (static_cast<int16_t>(pData[1]) << 8)) & 0x07FF;
    ch1 = ((static_cast<int16_t>(pData[1]) >> 3) | (static_cast<int16_t>(pData[2]) << 5)) & 0x07FF;
    ch2 = ((static_cast<int16_t>(pData[2]) >> 6) | (static_cast<int16_t>(pData[3]) << 2) | (static_cast<int16_t>(pData[
        4]) << 10)) & 0x07FF;
    ch3 = ((static_cast<int16_t>(pData[4]) >> 1) | (static_cast<int16_t>(pData[5]) << 7)) & 0x07FF;
    s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    s2 = ((pData[5] >> 4) & 0x0003);

    Right_X = LinearMapping(364, 1684, ch0, -1.0f, 1.0f);
    Right_Y = LinearMapping(364, 1684, ch1, -1.0f, 1.0f);
    Left_X = LinearMapping(364, 1684, ch2, -1.0f, 1.0f);
    Left_Y = LinearMapping(364, 1684, ch3, -1.0f, 1.0f);

    switch (s1)
    {
        case 1:
            switch_left = UP;
            break;
        case 2:
            switch_left = DOWN;
            break;
        case 3:
            switch_left = MID;
            break;
        default:
            switch_left = MID;
    }

    switch (s2)
    {
        case 1:
            switch_right = UP;
            break;
        case 2:
            switch_right = DOWN;
            break;
        case 3:
            switch_right = MID;
            break;
        default:
            switch_right = MID;
    }
}

extern "C" void RcInitWrapper(void)
{
    rc_controller.Init();
}