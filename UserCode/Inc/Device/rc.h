//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RC_H
#define FINALTASK_RC_H

#include <main.h>
#include "rtos.h"

enum SwitchStatus
{
    UP,
    DOWN,
    MID
};

#ifdef __cplusplus
class RemoteControl
{
public:
    bool is_connected;

    uint32_t lastTick;
    uint32_t curTick;

    void Init();
    void Handle(const uint8_t* Data);

    float LinearMapping(int in_min, int in_max, int input, float out_min, float out_max);
    void DataProcess(const uint8_t* pData);
    bool IsOffline();

    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;

    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    // 注意：这里使用了上面定义的 SwitchStatus
    SwitchStatus switch_left;
    SwitchStatus switch_right;
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
    #endif

    void RcInitWrapper(void);

    #ifdef __cplusplus
}
#endif // __cplusplus

#endif //FINALTASK_RC_H