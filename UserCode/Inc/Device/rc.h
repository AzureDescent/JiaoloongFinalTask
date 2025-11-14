//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_RC_H
#define FINALTASK_RC_H

#include <main.h>
#include "cmsis_os2.h"
#include "cmath"

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
    // 用于主控制线程获取数据的结构体
    struct ControlData
    {
        float yaw_stick;
        float pitch_stick;
        SwitchStatus switch_right;
        SwitchStatus switch_left;
    };

    bool is_connected;

    uint32_t lastTick;
    uint32_t curTick;

    void Init();
    void Handle(const uint8_t* Data);

    float LinearMapping(int in_min, int in_max, int input, float out_min, float out_max);
    void DataProcess(const uint8_t* pData);
    bool IsOffline();

    // 新增：获取处理后数据的 getter 函数
    ControlData get_control_data() const;

    // 原始数据
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t s1;
    uint8_t s2;

    // 映射数据
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    // 映射后的拨杆状态
    SwitchStatus switch_left;
    SwitchStatus switch_right;

private:
    ControlData control_data;

    const float RC_DEAD_ZONE = 0.05f;
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