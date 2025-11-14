//
// Created by DrownFish on 2025/11/4.
//

#ifndef FINALTASK_PID_H
#define FINALTASK_PID_H

#ifdef __cplusplus
class PID
{
public:
    PID(void): PID(0, 0, 0, 0, 0){}

    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k = 1);

    void Reset(void);
    float Calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;

    float err_sum_;

private:
    float ref_, fdb_;
    float err_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};
#endif

#endif //FINALTASK_PID_H