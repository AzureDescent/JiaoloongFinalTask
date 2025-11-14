//
// Created by DrownFish on 2025/11/4.
//
#include <pid.h>

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    d_filter_k_(d_filter_k),
    i_max_(i_max),
    out_max_(out_max),
    output_(0.0f),
    err_sum_(0.0),
    ref_(0.0f),
    fdb_(0.0f),
    err_(0.0f),
    last_err_(0.0f),
    pout_(0.0f),
    iout_(0.0f),
    dout_(0.0f),
    last_dout_(0.0f) {}

void PID::Reset()
{
    ref_ = 0.0f;
    fdb_ = 0.0f;
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
    output_ = 0.0f;
}

float PID::Calc(float ref, float fdb)
{
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;

    pout_ = kp_ * err_;

    err_sum_ += err_;
    if (err_sum_ > i_max_)
    {
        err_sum_ = i_max_;
    }
    else if (err_sum_ < -i_max_)
    {
        err_sum_ = -i_max_;
    }
    iout_ = ki_ * err_sum_;

    dout_ = kd_ * (err_ - last_err_);

    dout_ = d_filter_k_ * dout_ + (1 - d_filter_k_) * last_dout_;

    last_err_ = err_;
    last_dout_ = dout_;

    output_ = pout_ + iout_ + dout_;
    if (output_ > out_max_)
    {
        output_ = out_max_;
    }
    else if (output_ < -out_max_)
    {
        output_ = -out_max_;
    }
    return output_;
}

// TODO: Implement the Clear function to reset the integral sum
void PID::Clear()
{
    err_sum_ = 0.0f;
}