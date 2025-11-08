//
// Created by DrownFish on 2025/11/8.
//
#include "mahony.h"
#include "string.h"
#include "cmath"

// Mahony algorithm, sensor data fusion, update quaternion
// Mahony算法，传感器数据融合，四元数更新
void Mahony::Update(float q[4], float ws[3], float as[3])
{
    // Update sensor data
    memcpy(q_, q, 4 * sizeof(float));
    memcpy(ws_, ws, 3 * sizeof(float));
    memcpy(as_, as, 3 * sizeof(float));

    // Calculate rotate matrix
    R_[0][0] = 1 - 2.f * (q_[2] * q_[2] + q_[3] * q_[3]);
    R_[0][1] = 2.f * (q_[1] * q_[2] - q_[0] * q_[3]);
    R_[0][2] = 2.f * (q_[1] * q_[3] + q_[0] * q_[2]);
    R_[1][0] = 2.f * (q_[1] * q_[2] + q_[0] * q_[3]);
    R_[1][1] = 1 - 2.f * (q_[1] * q_[1] + q_[3] * q_[3]);
    R_[1][2] = 2.f * (q_[2] * q_[3] - q_[0] * q_[1]);
    R_[2][0] = 2.f * (q_[1] * q_[3] - q_[0] * q_[2]);
    R_[2][1] = 2.f * (q_[2] * q_[3] + q_[0] * q_[1]);
    R_[2][2] = 1 - 2.f * (q_[1] * q_[1] + q_[2] * q_[2]);
    Matrix33fTrans(R_, RT_);

    // Calculate acceleration(g) field reference &
    // centripetal acceleration reference
    Matrix33fMultVector3f(RT_, _gw_, _gs_); // -gs=R^T*(-gw)

    // calculate error between measure & reference of acceleration
    Vector3fCross(as_, _gs_, ea_);

    // wu(update)=ws+kg*eg
    if (fabs(Vector3fNorm(as_) - gravity_accel) < g_threshold_)
    {
        for (int i = 0; i < 3; i++)
        {
            we_[i] = kg_ * ea_[i];
        }
    }

    Vector3fAdd(ws_, we_, wu_);

    // ww=R*ws
    Matrix33fMultVector3f(R_, ws_, ww_);
    // ap_w=R*as+g (as~-gs)
    Matrix33fMultVector3f(R_, as_, aw_);
    Vector3fSub(aw_, _gw_, aw_);

    // update quaternion
    dq_[0] = 0.5f * (-q_[1] * wu_[0] - q_[2] * wu_[1] - q_[3] * wu_[2]) * dt_;
    dq_[1] = 0.5f * (q_[0] * wu_[0] - q_[3] * wu_[1] + q_[2] * wu_[2]) * dt_;
    dq_[2] = 0.5f * (q_[3] * wu_[0] + q_[0] * wu_[1] - q_[1] * wu_[2]) * dt_;
    dq_[3] = 0.5f * (-q_[2] * wu_[0] + q_[1] * wu_[1] + q_[0] * wu_[2]) * dt_;
    for (int i = 0; i < 4; i++)
    {
        q_[i] += dq_[i];
    }
    Vector4fUnit(q_, q_);
    memcpy(q, q_, 4 * sizeof(float));
}

// 矩阵转置：将3x3矩阵R转置存入RT
void Mahony::Matrix33fTrans(float R[3][3], float RT[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            RT[j][i] = R[i][j];
        }
    }
}

// 3x3矩阵与3维向量相乘：result = matrix × vector
void Mahony::Matrix33fMultVector3f(float matrix[3][3], float vector[3], float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        result[i] = 0;
        for (int j = 0; j < 3; j++)
        {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

void Mahony::Vector3fCross(float a[3], float b[3], float result[3])
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

// 3维向量模长（范数）
float Mahony::Vector3fNorm(float vector[3])
{
    return std::sqrt(vector[0] * vector[0] +
        vector[1] * vector[1] +
        vector[2] * vector[2]);
}

// 3维向量加法：result = a + b
void Mahony::Vector3fAdd(float a[3], float b[3], float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        result[i] = a[i] + b[i];
    }
}

// 3维向量减法：result = a - b
void Mahony::Vector3fSub(float a[3], float b[3], float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        result[i] = a[i] - b[i];
    }
}

// 4维向量单位化（归一化）
void Mahony::Vector4fUnit(float q[4], float result[4])
{
    float norm = std::sqrt(q[0] * q[0] +
        q[1] * q[1] +
        q[2] * q[2] +
        q[3] * q[3]);

    if (norm > 0.0001f)
    {
        // 避免除以零
        for (int i = 0; i < 4; i++)
        {
            result[i] = q[i] / norm;
        }
    }
    else
    {
        // 如果模长太小，设置为单位四元数
        result[0] = 1.0f;
        result[1] = 0.0f;
        result[2] = 0.0f;
        result[3] = 0.0f;
    }
}