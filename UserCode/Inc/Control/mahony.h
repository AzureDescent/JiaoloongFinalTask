//
// Created by DrownFish on 2025/11/8.
//

#ifndef FINALTASK_MAHONY_H
#define FINALTASK_MAHONY_H

constexpr float gravity_accel = 9.794f;

class Mahony
{
public:
    // Initialize & config parameter(time step & fusion coefficient)
    // 初始化，配置参数（时间步长，数据融合系数）
    Mahony(float dt, float kg, float g_threshold):
        dt_(dt),
        kg_(kg),
        g_threshold_(g_threshold) {}

    // Mahony algorithm, sensor data fusion, update quaternion
    // Mahony算法，传感器数据融合，四元数更新
    void Update(float q[4], float ws[3], float as[3]);

    static void Matrix33fTrans(float R[3][3], float RT[3][3]);
    static void Matrix33fMultVector3f(float matrix[3][3], float vector[3], float result[3]);
    static void Vector3fCross(float a[3], float b[3], float result[3]);
    static float Vector3fNorm(float vector[3]);
    static void Vector3fAdd(float a[3], float b[3], float result[3]);
    static void Vector3fSub(float a[3], float b[3], float result[3]);
    static void Vector4fUnit(float q[4], float result[4]);

public:
    float q_[4]; // quaternion
    float ws_[3], as_[3]; // gyro/acceleration(sensor)
    float R_[3][3]; // rotate matrix
    float ww_[3], aw_[3]; // gyro/acceleration(no g)(world)

private:
    float dq_[4]; // quaternion increment
    float RT_[3][3]; // rotate matrix transpose
    float _gs_[3]; // minus gravity acceleration (sensor)
    float ea_[3]; // error vector, ea=cross(-a,g)
    float wu_[3], we_[3]; // wu(update)=ws+kg*ea

    float dt_; // update time step;
    float kg_, km_; // data fusion coefficient

    float _gw_[] = { 0, 0, gravity_accel }; // -g(world)
    const float g_threshold_; // threshold of g offset
};

#endif //FINALTASK_MAHONY_H