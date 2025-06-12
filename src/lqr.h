#ifndef LQR_H
#define LQR_H

#include "Arduino.h"
#include "robot.h"

// LQR控制器状态维度和输入维度
#define STATE_DIM 4    // 状态向量维度：[角度, 角速度, 位置, 线速度]
#define INPUT_DIM 1    // 输入向量维度：[电机控制信号]

// LQR控制器参数结构体
struct LQRParams {
    float Q[STATE_DIM][STATE_DIM];  // 状态权重矩阵Q
    float R[INPUT_DIM][INPUT_DIM];  // 输入权重矩阵R
    float K[INPUT_DIM][STATE_DIM];  // 反馈增益矩阵K
    bool initialized;               // 初始化标志
};

// LQR状态变量结构体
struct LQRState {
    float angle;        // 机器人倾斜角度 (pitch)
    float angular_vel;  // 机器人角速度 (gyroY)
    float position;     // 机器人位置（通过轮子速度积分）
    float velocity;     // 机器人线速度（轮子平均速度）
};

// 全局变量声明
extern LQRParams lqr_params;
extern LQRState lqr_state;
extern float wheel_motor1_target_lqr, wheel_motor2_target_lqr; // LQR电机目标值
extern float position_integral;  // 位置积分值
extern float dt;                 // 采样时间间隔
extern int speed_limit_lqr;      // LQR轮毂电机速度限制

// 函数声明
void lqr_init();                           // 初始化LQR控制器
void lqr_update_state();                   // 更新状态变量
void lqr_wheel_control();                  // LQR轮子控制主函数
float lqr_compute_control_signal();        // 计算LQR控制信号
void lqr_matrix_multiply(float result[], const float A[][STATE_DIM], const float B[], int rows, int cols);
float lqr_clamp_to_range(float value, float minVal, float maxVal);  // 限幅函数
void lqr_set_reference(float ref_angle, float ref_position);        // 设置参考值
void lqr_tune_parameters(float q_angle, float q_angular_vel, float q_position, float q_velocity, float r_input);

#endif 