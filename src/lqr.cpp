#include "lqr.h"

// 全局变量定义
LQRParams lqr_params;
LQRState lqr_state;
float wheel_motor1_target_lqr = 0, wheel_motor2_target_lqr = 0;
float position_integral = 0;
float dt = 0.01; // 10ms采样周期
int speed_limit_lqr = 5;
float reference_angle = 0;
float reference_position = 0;

// LQR控制器初始化
void lqr_init() {
    // 初始化状态权重矩阵Q (对角矩阵)
    // Q[0][0] - 角度权重，Q[1][1] - 角速度权重，Q[2][2] - 位置权重，Q[3][3] - 速度权重
    for(int i = 0; i < STATE_DIM; i++) {
        for(int j = 0; j < STATE_DIM; j++) {
            lqr_params.Q[i][j] = 0.0;
        }
    }
    lqr_params.Q[0][0] = 100.0;  // 角度权重 - 保持直立最重要
    lqr_params.Q[1][1] = 10.0;   // 角速度权重 - 抑制震荡
    lqr_params.Q[2][2] = 1.0;    // 位置权重 - 保持位置
    lqr_params.Q[3][3] = 1.0;    // 速度权重 - 速度稳定
    
    // 初始化输入权重矩阵R
    lqr_params.R[0][0] = 0.1;  // 控制输入权重 - 降低能耗
    
    // 预计算的LQR增益矩阵K (基于双轮自平衡机器人线性化模型)
    // 这些增益是通过求解Riccati方程得到的近似值
    lqr_params.K[0][0] = -31.62;  // 角度反馈增益
    lqr_params.K[0][1] = -4.47;   // 角速度反馈增益  
    lqr_params.K[0][2] = -1.73;   // 位置反馈增益
    lqr_params.K[0][3] = -2.24;   // 速度反馈增益
    
    // 初始化状态变量
    lqr_state.angle = 0;
    lqr_state.angular_vel = 0;
    lqr_state.position = 0;
    lqr_state.velocity = 0;
    
    position_integral = 0;
    lqr_params.initialized = true;
    
    Serial.println("LQR控制器初始化完成");
}

// 更新系统状态变量
void lqr_update_state() {
    // 更新角度和角速度 (来自MPU6050)
    lqr_state.angle = pitch - balance_offset - remoteBalanceOffset;  // 减去偏置
    lqr_state.angular_vel = gyroY;
    
    // 更新位置和速度 (基于轮子编码器)
    lqr_state.velocity = (motor1_vel + motor2_vel) / 2.0;  // 平均轮速
    position_integral += lqr_state.velocity * dt;           // 位置积分
    lqr_state.position = position_integral - reference_position;
    
    // 添加前后移动指令影响
    lqr_state.velocity -= forwardBackward;  // 减去期望速度，得到速度误差
}

// 计算LQR控制信号
float lqr_compute_control_signal() {
    if (!lqr_params.initialized) {
        lqr_init();
    }
    
    // 计算状态误差 (当前状态 - 参考状态)
    float state_error[STATE_DIM];
    state_error[0] = lqr_state.angle - reference_angle;       // 角度误差
    state_error[1] = lqr_state.angular_vel;                   // 角速度误差
    state_error[2] = lqr_state.position;                      // 位置误差
    state_error[3] = lqr_state.velocity;                      // 速度误差
    
    // 计算控制信号 u = -K * x
    float control_signal = 0;
    for(int i = 0; i < STATE_DIM; i++) {
        control_signal -= lqr_params.K[0][i] * state_error[i];
    }
    
    return control_signal;
}

// LQR轮子控制主函数
void lqr_wheel_control() {
    // 更新状态变量
    lqr_update_state();
    
    // 计算基础LQR控制信号
    float base_control = lqr_compute_control_signal();
    
    // 根据高度调整控制增益
    float height_factor = 1.0;
    if (ZeparamremoteValue > 80) {
        height_factor = 1.15;  // 高机身时增强控制
    }
    
    // 应用高度因子
    base_control *= height_factor;
    
    // 计算每个轮子的控制信号
    wheel_motor1_target_lqr = base_control - 0.7 * steering;  // 左轮
    wheel_motor2_target_lqr = base_control + 0.7 * steering;  // 右轮
    
    // 应用速度限制
    wheel_motor1_target_lqr = lqr_clamp_to_range(wheel_motor1_target_lqr, -speed_limit_lqr, speed_limit_lqr);
    wheel_motor2_target_lqr = lqr_clamp_to_range(wheel_motor2_target_lqr, -speed_limit_lqr, speed_limit_lqr);
}

// 限幅函数
float lqr_clamp_to_range(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// 矩阵乘法函数 (用于未来扩展)
void lqr_matrix_multiply(float result[], const float A[][STATE_DIM], const float B[], int rows, int cols) {
    for(int i = 0; i < rows; i++) {
        result[i] = 0;
        for(int j = 0; j < cols; j++) {
            result[i] += A[i][j] * B[j];
        }
    }
}

// 设置参考值
void lqr_set_reference(float ref_angle, float ref_position) {
    reference_angle = ref_angle;
    reference_position = ref_position;
}

// 在线调参函数
void lqr_tune_parameters(float q_angle, float q_angular_vel, float q_position, float q_velocity, float r_input) {
    lqr_params.Q[0][0] = q_angle;
    lqr_params.Q[1][1] = q_angular_vel;
    lqr_params.Q[2][2] = q_position;
    lqr_params.Q[3][3] = q_velocity;
    lqr_params.R[0][0] = r_input;
    
    // 注意：在实际应用中，改变Q和R后需要重新计算K矩阵
    // 这里为了简化，使用预设的K值
    Serial.println("LQR参数已更新");
} 