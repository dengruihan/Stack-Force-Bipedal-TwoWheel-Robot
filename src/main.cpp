#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "bipedal_data.h"
#include "CAN_comm.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ppm.h"
#include "Motor.h"
#include "robot.h"
#include "can.h"
#include "pid.h"
#include "lqr.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050
hw_timer_t *timer = NULL; // Timer for accurate pulse width measurement
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int cnt;
bool use_lqr_control = true;  // LQR控制开关：true=LQR控制，false=PID控制

void IMUTask(void *pvParameters);
void testdataprint(); //调试打印函数
void Open_thread_function();//启动线程


void setup()
{
  Wire.begin(1, 2, 400000UL);//初始化IIC
  mpu6050.begin(); //初始化MPU陀螺仪
  Open_thread_function();//启动线程
  Serial.begin(115200);//初始化调试串口
  CANInit(); // 初始化CAN
  ppm_init(); //遥控器读取中断初始化
  motorInit();
  delay(3000);
  CAN_Control();//使能关节电机
  delay(1000);
  
  // 初始化LQR控制器
  if (use_lqr_control) {
    lqr_init();
    Serial.println("系统启动 - 使用LQR控制模式");
  } else {
    Serial.println("系统启动 - 使用PID控制模式");
  }
}


void loop()
{

  serialReceiveUserCommand();                                 // 串口数据输入处理 用于调试pid用
  
  // 控制算法选择
  if (use_lqr_control) {
    lqr_wheel_control();                                      // LQR轮子控制函数
  } else {
    PIDValues pid = interpolatePID(ZeparamremoteValue);       //PID线性拟合函数
    wheel_control();                                          // 轮子 霍尔电机PID控制函数
  }
  CAN_Control();                                              // CAN 关节电机控制函数
  remote_switch();                                            // 遥控开关
  jump_control();                                             // 机器人跳跃控制
  inverseKinematics();                                           // 运动学逆解
  robot_control();                                            // 机器人行为控制
  // 发送控制轮毂电机的目标值 - 根据控制模式选择相应的目标值
  if (use_lqr_control) {
    sendMotorTargets(up_start * wheel_motor1_target_lqr, up_start * wheel_motor2_target_lqr);
  } else {
    sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target);
  }
  storeFilteredPPMData();                                     // 对ppm数据进行滤波处理 避免数据大幅度跳动
  mapPPMToRobotControl();                                     // 处理遥控器数据 将其映射为机器人行为控制
  testdataprint();                                            // 测试数据打印 可以打印输出相关调节信息

}

// 测试数据打印函数 用于打印调试相关数据
void testdataprint()
{
  if (cnt++ > 5000)
  {
    cnt = 0;
    Serial.print("模式: ");
    Serial.print(use_lqr_control ? "LQR" : "PID");
    Serial.print("\t角度: ");
    Serial.print(pitch);
    Serial.print("\t角速度: ");
    Serial.print(gyroY);
    Serial.print("\t高度: ");
    Serial.print(ZeparamremoteValue);
    
    if (use_lqr_control) {
      Serial.print("\tLQR输出1: ");
      Serial.print(wheel_motor1_target_lqr);
      Serial.print("\tLQR输出2: ");
      Serial.print(wheel_motor2_target_lqr);
    } else {
      Serial.print("\tPID输出1: ");
      Serial.print(wheel_motor1_target);
      Serial.print("\tPID输出2: ");
      Serial.print(wheel_motor2_target);
    }
    Serial.println();
  }
}
//启动线程
void Open_thread_function() 
{
    // 陀螺仪读取任务进程
    xTaskCreatePinnedToCore(
        IMUTask,   // 任务函数
        "IMUTask", // 任务名称
        4096,      // 堆栈大小
        NULL,      // 传递的参数
        1,         // 任务优先级
        NULL,      // 任务句柄
        1          // 运行在核心 0
    );
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters) 
{
  gyroY = mpu6050.getGyroY();  // 第一次读出来作为初值
  
  while (true) 
  {
    mpu6050.update();
    roll = mpu6050.getAngleX();
    pitch = mpu6050.getAngleY();
    yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroZ = mpu6050.getGyroZ();
    // 这里是设置一定死区 避免数据的波动
    if (gyroZ > -3 && gyroZ < 6) gyroZ = 0;
    gyroY = lowPassFilter(mpu6050.getGyroY(), gyroY, 0.005);

  }
}
