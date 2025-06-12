#include "pid.h"

// PID参数
float vel_kp = -0;//速度环
float balance_kp = -0;//直立环Kp
float balance_kd = 0;//直立环K
int speed_limit = 3; //轮毂电机速度限制
float motor1_target_vel = 0, motor2_target_vel = 0;
float wheel_motor1_target = 0, wheel_motor2_target = 0; // 电机目标值
float clampToRange(float value, float minVal, float maxVal);
// 底部轮子霍尔电机PID控制函数
void wheel_control()
{
  // 开启轮速环
  motor1_target_vel = (-vel_kp * (forwardBackward  - (motor1_vel  + motor2_vel) / 2));
  motor2_target_vel = (-vel_kp * (forwardBackward  - (motor1_vel  + motor2_vel) / 2));

  // 根据高度调整平衡控制参数
  float height_factor = 1.0;
  if (ZeparamremoteValue > 80) {
    height_factor = 1.2;  // 高机身时增强平衡控制
  }

  wheel_motor1_target = clampToRange((balance_kp * height_factor * (motor1_target_vel - pitch - balance_offset - remoteBalanceOffset) + balance_kd * gyroY), -5, 5);
  wheel_motor2_target = clampToRange((balance_kp * height_factor * (motor2_target_vel - pitch - balance_offset - remoteBalanceOffset) + balance_kd * gyroY), -5, 5);

  wheel_motor1_target = -wheel_motor1_target - 0.7 * steering;
  wheel_motor2_target = -wheel_motor2_target + 0.7 * steering;

  wheel_motor1_target = clampToRange(wheel_motor1_target, -speed_limit, speed_limit);
  wheel_motor2_target = clampToRange(wheel_motor2_target, -speed_limit, speed_limit);
}

//PID线性拟合函数
PIDValues  interpolatePID(int y_height) {
  if(Shake_shoulder == 0)
  {
      // 已知数据点 - 调整参数以提高高机身稳定性
      float y0 = 0, y1 = 80, y2 = 150;
      PIDValues pid0 = {-0.55,-0.155, 0.048, 3.5};  // 低机身参数
      PIDValues pid1 = {-0.55,-0.163,0.06, 4.5};    // 中机身参数
      PIDValues pid2 = {-0.52,-0.175,0.072, 7.0};   // 高机身参数 - 增强稳定性
      PIDValues result;
      if (y_height <= y1) 
      {
          speed_limit = 5;
          float t = (y_height - y0) / (y1 - y0);
          vel_kp = pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp);
          balance_kp = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
          balance_kd = pid0.linear_balance_kd + t * (pid1.linear_balance_kd - pid0.linear_balance_kd);
          robot_kp = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp);
      } 
      else 
      {
          speed_limit = 2;  // 降低高机身时的速度限制
          float t = (y_height - y1) / (y2 - y1);
          vel_kp = pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp);
          balance_kp = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp);
          balance_kd = pid1.linear_balance_kd + t * (pid2.linear_balance_kd - pid1.linear_balance_kd);
          robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
      }
      return result;
  }
}

//PID串口调参函数
String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      command = received_chars;
      command.trim(); // 去除前后空格
      
      // 检查控制模式切换命令
      if (command.equals("lqr")) {
        extern bool use_lqr_control; // 引用主程序中的控制开关
        use_lqr_control = true;
        Serial.println("已切换到LQR控制模式");
        received_chars = "";
        return command;
      }
      else if (command.equals("pid")) {
        extern bool use_lqr_control;
        use_lqr_control = false;
        Serial.println("已切换到PID控制模式");
        received_chars = "";
        return command;
      }
      
      int commaPosition = command.indexOf(',');
      int newlinePosition = command.indexOf('\n');
      if (commaPosition != -1 && newlinePosition != -1) //给的第一个值
      {
        String firstParam = command.substring(0, commaPosition);
        vel_kp = firstParam.toDouble(); //速度环Kp
        Serial.print("vel_kp:");
        Serial.println(vel_kp);

        String secondParamStr = command.substring(commaPosition + 1, newlinePosition);
        int secondCommaPosition = secondParamStr.indexOf(',');
        if (secondCommaPosition != -1) //如果给三个值
        {
          balance_kp = secondParamStr.substring(0, secondCommaPosition).toDouble();//直立环Kp
          balance_kd = secondParamStr.substring(secondCommaPosition + 1).toDouble();//直立环Kd
        }
        else //如果只给两个值
        {
          robot_kp = secondParamStr.toDouble();
        }
        Serial.print("balance_kp:");
        Serial.println(balance_kp,3);
        Serial.print("balance_kd:");
        Serial.println(balance_kd,3);
        Serial.print("robot_kp:");
        Serial.println(robot_kp,3);
      }
      received_chars = "";
    }
  }
  return command;
}

// 限幅函数
float clampToRange(float value, float minVal, float maxVal)
{
  if (value < minVal)
    return minVal;
  if (value > maxVal)
    return maxVal;
  return value;
}