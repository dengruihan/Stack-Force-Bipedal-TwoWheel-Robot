#include "main.h"

#define DEBUG_TIME false // 打印时间间隔
#define DEBUG_REMOTOR false // 打印遥控器数据
#define DEBUG_MPU false // 打印陀螺儀的調試信息
#define DEBUG_DURATION 10 // DEBUG_REMOTOR 建議為10，
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050
hw_timer_t *timer = NULL; // Timer for accurate pulse width measurement
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int cnt;
unsigned long __lastTime = 0;
unsigned long loopCount = 0;

// PID数据发送相关变量
unsigned long lastPIDSendTime = 0;
const unsigned long PID_SEND_INTERVAL = 50; // 50ms发送一次，20Hz频率

void IMUTask(void *pvParameters);
void testdataprint(); //调试打印函数
void Open_thread_function();//启动线程
void sendPIDData(); // 发送PID数据到网页
void processPIDParams(const JsonDocument& doc); // 处理PID参数

// WebServer实例
WebServer webserver;                               // server服务器
WebSocketsServer websocket = WebSocketsServer(81); // 定义一个webSocket服务器来处理客户发送的消息

void setup()
{
  Wire.begin(1, 2, 400000UL);//初始化IIC
  mpu6050.begin(); //初始化MPU陀螺仪


  #ifdef USE_WEB_SERVER
      // Wifi初始化
      WiFi_SetAP();
      // set_sta_wifi();      // ESP-01S STA模式接入WiFi网络
      webserver.begin();
      webserver.on("/", HTTP_GET, basicWebCallback);
      websocket.begin();
      websocket.onEvent(webSocketEventCallback);
  #endif

  Open_thread_function();//启动线程
  Serial.begin(115200);//初始化调试串口
  CANInit(); // 初始化CAN
  ppm_init(); //遥控器读取中断初始化
  motorInit();
  delay(3000);
  CAN_Control();//使能关节电机
  delay(1000);
}

void loop()
{
  web_loop();         // Web数据更新
  serialReceiveUserCommand();                                 // 串口数据输入处理 用于调试pid用
  PIDValues pid = interpolatePID(ZeparamremoteValue);         //PID线性拟合函数
  wheel_control();                                            // 轮子 霍尔电机PID控制函数
  CAN_Control();                                              // CAN 关节电机控制函数
  remote_switch();                                            // 遥控开关
  jump_control();                                             // 机器人跳跃控制
  inverseKinematics();                                           // 运动学逆解
  robot_control();                                            // 机器人行为控制
  sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target); // 发送控制轮毂电机的目标值
  storeFilteredPPMData();                                     // 对ppm数据进行滤波处理 避免数据大幅度跳动
  mapPPMToRobotControl();                                     // 处理遥控器数据 将其映射为机器人行为控制
  
  // 定期发送PID数据到网页
  #ifdef USE_WEB_SERVER
    sendPIDData();
  #endif
  
  #if DEBUG_MPU == true or DEBUG_REMOTOR == true
    testdataprint();         
  #endif

  #if DEBUG_TIME == true
    loopCount++;
    unsigned long currentTime = millis();
    Serial.println(currentTime - __lastTime);
    __lastTime = currentTime;
  #endif
}

// 测试数据打印函数 用于打印调试相关数据
void testdataprint()
{
  if (cnt++ > DEBUG_DURATION)
  {
    cnt = 0;
    
    #if DEBUG_MPU == true
      Serial.print(gyroZ);
      Serial.print("\t");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print(gyroY);
      Serial.print("\t");
      Serial.println(ZeparamremoteValue);
    #endif

    #if DEBUG_REMOTOR == true
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        Serial.print(ppmValues[i]);
        if (i< NUM_CHANNELS-1) Serial.print(",");
        else Serial.println();
      }
    #endif
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


#ifdef USE_WEB_SERVER
// Web数据更新
void web_loop()
{
  webserver.handleClient();
  websocket.loop();
}

void basicWebCallback(void)
{
  webserver.send(300, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (type == WStype_TEXT)
  {
    String payload_str = String((char *)payload);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload_str);
    if (error)
    {
      Serial.println("JSON解析失败");
      return;
    }

    // 获取消息类型
    String messageType = doc["type"];
    
    if (messageType == "pid_params")
    {
      // 处理PID参数设置
      processPIDParams(doc);
    }
    else if (messageType == "control")
    {
      // 处理控制命令（可以在这里添加其他控制逻辑）
      Serial.println("收到控制命令");
    }
  }
}

// 处理PID参数
void processPIDParams(const JsonDocument& doc)
{
  Serial.println("收到PID参数设置:");
  
  // 更新PID参数
  if (doc.containsKey("vel_kp"))
  {
    vel_kp = doc["vel_kp"];
    Serial.print("vel_kp: ");
    Serial.println(vel_kp, 3);
  }
  
  if (doc.containsKey("balance_kp"))
  {
    balance_kp = doc["balance_kp"];
    Serial.print("balance_kp: ");
    Serial.println(balance_kp, 3);  
  }
  
  if (doc.containsKey("balance_kd"))
  {
    balance_kd = doc["balance_kd"];
    Serial.print("balance_kd: ");
    Serial.println(balance_kd, 3);
  }
  
  if (doc.containsKey("robot_kp"))
  {
    robot_kp = doc["robot_kp"];
    Serial.print("robot_kp: ");
    Serial.println(robot_kp, 3);
  }
  
  // 发送确认消息到网页
  StaticJsonDocument<200> response;
  response["type"] = "pid_confirm";
  response["vel_kp"] = vel_kp;
  response["balance_kp"] = balance_kp;
  response["balance_kd"] = balance_kd;
  response["robot_kp"] = robot_kp;
  
  String responseStr;
  serializeJson(response, responseStr);
  websocket.broadcastTXT(responseStr);
}

// 发送PID数据到网页
void sendPIDData()
{
  unsigned long currentTime = millis();
  if (currentTime - lastPIDSendTime >= PID_SEND_INTERVAL)
  {
    lastPIDSendTime = currentTime;
    
    // 计算PID输出值（这些是实际的误差数据）
    float vel_output = -vel_kp * (forwardBackward - (motor1_vel + motor2_vel) / 2);
    float balance_output = balance_kp * (pitch - balance_offset - remoteBalanceOffset);
    float balance_kd_output = balance_kd * gyroY;
    float robot_output = robot_kp * 0.1; // 示例值，根据实际逻辑调整
    
    // 创建JSON数据
    StaticJsonDocument<300> pidData;
    pidData["type"] = "pid_data";
    pidData["timestamp"] = currentTime;
    pidData["vel_output"] = vel_output;
    pidData["balance_output"] = balance_output;
    pidData["balance_kd_output"] = balance_kd_output;
    pidData["robot_output"] = robot_output;
    
    // 添加一些额外的状态信息
    pidData["pitch"] = pitch;
    pidData["gyroY"] = gyroY;
    pidData["motor1_vel"] = motor1_vel;
    pidData["motor2_vel"] = motor2_vel;
    
    String pidDataStr;
    serializeJson(pidData, pidDataStr);
    websocket.broadcastTXT(pidDataStr);
  }
}
#endif