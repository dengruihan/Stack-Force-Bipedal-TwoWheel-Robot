# 🤖 机器人Web控制 & PID监控系统

## 📋 系统概述

这是一个增强版的ESP32机器人Web控制系统，集成了PID参数调节和实时曲线监控功能。

### ✨ 主要功能

1. **🎮 机器人控制**
   - 摇杆控制（X/Y轴）
   - 方向按钮（前进/后退/左转/右转/跳跃）
   - 滑块控制（Roll角度、基础高度、线速度、角速度）
   - 机器人启停开关

2. **⚙️ PID参数设置**
   - `vel_kp` - 速度控制器比例增益
   - `balance_kp` - 平衡控制器比例增益  
   - `balance_kd` - 平衡控制器微分增益
   - `robot_kp` - 机器人控制器比例增益

3. **📈 实时曲线监控**
   - 四路PID输出值实时显示
   - Chart.js图表可视化
   - 数据窗口管理（最多100个数据点）
   - 采样率实时显示

4. **🚫 数据堆积防护**
   - 滑动窗口机制
   - 20Hz采样率
   - 自动清理老旧数据

## 🚀 快速开始

### 方法1: 一键启动（推荐）
```bash
python start_enhanced_server.py
```

### 方法2: 手动启动
```bash
# 安装依赖
pip install -r requirements.txt

# 确保HTML文件在正确位置
mkdir -p data
cp robot_web_control.html data/

# 启动服务器
python test_server_enhanced.py
```

### 访问界面
- 打开浏览器访问: http://localhost
- WebSocket端口: 81

## 🎯 使用指南

### 1. PID参数调节
1. 在左侧控制面板找到"⚙️ PID参数设置"
2. 调整四个PID参数值
3. 点击"发送PID参数"按钮
4. 服务器会实时接收并应用新参数

### 2. 实时曲线监控
1. 在右侧监控面板点击"开始监控"
2. 观察四路PID输出曲线的实时变化
3. 调节PID参数，观察曲线变化
4. 使用"清除数据"重置图表

### 3. 机器人控制
1. 开启"Robot Go!"开关
2. 使用摇杆、按钮或滑块控制机器人
3. 观察PID输出如何响应控制输入

## 📊 数据格式

### PID参数数据格式
```json
{
  "type": "pid_params",
  "vel_kp": 1.0,
  "balance_kp": 50.0,
  "balance_kd": 0.8,
  "robot_kp": 10.0,
  "mode": "basic"
}
```

### PID监控数据格式
```json
{
  "type": "pid_data",
  "timestamp": 1703123456.789,
  "vel_output": 0.123,
  "balance_output": 0.456,
  "balance_kd_output": 0.789,
  "robot_output": 0.012
}
```

### 控制命令数据格式
```json
{
  "type": "control",
  "roll": 0,
  "height": 38,
  "linear": 0,
  "angular": 0,
  "stable": 1,
  "mode": "basic",
  "dir": "stop",
  "joy_x": 0,
  "joy_y": 0
}
```

## 🏗️ 技术架构

### 前端技术栈
- **HTML5 + CSS3**: 响应式布局
- **JavaScript**: 原生JS，无外部依赖
- **Chart.js**: 实时图表绘制
- **WebSocket**: 双向通信

### 后端技术栈
- **Python 3.7+**: 服务器语言
- **websockets**: WebSocket服务器
- **asyncio**: 异步处理
- **HTTP Server**: 静态文件服务

### 数据流程
```
网页界面 ←→ WebSocket ←→ Python服务器 ←→ 模拟PID算法
    ↓                                        ↓
 图表显示                               数据计算&存储
```

## ⚙️ 配置说明

### 采样率配置
```python
# 在 test_server_enhanced.py 中修改
await asyncio.sleep(0.05)  # 20Hz = 50ms间隔
```

### 数据窗口配置
```javascript
// 在 HTML 中修改
var dataWindow = 100; // 最大数据点数
```

### 端口配置
```python
# 在服务器中修改
http_port = 80      # HTTP端口
websocket_port = 81 # WebSocket端口
```

## 🔧 ESP32集成

### 修改ESP32代码
在你的ESP32项目中添加PID参数处理：

```cpp
// 在 webSocketEventCallback 函数中添加
if (doc["type"] == "pid_params") {
    vel_kp = doc["vel_kp"];
    balance_kp = doc["balance_kp"];
    balance_kd = doc["balance_kd"];
    robot_kp = doc["robot_kp"];
    Serial.println("PID参数已更新");
}
```

### 发送PID数据
```cpp
void sendPIDData() {
    StaticJsonDocument<200> pidDoc;
    pidDoc["type"] = "pid_data";
    pidDoc["timestamp"] = millis() / 1000.0;
    pidDoc["vel_output"] = current_vel_output;
    pidDoc["balance_output"] = current_balance_output;
    pidDoc["balance_kd_output"] = current_balance_kd_output;
    pidDoc["robot_output"] = current_robot_output;
    
    String pidData;
    serializeJson(pidDoc, pidData);
    websocket.broadcastTXT(pidData);
}
```

## 🐛 故障排除

### 常见问题

1. **连接失败**
   - 检查端口是否被占用
   - 确认防火墙设置
   - 验证WebSocket地址

2. **图表不显示**
   - 检查Chart.js CDN连接
   - 确认JavaScript控制台无错误
   - 验证数据格式正确

3. **PID数据异常**
   - 检查服务器端模拟算法
   - 验证数据类型转换
   - 确认采样率设置

### 调试方法

1. **浏览器调试**
   ```javascript
   // 在浏览器控制台中运行
   console.log('WebSocket状态:', socket.readyState);
   console.log('PID参数:', pidParams);
   ```

2. **服务器调试**
   ```python
   # 在 Python 代码中添加
   print(f"接收到数据: {data}")
   print(f"当前PID参数: {self.pid_params}")
   ```

## 📈 性能优化

### 前端优化
- 图表动画禁用（提高性能）
- 数据窗口限制（防止内存泄漏）
- 事件节流（减少频繁更新）

### 后端优化
- 异步处理（提高并发）
- 连接池管理（处理断开重连）
- 数据缓存（减少计算开销）

## 🔮 未来扩展

### 计划功能
- [ ] 多机器人支持
- [ ] 历史数据存储
- [ ] PID自动调参
- [ ] 数据导出功能
- [ ] 移动端适配

### 自定义扩展
- 添加新的PID控制器
- 集成机器学习算法
- 支持多种控制模式
- 增加数据分析功能

## 📞 支持

如有问题或建议，请：
1. 检查此文档的故障排除部分
2. 查看服务器控制台输出
3. 检查浏览器开发者工具
4. 参考示例代码实现

---

**祝您使用愉快！** 🎉 