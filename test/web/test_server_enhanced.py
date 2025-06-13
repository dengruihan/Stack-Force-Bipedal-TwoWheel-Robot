#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32机器人Web控制增强版服务器模拟程序
支持PID参数控制和实时曲线监控
"""

import asyncio
import websockets
import json
import threading
import time
import math
import random
from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
from urllib.parse import urlparse

class EnhancedRobotTestServer:
    def __init__(self, http_port=8080, websocket_port=8081):
        self.http_port = http_port
        self.websocket_port = websocket_port
        self.robot_state = {
            'roll': 0,
            'height': 38,
            'linear': 0,
            'angular': 0,
            'stable': 0,
            'mode': 'basic',
            'dir': 'stop',
            'joy_x': 0,
            'joy_y': 0
        }
        
        # PID参数
        self.pid_params = {
            'vel_kp': 1.0,
            'balance_kp': 50.0,
            'balance_kd': 0.8,
            'robot_kp': 10.0
        }
        
        # 监控状态
        self.monitoring = False
        self.connected_clients = set()
        self.simulation_time = 0
        self.data_counter = 0
        
    def load_html_content(self):
        """加载HTML文件内容"""
        html_path = "../../data/robot_web_control.html"
        if os.path.exists(html_path):
            with open(html_path, 'r', encoding='utf-8') as f:
                return f.read()
        else:
            return """
            <html>
            <body>
                <h1>HTML文件未找到</h1>
                <p>请确保 data/robot_web_control.html 文件存在</p>
                <p>或者运行以下命令创建测试文件:</p>
                <pre>mkdir -p data && cp robot_web_control.html data/</pre>
            </body>
            </html>
            """

    def simulate_pid_output(self, kp, input_signal=None):
        """模拟PID控制器输出"""
        if input_signal is None:
            # 生成模拟输入信号（包含噪声和趋势）
            input_signal = (
                math.sin(self.simulation_time * 0.5) * 0.3 +  # 主要信号
                math.sin(self.simulation_time * 2.1) * 0.1 +   # 高频分量
                random.uniform(-0.05, 0.05)                    # 噪声
            )
        
        # 模拟PID响应（简化版）
        output = kp * input_signal
        
        # 添加一些非线性和延迟效应
        if abs(output) > 1.0:
            output = math.copysign(1.0, output) * (1.0 - math.exp(-abs(output)))
        
        return output

    def generate_pid_data(self):
        """生成PID曲线数据"""
        self.simulation_time += 0.05  # 50ms采样
        
        # 根据机器人状态调整输入信号
        base_input = 0
        if self.robot_state['stable'] == 1:
            base_input += self.robot_state['linear'] / 1000.0  # 线速度影响
            base_input += self.robot_state['angular'] / 500.0  # 角速度影响
            base_input += self.robot_state['roll'] / 100.0     # Roll角度影响
        
        # 模拟各个PID控制器的输出
        pid_data = {
            'type': 'pid_data',
            'timestamp': time.time(),
            'vel_output': self.simulate_pid_output(
                self.pid_params['vel_kp'], 
                base_input + math.sin(self.simulation_time * 0.8) * 0.2
            ),
            'balance_output': self.simulate_pid_output(
                self.pid_params['balance_kp'] / 50.0,  # 缩放以便显示
                base_input + math.sin(self.simulation_time * 1.2) * 0.3
            ),
            'balance_kd_output': self.simulate_pid_output(
                self.pid_params['balance_kd'],
                base_input + math.cos(self.simulation_time * 1.5) * 0.15
            ),
            'robot_output': self.simulate_pid_output(
                self.pid_params['robot_kp'] / 10.0,  # 缩放以便显示
                base_input + math.sin(self.simulation_time * 0.3) * 0.4
            )
        }
        
        return pid_data

class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, server_instance, *args, **kwargs):
        self.server_instance = server_instance
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """处理HTTP GET请求"""
        if self.path == '/' or self.path == '/index.html':
            # 返回HTML页面
            html_content = self.server_instance.load_html_content()
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(html_content.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(html_content.encode('utf-8'))
        else:
            # 其他请求返回404
            self.send_response(404)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body><h1>404 Not Found</h1></body></html>')
    
    def log_message(self, format, *args):
        """自定义日志输出"""
        print(f"🌐 HTTP请求: {format % args}")

async def websocket_handler(websocket, path, server_instance):
    """WebSocket消息处理器"""
    print(f"🔗 WebSocket客户端连接: {websocket.remote_address}")
    server_instance.connected_clients.add(websocket)
    
    try:
        async for message in websocket:
            try:
                # 解析JSON数据
                data = json.loads(message)
                await handle_client_message(data, server_instance, websocket)
                
            except json.JSONDecodeError as e:
                print(f"❌ JSON解析错误: {e}")
            except Exception as e:
                print(f"❌ 处理消息时出错: {e}")
                
    except websockets.exceptions.ConnectionClosed:
        print(f"🔌 WebSocket客户端断开连接: {websocket.remote_address}")
    except Exception as e:
        print(f"❌ WebSocket连接错误: {e}")
    finally:
        server_instance.connected_clients.discard(websocket)

async def handle_client_message(data, server_instance, websocket):
    """处理客户端消息"""
    message_type = data.get('type', 'control')
    
    if message_type == 'control':
        # 更新机器人状态
        server_instance.robot_state.update(data)
        await simulate_robot_response(data, server_instance)
        
    elif message_type == 'pid_params':
        # 更新PID参数
        server_instance.pid_params.update({
            'vel_kp': data.get('vel_kp', server_instance.pid_params['vel_kp']),
            'balance_kp': data.get('balance_kp', server_instance.pid_params['balance_kp']),
            'balance_kd': data.get('balance_kd', server_instance.pid_params['balance_kd']),
            'robot_kp': data.get('robot_kp', server_instance.pid_params['robot_kp'])
        })
        print(f"⚙️  PID参数更新:")
        for key, value in server_instance.pid_params.items():
            print(f"   {key}: {value}")
        print("-" * 40)
        
    elif message_type == 'start_monitoring':
        server_instance.monitoring = True
        server_instance.simulation_time = 0
        print("📈 开始PID监控")
        
    elif message_type == 'stop_monitoring':
        server_instance.monitoring = False
        print("⏹️  停止PID监控")

async def simulate_robot_response(data, server_instance):
    """模拟机器人行为响应"""
    mode = data.get('mode', 'basic')
    direction = data.get('dir', 'stop')
    stable = data.get('stable', 0)
    
    status_msg = "🤖 机器人状态: " + ("🟢 启动" if stable == 1 else "🔴 停止")
    
    if mode == 'basic':
        action_msg = ""
        if direction == 'forward':
            action_msg = "🤖 动作: ⬆️ 前进"
        elif direction == 'back':
            action_msg = "🤖 动作: ⬇️ 后退"
        elif direction == 'left':
            action_msg = "🤖 动作: ⬅️ 左转"
        elif direction == 'right':
            action_msg = "🤖 动作: ➡️ 右转"
        elif direction == 'jump':
            action_msg = "🤖 动作: ⬆️ 跳跃"
        elif direction == 'stop':
            if data.get('joy_x', 0) != 0 or data.get('joy_y', 0) != 0:
                action_msg = f"🕹️  摇杆控制: X={data.get('joy_x', 0)}, Y={data.get('joy_y', 0)}"
            else:
                action_msg = "🤖 动作: ⏸️ 停止"
        
        if action_msg:
            print(f"{status_msg} | {action_msg}")
    
    # 显示参数变化（只在有变化时显示）
    params_changed = []
    if 'roll' in data:
        params_changed.append(f"📐 Roll: {data['roll']}°")
    if 'height' in data:
        params_changed.append(f"📏 高度: {data['height']}mm")
    if 'linear' in data:
        params_changed.append(f"🏃 线速度: {data['linear']}mm/s")
    if 'angular' in data:
        params_changed.append(f"🔄 角速度: {data['angular']}°/s")
    
    if params_changed:
        print(" | ".join(params_changed))
    
    if action_msg or params_changed:
        print("-" * 50)

async def pid_monitoring_task(server_instance):
    """PID监控任务"""
    while True:
        if server_instance.monitoring and server_instance.connected_clients:
            # 生成PID数据
            pid_data = server_instance.generate_pid_data()
            
            # 发送给所有连接的客户端
            disconnected_clients = set()
            for client in server_instance.connected_clients:
                try:
                    await client.send(json.dumps(pid_data))
                except websockets.exceptions.ConnectionClosed:
                    disconnected_clients.add(client)
                except Exception as e:
                    print(f"❌ 发送PID数据失败: {e}")
                    disconnected_clients.add(client)
            
            # 移除断开连接的客户端
            server_instance.connected_clients -= disconnected_clients
            
            server_instance.data_counter += 1
            if server_instance.data_counter % 20 == 0:  # 每秒打印一次状态
                print(f"📊 PID数据发送中... (客户端: {len(server_instance.connected_clients)}, "
                      f"数据点: {server_instance.data_counter})")
        
        await asyncio.sleep(0.05)  # 20Hz采样率

def start_http_server(server_instance):
    """启动HTTP服务器"""
    def handler(*args, **kwargs):
        return CustomHTTPRequestHandler(server_instance, *args, **kwargs)
    
    httpd = HTTPServer(('', server_instance.http_port), handler)
    print(f"🌐 HTTP服务器启动在端口 {server_instance.http_port}")
    print(f"🔗 访问地址: http://localhost:{server_instance.http_port}")
    httpd.serve_forever()

async def start_websocket_server(server_instance):
    """启动WebSocket服务器"""
    async def handler(websocket, path):
        await websocket_handler(websocket, path, server_instance)
    
    print(f"🔌 WebSocket服务器启动在端口 {server_instance.websocket_port}")
    await websockets.serve(handler, "localhost", server_instance.websocket_port)

async def main():
    """主函数"""
    print("=" * 60)
    print("🤖 ESP32 机器人Web控制增强版服务器模拟程序")
    print("📈 支持PID参数控制和实时曲线监控")
    print("=" * 60)
    
    # 创建服务器实例
    server = EnhancedRobotTestServer()
    
    # 在单独线程中启动HTTP服务器
    http_thread = threading.Thread(target=start_http_server, args=(server,))
    http_thread.daemon = True
    http_thread.start()
    
    # 启动PID监控任务
    pid_task = asyncio.create_task(pid_monitoring_task(server))
    
    # 启动WebSocket服务器
    websocket_task = asyncio.create_task(start_websocket_server(server))
    
    print("\n📋 功能说明:")
    print("  🎮 机器人控制: 摇杆、按钮、滑块控制")
    print("  ⚙️  PID参数设置: vel_kp, balance_kp, balance_kd, robot_kp")
    print("  📈 实时曲线监控: PID输出值可视化")
    print("  🚫 数据窗口限制: 最多显示100个数据点，防止堆积")
    print("  📊 采样率显示: 实时显示数据采样频率")
    print("\n🚀 服务器就绪，等待连接...")
    print("   按 Ctrl+C 停止服务器\n")
    
    # 保持程序运行
    try:
        await asyncio.gather(pid_task, websocket_task)
    except KeyboardInterrupt:
        print("\n🛑 服务器停止")

if __name__ == "__main__":
    # 检查依赖
    try:
        import websockets
    except ImportError:
        print("❌ 请先安装websockets库:")
        print("   pip install websockets")
        exit(1)
    
    # 检查HTML文件
    if not os.path.exists("../../data/robot_web_control.html"):
        print("⚠️  警告: data/robot_web_control.html 文件不存在")
        print("   请确保HTML文件在正确位置")
        print("   或运行: mkdir -p data && cp robot_web_control.html data/")
        print()
    
    # 运行服务器
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 再见！") 