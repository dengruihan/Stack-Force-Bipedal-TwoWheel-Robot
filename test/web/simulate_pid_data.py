#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID数据模拟器
模拟四路PID控制器的输出数据
"""

import time
import math
import random
import json
from datetime import datetime

class PIDSimulator:
    def __init__(self):
        # PID参数
        self.pid_params = {
            'vel_kp': 1.0,
            'balance_kp': 50.0,
            'balance_kd': 0.8,
            'robot_kp': 10.0
        }
        
        # 机器人状态
        self.robot_state = {
            'stable': 1,
            'linear': 0,
            'angular': 0,
            'roll': 0
        }
        
        # 仿真时间
        self.simulation_time = 0
        self.data_counter = 0
        
    def update_pid_params(self, vel_kp=None, balance_kp=None, balance_kd=None, robot_kp=None):
        """更新PID参数"""
        if vel_kp is not None:
            self.pid_params['vel_kp'] = vel_kp
        if balance_kp is not None:
            self.pid_params['balance_kp'] = balance_kp
        if balance_kd is not None:
            self.pid_params['balance_kd'] = balance_kd
        if robot_kp is not None:
            self.pid_params['robot_kp'] = robot_kp
            
        print(f"📐 PID参数更新: {self.pid_params}")
    
    def update_robot_state(self, stable=None, linear=None, angular=None, roll=None):
        """更新机器人状态"""
        if stable is not None:
            self.robot_state['stable'] = stable
        if linear is not None:
            self.robot_state['linear'] = linear
        if angular is not None:
            self.robot_state['angular'] = angular
        if roll is not None:
            self.robot_state['roll'] = roll
            
        print(f"🤖 机器人状态更新: {self.robot_state}")

    def simulate_pid_output(self, kp, input_signal=None):
        """模拟PID控制器输出"""
        if input_signal is None:
            # 生成模拟输入信号（包含噪声和趋势）
            input_signal = (
                math.sin(self.simulation_time * 0.5) * 0.3 +    # 主要信号
                math.sin(self.simulation_time * 2.1) * 0.1 +     # 高频分量
                random.uniform(-0.05, 0.05)                      # 噪声
            )
        
        # 模拟PID响应（简化版）
        output = kp * input_signal
        
        # 添加一些非线性和延迟效应
        if abs(output) > 1.0:
            output = math.copysign(1.0, output) * (1.0 - math.exp(-abs(output)))
        
        return output

    def generate_pid_data(self):
        """生成一次PID数据"""
        self.simulation_time += 0.05  # 50ms采样
        
        # 根据机器人状态调整输入信号
        base_input = 0
        if self.robot_state['stable'] == 1:
            base_input += self.robot_state['linear'] / 1000.0    # 线速度影响
            base_input += self.robot_state['angular'] / 500.0    # 角速度影响
            base_input += self.robot_state['roll'] / 100.0       # Roll角度影响
        
        # 模拟各个PID控制器的输出
        pid_data = {
            'type': 'pid_data',
            'timestamp': time.time(),
            'simulation_time': self.simulation_time,
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
        
        self.data_counter += 1
        return pid_data

    def run_simulation(self, duration=30, sample_rate=20):
        """运行仿真"""
        print("🚀 开始PID数据仿真")
        print(f"📊 仿真时长: {duration}秒")
        print(f"🔄 采样率: {sample_rate}Hz")
        print(f"📈 预计生成数据点: {duration * sample_rate}")
        print("-" * 50)
        
        sample_interval = 1.0 / sample_rate
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                loop_start = time.time()
                
                # 生成PID数据
                pid_data = self.generate_pid_data()
                
                # 打印数据（每秒打印一次）
                if self.data_counter % sample_rate == 0:
                    current_time = datetime.now().strftime("%H:%M:%S")
                    print(f"[{current_time}] 数据点 #{self.data_counter}")
                    print(f"  📐 vel_kp输出: {pid_data['vel_output']:.4f}")
                    print(f"  ⚖️  balance_kp输出: {pid_data['balance_output']:.4f}")
                    print(f"  📊 balance_kd输出: {pid_data['balance_kd_output']:.4f}")
                    print(f"  🤖 robot_kp输出: {pid_data['robot_output']:.4f}")
                    print(f"  ⏱️  仿真时间: {self.simulation_time:.2f}s")
                    print("-" * 40)
                
                # 保持采样率
                elapsed = time.time() - loop_start
                sleep_time = max(0, sample_interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n⏹️  仿真被用户中断")
        
        print(f"\n✅ 仿真完成!")
        print(f"📊 总计生成数据点: {self.data_counter}")
        print(f"⏱️  总仿真时间: {self.simulation_time:.2f}s")

def demo_pid_simulation():
    """演示PID仿真"""
    print("=" * 60)
    print("🧪 PID控制器数据仿真演示")
    print("=" * 60)
    
    # 创建仿真器
    simulator = PIDSimulator()
    
    print("\n📋 初始PID参数:")
    for key, value in simulator.pid_params.items():
        print(f"  {key}: {value}")
    
    print("\n🤖 机器人初始状态:")
    for key, value in simulator.robot_state.items():
        print(f"  {key}: {value}")
    
    print("\n" + "=" * 40)
    
    # 运行基础仿真
    print("🔹 场景1: 基础稳定状态仿真 (10秒)")
    simulator.run_simulation(duration=10, sample_rate=10)
    
    # 修改PID参数，观察影响
    print("\n🔹 场景2: 增加vel_kp参数后的仿真 (10秒)")
    simulator.update_pid_params(vel_kp=2.0)
    simulator.run_simulation(duration=10, sample_rate=10)
    
    # 激活机器人运动
    print("\n🔹 场景3: 机器人运动状态仿真 (10秒)")
    simulator.update_robot_state(linear=100, angular=30, roll=5)
    simulator.run_simulation(duration=10, sample_rate=10)
    
    print("\n🎉 所有仿真场景完成!")

def generate_sample_data(num_points=50):
    """生成示例数据用于测试"""
    print("📊 生成示例PID数据...")
    
    simulator = PIDSimulator()
    data_points = []
    
    for i in range(num_points):
        pid_data = simulator.generate_pid_data()
        data_points.append(pid_data)
        
        # 每10个点修改一次参数，模拟参数调节
        if i % 10 == 0 and i > 0:
            new_kp = 1.0 + (i / 50.0) * 2.0  # 从1.0逐渐增加到3.0
            simulator.update_pid_params(vel_kp=new_kp)
    
    print(f"✅ 生成了 {len(data_points)} 个数据点")
    
    # 保存到JSON文件
    with open('sample_pid_data.json', 'w', encoding='utf-8') as f:
        json.dump(data_points, f, indent=2, ensure_ascii=False)
    
    print("💾 数据已保存到 sample_pid_data.json")
    
    # 显示统计信息
    vel_outputs = [d['vel_output'] for d in data_points]
    balance_outputs = [d['balance_output'] for d in data_points]
    
    print(f"\n📈 数据统计:")
    print(f"  vel_kp输出范围: {min(vel_outputs):.4f} ~ {max(vel_outputs):.4f}")
    print(f"  balance_kp输出范围: {min(balance_outputs):.4f} ~ {max(balance_outputs):.4f}")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "demo":
            demo_pid_simulation()
        elif sys.argv[1] == "sample":
            num_points = int(sys.argv[2]) if len(sys.argv) > 2 else 50
            generate_sample_data(num_points)
        else:
            print("用法:")
            print("  python simulate_pid_data.py demo    # 运行演示仿真")
            print("  python simulate_pid_data.py sample [数量]  # 生成示例数据")
    else:
        print("🎯 选择运行模式:")
        print("1. demo - 运行演示仿真")
        print("2. sample - 生成示例数据")
        
        choice = input("\n请选择 (1/2): ").strip()
        
        if choice == "1":
            demo_pid_simulation()
        elif choice == "2":
            num_points = input("数据点数量 (默认50): ").strip()
            num_points = int(num_points) if num_points else 50
            generate_sample_data(num_points)
        else:
            print("❌ 无效选择") 