#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键启动增强版机器人Web控制服务器
包含依赖检查和自动安装功能
"""

import os
import sys
import subprocess

def check_and_install_dependencies():
    """检查并安装Python依赖"""
    try:
        import websockets
        print("✅ websockets 已安装")
        return True
    except ImportError:
        print("📦 正在安装 websockets...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets>=11.0.3"])
            print("✅ websockets 安装成功")
            return True
        except subprocess.CalledProcessError:
            print("❌ websockets 安装失败，请手动安装:")
            print("   pip install websockets")
            return False

def check_html_file():
    """检查HTML文件是否存在"""
    if os.path.exists("../../data/robot_web_control.html"):
        print("✅ HTML文件已找到")
        return True
    else:
        print("⚠️  警告: data/robot_web_control.html 文件不存在")
        
        # 检查根目录是否有HTML文件
        if os.path.exists("robot_web_control.html"):
            print("📁 在根目录找到HTML文件，正在复制到data文件夹...")
            try:
                os.makedirs("data", exist_ok=True)
                import shutil
                shutil.copy("robot_web_control.html", "../../data/robot_web_control.html")
                print("✅ HTML文件复制成功")
                return True
            except Exception as e:
                print(f"❌ 复制文件失败: {e}")
                return False
        else:
            print("❌ 未找到HTML文件，请确保文件存在")
            return False

def main():
    """主函数"""
    print("🚀 启动增强版机器人Web控制服务器")
    print("=" * 50)
    
    # 检查Python版本
    if sys.version_info < (3, 7):
        print("❌ 需要Python 3.7或更高版本")
        print(f"   当前版本: {sys.version}")
        return False
    
    print(f"✅ Python版本: {sys.version}")
    
    # 检查并安装依赖
    if not check_and_install_dependencies():
        return False
    
    # 检查HTML文件
    if not check_html_file():
        return False
    
    print("\n🌟 所有检查通过，启动服务器...")
    print("🔗 访问地址: http://localhost")
    print("📈 功能特性:")
    print("   • 🎮 机器人遥控")
    print("   • ⚙️  PID参数调节")
    print("   • 📊 实时曲线监控")
    print("   • 🚫 数据窗口管理")
    print("\n按 Ctrl+C 停止服务器\n")
    
    # 启动服务器
    try:
        import test_server_enhanced
        import asyncio
        asyncio.run(test_server_enhanced.main())
    except KeyboardInterrupt:
        print("\n🛑 服务器已停止")
    except ImportError:
        print("❌ 找不到 test_server_enhanced.py 文件")
    except Exception as e:
        print(f"❌ 启动失败: {e}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 再见！") 