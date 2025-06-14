const char basic_web[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Web Control & PID Monitor</title>
  <script src="https://cdn.jsdelivr.net/npm/echarts@5.4.3/dist/echarts.min.js"></script>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 10px; background-color: #f5f5f5; }
    
    h2{
        width: auto;
        height: 60px;
        line-height: 60px;
        text-align: center;
        font-family: 等线;
        color: white;
        background-color:cornflowerblue;
        border-radius: 12px;
        margin: 10px 0;
    }
    
    .container {
        max-width: 1200px;
        margin: 0 auto;
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 20px;
    }
    
    .control-panel, .monitor-panel {
        background: white;
        border-radius: 12px;
        padding: 20px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    }
    
    .section-title {
        font-size: 18px;
        font-weight: bold;
        color: cornflowerblue;
        margin-bottom: 15px;
        border-bottom: 2px solid cornflowerblue;
        padding-bottom: 5px;
    }
    
    input{
        width: 160px;
        height: 30px;
        margin: 5px;
        border: 1px solid #ddd;
        border-radius: 5px;
        padding: 0 10px;
    }
    
    .pid-input {
        display: flex;
        align-items: center;
        margin: 10px 0;
    }
    
    .pid-input label {
        width: 120px;
        font-weight: bold;
    }
    
    .pid-input input {
        width: 100px;
        margin-right: 10px;
    }
    
    .sliderLabel{
        display: block;
        text-align: left;
        line-height: 30px;
        height: 30px;  
        width: 100%;
        margin: 5px 0;
    }

    .sliders{
        margin: 10px 0;
    }
    
    .slider-container {
        margin: 15px 0;
    }
    
    .slider-container input[type="range"] {
        width: 100%;
        margin: 5px 0;
    }

    .view2 {
        text-align: center;
        margin: 15px 0;
    }
    
    .buttons{
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 10px;
        margin: 20px 0;
        max-width: 300px;
    }
    
    .dir{
        font-size: 15px;
        width: 100%;
        height: 50px;
        text-align: center;
        border-radius: 12px;
        background-color: white;
        color: cornflowerblue;
        border: 3px solid cornflowerblue;
        transition: all 0.3s;
        cursor: pointer;
    }
    
    .dir:hover {
        background-color: #f0f8ff;
    }
    
    .row {
        display: flex;
        justify-content: center;
        margin: 20px 0;
    }
    
    .columnLateral {
        text-align: center;
    }

    #joystick {
        border: 2px solid cornflowerblue;
        border-radius: 50%;
    }
    
    /* Switch开关样式 */
    input[type='checkbox'].switch{
        outline: none;
        appearance: none;
        -webkit-appearance: none;
        -moz-appearance: none;
        position: relative;
        width: 40px;
        height: 20px;
        background: #ccc;
        border-radius: 10px;
        transition: border-color .3s, background-color .3s;
        margin: 0px 20px 0px 0px;
    }

    input[type='checkbox'].switch::after {
        content: '';
        display: inline-block;
        width: 1rem;
        height:1rem;
        border-radius: 50%;
        background: #fff;
        box-shadow: 0 0 2px #999;
        transition:.4s;
        top: 2px;
        position: absolute;
        left: 2px;
    }

    input[type='checkbox'].switch:checked {
        background: rgb(78, 78, 240);
    }
    
    input[type='checkbox'].switch:checked::after {
        content: '';
        position: absolute;
        left: 55%;
        top: 2px;
    }
    
    .chart-container {
        position: relative;
        height: 300px;
        margin: 20px 0;
    }
    
    .btn-group {
        display: flex;
        gap: 10px;
        margin: 15px 0;
        flex-wrap: wrap;
    }
    
    .btn {
        padding: 8px 16px;
        border: 2px solid cornflowerblue;
        background: white;
        color: cornflowerblue;
        border-radius: 8px;
        cursor: pointer;
        font-size: 14px;
        transition: all 0.3s;
    }
    
    .btn:hover {
        background: cornflowerblue;
        color: white;
    }
    
    .status-indicator {
        display: inline-block;
        width: 12px;
        height: 12px;
        border-radius: 50%;
        margin-right: 8px;
    }
    
    .status-connected { background-color: #4CAF50; }
    .status-disconnected { background-color: #f44336; }
    
    /* 通知样式 */
    .notification {
        position: fixed;
        top: 20px;
        right: 20px;
        padding: 12px 24px;
        border-radius: 8px;
        color: white;
        font-weight: bold;
        z-index: 1000;
        opacity: 0;
        transform: translateX(100%);
        transition: all 0.3s ease-in-out;
    }
    
    .notification.show {
        opacity: 1;
        transform: translateX(0);
    }
    
    .notification.success {
        background-color: #4CAF50;
    }
    
    .notification.error {
        background-color: #f44336;
    }
    
    .notification.info {
        background-color: #2196F3;
    }
    
    *{
        -webkit-touch-callout:none; 
        -webkit-user-select:none; 
        -khtml-user-select:none; 
        -moz-user-select:none;
        -ms-user-select:none; 
        user-select:none;
    }
        
  </style>
</head>

<body onload="javascript:init()">
    <h2>🤖 Robot Web Control & PID Monitor</h2>
    
    <!-- 连接状态显示 -->
    <div style="text-align: center; margin-bottom: 20px;">
        <span id="connectionStatus" class="status-indicator status-disconnected"></span>
        <span id="connectionText">未连接</span>
    </div>
    
    <div class="container">
        <!-- 控制面板 -->
        <div class="control-panel">
            
            <!-- PID参数控制 -->
            <div class="section-title">⚙️ PID参数设置</div>
            
            <div class="pid-input">
                <label for="vel_kp">vel_kp:</label>
                <input type="number" id="vel_kp" value="1.0" step="0.1" onchange="updatePID()">
            </div>
            <div class="pid-input">
                <label for="balance_kp">balance_kp:</label>
                <input type="number" id="balance_kp" value="50.0" step="1.0" onchange="updatePID()">
            </div>
            <div class="pid-input">
                <label for="balance_kd">balance_kd:</label>
                <input type="number" id="balance_kd" value="0.8" step="0.1" onchange="updatePID()">
            </div>
            <div class="pid-input">
                <label for="robot_kp">robot_kp:</label>
                <input type="number" id="robot_kp" value="10.0" step="0.5" onchange="updatePID()">
            </div>
            
            <div class="btn-group">
                <button class="btn" onclick="sendPIDParams()">发送PID参数</button>
                <button class="btn" onclick="resetPIDParams()">重置参数</button>
            </div>
        </div>
        
        <!-- 监控面板 -->
        <div class="monitor-panel">
            <div class="section-title">📈 PID曲线监控</div>
            
            <div class="btn-group">
                <button class="btn" onclick="startMonitoring()">开始监控</button>
                <button class="btn" onclick="stopMonitoring()">停止监控</button>
                <button class="btn" onclick="clearChart()">清除数据</button>
                <button class="btn" onclick="resetZoom()">重置缩放</button>
                <button class="btn" onclick="toggleLegend()">切换图例</button>
            </div>
            
            <!-- 数据窗口控制 -->
            <div class="control-group">
                <label>数据窗口大小:</label>
                <select id="dataWindowSelect" onchange="changeDataWindow()">
                    <option value="500">500点</option>
                    <option value="1000" selected>1000点</option>
                    <option value="2000">2000点</option>
                    <option value="5000">5000点</option>
                </select>
                
                <label style="margin-left: 20px;">显示时间范围:</label>
                <select id="displayTimeSelect" onchange="changeDisplayTime()">
                    <option value="10">最近10秒</option>
                    <option value="30" selected>最近30秒</option>
                    <option value="60">最近60秒</option>
                    <option value="0">全部数据</option>
                </select>
            </div>
            
            <!-- PID曲线图 -->
            <div class="chart-container">
                <div id="pidChart" style="width: 100%; height: 400px;"></div>
            </div>
            
            <!-- 实时数据显示 -->
            <div class="section-title" style="margin-top: 120px;">📊 实时数据</div>
            <div id="realTimeData">
                <div>vel_kp输出: <span id="vel_output">0</span></div>
                <div>balance_kp输出: <span id="balance_output">0</span></div>
                <div>balance_kd输出: <span id="balance_kd_output">0</span></div>
                <div>robot_kp输出: <span id="robot_output">0</span></div>
                <div>数据点数: <span id="dataCount">0</span></div>
                <div>采样率: <span id="sampleRate">0</span> Hz</div>
            </div>
        </div>
    </div>

    <script>
        var socket; // WebSocket连接
        var g_roll=0, g_h=38; 
        var g_linear = 0, g_angular = 0, g_stable = 0; 
        var joyX = 0, joyY = 0;
        
        // PID参数
        var pidParams = {
            vel_kp: 1.0,
            balance_kp: 50.0,
            balance_kd: 0.8,
            robot_kp: 10.0
        };
        
        // ECharts图表相关
        var pidChart;
        var chartData = {
            timestamps: [],
            vel_output: [],
            balance_output: [],
            balance_kd_output: [],
            robot_output: []
        };
        
        var isMonitoring = false;
        var dataWindow = 1000; // 增加到1000个数据点
        var displayTimeRange = 30; // 显示时间范围（秒）
        var sampleCount = 0;
        var lastSampleTime = 0;
        var startTime = 0;
        var legendVisible = true;
        
        // 初始化函数
        function init() {
            socket_init();
            initChart();
            updatePIDInputs();
        }
        
        // WebSocket初始化
        function socket_init() {
            try {
                socket = new WebSocket('ws://' + window.location.hostname + ':81/');
                
                socket.onopen = function(event) {
                    console.log('WebSocket连接成功');
                    updateConnectionStatus(true);
                };
                
                socket.onmessage = function(event) {
                    try {
                        var data = JSON.parse(event.data);
                        handleServerData(data);
                    } catch (e) {
                        console.error('解析服务器数据失败:', e);
                    }
                };
                
                socket.onclose = function(event) {
                    console.log('WebSocket连接关闭');
                    updateConnectionStatus(false);
                };
                
                socket.onerror = function(event) {
                    console.error('WebSocket错误:', event);
                    updateConnectionStatus(false);
                };
            } catch (e) {
                console.error('WebSocket初始化失败:', e);
                updateConnectionStatus(false);
            }
        }
        
        // 更新连接状态
        function updateConnectionStatus(connected) {
            var statusIndicator = document.getElementById('connectionStatus');
            var statusText = document.getElementById('connectionText');
            
            if (connected) {
                statusIndicator.className = 'status-indicator status-connected';
                statusText.textContent = '已连接';
            } else {
                statusIndicator.className = 'status-indicator status-disconnected';
                statusText.textContent = '未连接';
            }
        }
        
        // 显示通知消息
        function showNotification(message, type = 'info', duration = 3000) {
            // 创建通知元素
            var notification = document.createElement('div');
            notification.className = 'notification ' + type;
            notification.textContent = message;
            
            // 添加到页面
            document.body.appendChild(notification);
            
            // 显示动画
            setTimeout(function() {
                notification.classList.add('show');
            }, 100);
            
            // 自动隐藏
            setTimeout(function() {
                notification.classList.remove('show');
                setTimeout(function() {
                    if (notification.parentNode) {
                        notification.parentNode.removeChild(notification);
                    }
                }, 300);
            }, duration);
        }
        
        // 初始化ECharts图表
        function initChart() {
            var chartDom = document.getElementById('pidChart');
            pidChart = echarts.init(chartDom);
            
            var option = {
                title: {
                    text: 'PID控制器实时输出曲线',
                    left: 'center'
                },
                tooltip: {
                    trigger: 'axis',
                    axisPointer: {
                        type: 'cross'
                    }
                },
                legend: {
                    data: ['vel_kp输出', 'balance_kp输出', 'balance_kd输出', 'robot_kp输出'],
                    top: 30
                },
                grid: {
                    left: '3%',
                    right: '4%',
                    bottom: '15%',
                    containLabel: true
                },
                toolbox: {
                    feature: {
                        dataZoom: {
                            yAxisIndex: 'none'
                        },
                        restore: {},
                        saveAsImage: {}
                    }
                },
                xAxis: {
                    type: 'value',
                    name: '时间 (s)',
                    nameLocation: 'middle',
                    nameGap: 30,
                    boundaryGap: false,
                    axisLabel: {
                        formatter: function (value) {
                            return value.toFixed(1);
                        }
                    }
                },
                yAxis: {
                    type: 'value',
                    name: 'PID输出值',
                    nameLocation: 'middle',
                    nameGap: 50
                },
                dataZoom: [
                    {
                        type: 'inside',
                        start: 70,
                        end: 100
                    },
                    {
                        start: 70,
                        end: 100,
                        handleIcon: 'M10.7,11.9v-1.3H9.3v1.3c-4.9,0.3-8.8,4.4-8.8,9.4c0,5,3.9,9.1,8.8,9.4v1.3h1.3v-1.3c4.9-0.3,8.8-4.4,8.8-9.4C19.5,16.3,15.6,12.2,10.7,11.9z M13.3,24.4H6.7V23.1h6.6V24.4z M13.3,19.6H6.7v-1.4h6.6V19.6z',
                        handleSize: '80%',
                        handleStyle: {
                            color: '#fff',
                            shadowBlur: 3,
                            shadowColor: 'rgba(0, 0, 0, 0.6)',
                            shadowOffsetX: 2,
                            shadowOffsetY: 2
                        }
                    }
                ],
                series: [
                    {
                        name: 'vel_kp输出',
                        type: 'line',
                        data: [],
                        smooth: true,
                        symbol: 'none',
                        lineStyle: {
                            color: '#ff6384',
                            width: 2
                        }
                    },
                    {
                        name: 'balance_kp输出',
                        type: 'line',
                        data: [],
                        smooth: true,
                        symbol: 'none',
                        lineStyle: {
                            color: '#36a2eb',
                            width: 2
                        }
                    },
                    {
                        name: 'balance_kd输出',
                        type: 'line',
                        data: [],
                        smooth: true,
                        symbol: 'none',
                        lineStyle: {
                            color: '#ffcd56',
                            width: 2
                        }
                    },
                    {
                        name: 'robot_kp输出',
                        type: 'line',
                        data: [],
                        smooth: true,
                        symbol: 'none',
                        lineStyle: {
                            color: '#4bc0c0',
                            width: 2
                        }
                    }
                ]
            };
            
            pidChart.setOption(option);
            
            // 响应式调整
            window.addEventListener('resize', function () {
                pidChart.resize();
            });
        }
        
        // 处理服务器数据
        function handleServerData(data) {
            if (data.type === 'pid_data' && isMonitoring) {
                updateChart(data);
                updateRealTimeDisplay(data);
            } else if (data.type === 'pid_confirm') {
                // 处理PID参数确认消息
                console.log('收到PID参数确认:', data);
                // 更新本地PID参数显示
                pidParams.vel_kp = data.vel_kp;
                pidParams.balance_kp = data.balance_kp;
                pidParams.balance_kd = data.balance_kd;
                pidParams.robot_kp = data.robot_kp;
                updatePIDInputs();
                
                // 显示成功提示
                showNotification('PID参数设置成功！', 'success');
            }
        }
        
        // 更新ECharts图表
        function updateChart(data) {
            if (startTime === 0) {
                startTime = Date.now();
            }
            
            var currentTime = (Date.now() - startTime) / 1000; // 相对时间（秒）
            
            // 添加新数据点
            chartData.timestamps.push(currentTime);
            chartData.vel_output.push([currentTime, data.vel_output || 0]);
            chartData.balance_output.push([currentTime, data.balance_output || 0]);
            chartData.balance_kd_output.push([currentTime, data.balance_kd_output || 0]);
            chartData.robot_output.push([currentTime, data.robot_output || 0]);
            
            // 限制数据窗口大小，防止堆积
            if (chartData.timestamps.length > dataWindow) {
                chartData.timestamps.shift();
                chartData.vel_output.shift();
                chartData.balance_output.shift();
                chartData.balance_kd_output.shift();
                chartData.robot_output.shift();
            }
            
            // 更新图表数据
            pidChart.setOption({
                series: [
                    {
                        data: chartData.vel_output
                    },
                    {
                        data: chartData.balance_output
                    },
                    {
                        data: chartData.balance_kd_output
                    },
                    {
                        data: chartData.robot_output
                    }
                ]
            });
            
            // 自动调整数据缩放范围，显示最新的数据
            if (chartData.timestamps.length > 50 && displayTimeRange > 0) {
                var totalTime = currentTime;
                var displayTime = Math.min(displayTimeRange, totalTime);
                var startPercent = Math.max(0, (totalTime - displayTime) / totalTime * 100);
                
                pidChart.setOption({
                    dataZoom: [
                        {
                            type: 'inside',
                            start: startPercent,
                            end: 100
                        },
                        {
                            start: startPercent,
                            end: 100
                        }
                    ]
                });
            }
            
            // 计算采样率
            sampleCount++;
            if (lastSampleTime === 0) {
                lastSampleTime = Date.now() / 1000;
            } else if (Date.now() / 1000 - lastSampleTime >= 1) {
                var sampleRate = sampleCount / (Date.now() / 1000 - lastSampleTime);
                document.getElementById('sampleRate').textContent = sampleRate.toFixed(1);
                sampleCount = 0;
                lastSampleTime = Date.now() / 1000;
            }
        }
        
        // 更新实时数据显示
        function updateRealTimeDisplay(data) {
            document.getElementById('vel_output').textContent = (data.vel_output || 0).toFixed(3);
            document.getElementById('balance_output').textContent = (data.balance_output || 0).toFixed(3);
            document.getElementById('balance_kd_output').textContent = (data.balance_kd_output || 0).toFixed(3);
            document.getElementById('robot_output').textContent = (data.robot_output || 0).toFixed(3);
            document.getElementById('dataCount').textContent = chartData.timestamps.length;
        }
        
        // 开始监控
        function startMonitoring() {
            isMonitoring = true;
            console.log('开始PID监控');
            send_data('start_monitoring');
        }
        
        // 停止监控
        function stopMonitoring() {
            isMonitoring = false;
            console.log('停止PID监控');
            send_data('stop_monitoring');
        }
        
        // 清除图表数据
        function clearChart() {
            chartData.timestamps = [];
            chartData.vel_output = [];
            chartData.balance_output = [];
            chartData.balance_kd_output = [];
            chartData.robot_output = [];
            
            pidChart.setOption({
                series: [
                    { data: [] },
                    { data: [] },
                    { data: [] },
                    { data: [] }
                ]
            });
            
            sampleCount = 0;
            lastSampleTime = 0;
            startTime = 0;
            document.getElementById('dataCount').textContent = '0';
            document.getElementById('sampleRate').textContent = '0';
        }
        
        // 重置缩放
        function resetZoom() {
            pidChart.setOption({
                dataZoom: [
                    {
                        type: 'inside',
                        start: 0,
                        end: 100
                    },
                    {
                        start: 0,
                        end: 100
                    }
                ]
            });
        }
        
        // 切换图例显示
        function toggleLegend() {
            legendVisible = !legendVisible;
            pidChart.setOption({
                legend: {
                    show: legendVisible
                }
            });
        }
        
        // 改变数据窗口大小
        function changeDataWindow() {
            var select = document.getElementById('dataWindowSelect');
            dataWindow = parseInt(select.value);
            console.log('数据窗口大小改为:', dataWindow);
        }
        
        // 改变显示时间范围
        function changeDisplayTime() {
            var select = document.getElementById('displayTimeSelect');
            displayTimeRange = parseInt(select.value);
            console.log('显示时间范围改为:', displayTimeRange === 0 ? '全部数据' : displayTimeRange + '秒');
        }
        
        // 更新PID参数
        function updatePID() {
            pidParams.vel_kp = parseFloat(document.getElementById('vel_kp').value);
            pidParams.balance_kp = parseFloat(document.getElementById('balance_kp').value);
            pidParams.balance_kd = parseFloat(document.getElementById('balance_kd').value);
            pidParams.robot_kp = parseFloat(document.getElementById('robot_kp').value);
        }
        
        // 发送PID参数
        function sendPIDParams() {
            updatePID();
            var data = {
                type: 'pid_params',
                ...pidParams,
                mode: 'basic'
            };
            socket.send(JSON.stringify(data));
            console.log('发送PID参数:', pidParams);
        }
        
        // 重置PID参数
        function resetPIDParams() {
            document.getElementById('vel_kp').value = 1.0;
            document.getElementById('balance_kp').value = 50.0;
            document.getElementById('balance_kd').value = 0.8;
            document.getElementById('robot_kp').value = 10.0;
            updatePID();
            sendPIDParams();
        }
        
        // 更新PID输入框
        function updatePIDInputs() {
            document.getElementById('vel_kp').value = pidParams.vel_kp;
            document.getElementById('balance_kp').value = pidParams.balance_kp;
            document.getElementById('balance_kd').value = pidParams.balance_kd;
            document.getElementById('robot_kp').value = pidParams.robot_kp;
        }
        
        // 原有的控制函数
        function setroll() {
            var val = parseInt(document.getElementById("rollSlider").value);
            document.getElementById("rollLabel").innerHTML = "Roll: " + val + "°";
            g_roll = val;
            send_data('control');
        }
        
        function setHeight() {
            var val = parseInt(document.getElementById("hSlider").value);
            document.getElementById("hLabel").innerHTML = "BaseHeight: " + val + "mm";
            g_h = val;
            send_data('control');
        }
        
        function setLinear() {
            var val = parseInt(document.getElementById("linearSlider").value);
            document.getElementById("linearLabel").innerHTML = "LinearVel: " + val + "mm/s";
            g_linear = val;
            send_data('control');
        }
        
        function setAngular() {
            var val = parseInt(document.getElementById("angularSlider").value);
            document.getElementById("angularLabel").innerHTML = "AngularVel: " + val + "°/s";
            g_angular = val;
            send_data('control');
        }
        
        function is_stable() {
            var obj = document.getElementById("stable");
            g_stable = obj.checked ? 1 : 0;
            send_data('control');
        }
        
        // 发送数据
        function send_data(type) {
            if (socket && socket.readyState === WebSocket.OPEN) {
                var data = {
                    type: type || 'control',
                    roll: g_roll,
                    height: g_h,
                    linear: g_linear,
                    angular: g_angular,
                    stable: g_stable,
                    mode: 'basic',
                    dir: "stop",
                    joy_y: joyY,
                    joy_x: joyX
                };
                socket.send(JSON.stringify(data));
            } else {
                console.warn('WebSocket未连接，无法发送数据');
            }
        }
        
        // 方向按钮事件
        var buttons = document.getElementsByClassName("dir");
        for(var i=0; i<buttons.length; i++) {
            buttons[i].addEventListener("mousedown", move, true);
            buttons[i].addEventListener("mouseup", stop, true);
            buttons[i].addEventListener("touchstart", move, true);
            buttons[i].addEventListener("touchend", stop, true);
        }
        
        function move() {
            this.style.backgroundColor = "cornflowerblue";
            this.style.color = "white";
            if (socket && socket.readyState === WebSocket.OPEN) {
                var data = {
                    type: 'control',
                    dir: this.id,
                    mode: 'basic',
                    roll: g_roll,
                    height: g_h,
                    linear: g_linear,
                    angular: g_angular,
                    stable: g_stable,
                    joy_x: joyX,
                    joy_y: joyY
                };
                socket.send(JSON.stringify(data));
            }
        }
        
        function stop() {
            this.style.backgroundColor = "white";
            this.style.color = "cornflowerblue";
            if (socket && socket.readyState === WebSocket.OPEN) {
                var data = {
                    type: 'control',
                    dir: "stop",
                    mode: 'basic',
                    roll: g_roll,
                    height: g_h,
                    linear: g_linear,
                    angular: g_angular,
                    stable: g_stable,
                    joy_x: joyX,
                    joy_y: joyY
                };
                socket.send(JSON.stringify(data));
            }
        }
        
        // 摇杆代码（简化版）
        var JoyStick = (function(container, parameters) {
            parameters = parameters || {};
            var title = parameters.title || "joystick";
            var width = parameters.width || 0;
            var height = parameters.height || 0;
            var internalFillColor = parameters.internalFillColor || "#00979C";
            var internalStrokeColor = parameters.internalStrokeColor || "#00979C";
            var externalStrokeColor = parameters.externalStrokeColor || "#0097BC";
            var autoReturnToCenter = parameters.autoReturnToCenter !== undefined ? parameters.autoReturnToCenter : true;
            
            var objContainer = document.getElementById(container);
            var canvas = document.createElement("canvas");
            canvas.id = title;
            if(width === 0) width = objContainer.clientWidth;
            if(height === 0) height = objContainer.clientHeight;
            canvas.width = width;
            canvas.height = height;
            objContainer.appendChild(canvas);
            var context = canvas.getContext("2d");
            
            var isPressing = false;
            var centerX = canvas.width / 2;
            var centerY = canvas.height / 2;
            var internalRadius = Math.min(canvas.width, canvas.height) / 6;
            var externalRadius = Math.min(canvas.width, canvas.height) / 3;
            var maxMoveStick = externalRadius - internalRadius;
            var movedX = centerX;
            var movedY = centerY;
            
            function drawExternal() {
                context.beginPath();
                context.arc(centerX, centerY, externalRadius, 0, 2 * Math.PI, false);
                context.lineWidth = 2;
                context.strokeStyle = externalStrokeColor;
                context.stroke();
            }
            
            function drawInternal() {
                context.beginPath();
                context.arc(movedX, movedY, internalRadius, 0, 2 * Math.PI, false);
                context.fillStyle = internalFillColor;
                context.fill();
                context.lineWidth = 2;
                context.strokeStyle = internalStrokeColor;
                context.stroke();
            }
            
            function redraw() {
                context.clearRect(0, 0, canvas.width, canvas.height);
                drawExternal();
                drawInternal();
            }
            
            function updatePosition(x, y) {
                var dx = x - centerX;
                var dy = y - centerY;
                var distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance <= maxMoveStick) {
                    movedX = x;
                    movedY = y;
                } else {
                    movedX = centerX + (dx / distance) * maxMoveStick;
                    movedY = centerY + (dy / distance) * maxMoveStick;
                }
                
                joyX = Math.round(100 * (movedX - centerX) / maxMoveStick);
                joyY = Math.round(-100 * (movedY - centerY) / maxMoveStick);
                
                redraw();
                send_data('control');
            }
            
            canvas.addEventListener("mousedown", function(event) {
                isPressing = true;
                var rect = canvas.getBoundingClientRect();
                updatePosition(event.clientX - rect.left, event.clientY - rect.top);
            });
            
            canvas.addEventListener("mousemove", function(event) {
                if (isPressing) {
                    var rect = canvas.getBoundingClientRect();
                    updatePosition(event.clientX - rect.left, event.clientY - rect.top);
                }
            });
            
            document.addEventListener("mouseup", function() {
                if (isPressing) {
                    isPressing = false;
                    if (autoReturnToCenter) {
                        movedX = centerX;
                        movedY = centerY;
                        joyX = 0;
                        joyY = 0;
                        redraw();
                        send_data('control');
                    }
                }
            });
            
            // 触摸事件
            canvas.addEventListener("touchstart", function(event) {
                event.preventDefault();
                isPressing = true;
                var rect = canvas.getBoundingClientRect();
                var touch = event.touches[0];
                updatePosition(touch.clientX - rect.left, touch.clientY - rect.top);
            });
            
            canvas.addEventListener("touchmove", function(event) {
                event.preventDefault();
                if (isPressing) {
                    var rect = canvas.getBoundingClientRect();
                    var touch = event.touches[0];
                    updatePosition(touch.clientX - rect.left, touch.clientY - rect.top);
                }
            });
            
            document.addEventListener("touchend", function(event) {
                if (isPressing) {
                    isPressing = false;
                    if (autoReturnToCenter) {
                        movedX = centerX;
                        movedY = centerY;
                        joyX = 0;
                        joyY = 0;
                        redraw();
                        send_data('control');
                    }
                }
            });
            
            redraw();
            
            return {
                GetX: function() {
                    return joyX;
                },
                GetY: function() {
                    return joyY;
                }
            };
        });
        
        // 初始化摇杆
        var joy1Param = { "title": "joystick1" };
        var Joy1 = new JoyStick('joy1Div', joy1Param);
        
    </script>
</body>
</html> 

)=====";