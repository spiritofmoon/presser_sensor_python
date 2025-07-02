import socket
import struct
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import datetime
import os
import numpy as np
from matplotlib import gridspec

# 服务器配置
SERVER_HOST = "127.0.0.1"  # 监听所有接口
# SERVER_HOST = "0.0.0.0"  # 监听所有接口
SERVER_PORT = 5000
BUFFER_SIZE = 8192

# 绘图配置
PLOT_WINDOW_SIZE = 200  # 显示的数据点数（与客户端缓冲区大小一致）
PLOT_REFRESH_INTERVAL = 100  # 图表刷新间隔(毫秒)
PRESSURE_OFFSET = 101325.0  # 海平面标准大气压(Pa)

# 数据存储配置 - 通过设置此标志位控制是否保存数据
SAVE_DATA = True  # 设置True保存数据，False不保存
DATA_FILE = "sensor_data.txt"

# 全局变量用于存储传感器数据
sensor1_data = deque(maxlen=PLOT_WINDOW_SIZE)
sensor2_data = deque(maxlen=PLOT_WINDOW_SIZE)
timestamps = deque(maxlen=PLOT_WINDOW_SIZE)
latest_values = {"Sensor1": {"pressure": 0, "change": 0}, "Sensor2": {"pressure": 0, "change": 0}}

# 用于线程同步
data_lock = threading.Lock()
running = True


def parse_data_packet(data):
    """
    解析来自客户端的数据包
    数据包格式:
        [4字节: 总数据点数N]
        [N × (1字节传感器ID + 4字节时间差 + 4字节温度 + 4字节气压)]
    """
    if len(data) < 4:
        return []

    num_points = struct.unpack("!I", data[:4])[0]
    points = []
    offset = 4
    point_size = 13  # 1 + 4 + 4 + 4

    for _ in range(num_points):
        if offset + point_size > len(data):
            break

        sensor_id = struct.unpack("!B", data[offset:offset + 1])[0]
        time_diff = struct.unpack("!I", data[offset + 1:offset + 5])[0]
        temperature = struct.unpack("!f", data[offset + 5:offset + 9])[0]
        pressure = struct.unpack("!f", data[offset + 9:offset + 13])[0]

        points.append((sensor_id, time_diff, temperature, pressure))
        offset += point_size

    return points


def save_data_to_file(sensor_id, time_diff, pressure):
    """将数据保存到文件"""
    if not SAVE_DATA:
        return

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    with open(DATA_FILE, "a") as f:
        f.write(f"{timestamp},{sensor_id},{time_diff},{pressure}\n")


def handle_client(conn, addr):
    """处理客户端连接"""
    print(f"Connection from: {addr}")

    try:
        while running:
            # 接收数据
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break

            # 解析数据包
            points = parse_data_packet(data)

            # 处理每个数据点
            with data_lock:
                for sensor_id, time_diff, temperature, pressure in points:
                    # 保存原始数据
                    if SAVE_DATA:
                        save_data_to_file(sensor_id, time_diff, pressure)

                    # 保存用于绘图的数据
                    pressure_change = pressure - PRESSURE_OFFSET

                    # 只存储时间戳一次，确保所有传感器使用相同的时间序列
                    if not timestamps or timestamps[-1] != time_diff:
                        timestamps.append(time_diff)

                    if sensor_id == 1:
                        sensor1_data.append(pressure_change)
                        # 更新最新值
                        latest_values["Sensor1"]["pressure"] = pressure
                        latest_values["Sensor1"]["change"] = pressure_change
                    elif sensor_id == 2:
                        sensor2_data.append(pressure_change)
                        latest_values["Sensor2"]["pressure"] = pressure
                        latest_values["Sensor2"]["change"] = pressure_change

    except Exception as e:
        print(f"Error handling client {addr}: {e}")
    finally:
        conn.close()
        print(f"Connection closed with {addr}")


def update_plot(frame):
    """更新图表数据"""
    with data_lock:
        ax.clear()

        # 确保绘图数据长度一致
        min_length = min(len(timestamps), len(sensor1_data), len(sensor2_data))

        if min_length > 0:
            # 绘制传感器1数据 (蓝色)
            ax.plot(list(timestamps)[-min_length:],
                    list(sensor1_data)[-min_length:],
                    'b-', label='Sensor 1', linewidth=1.5)

            # 绘制传感器2数据 (红色)
            ax.plot(list(timestamps)[-min_length:],
                    list(sensor2_data)[-min_length:],
                    'r-', label='Sensor 2', linewidth=1.5)

            # 设置图表属性
            ax.set_title(f"Dual-Channel Pressure Sensor Real-time Data")
            ax.set_xlabel("Time (ms)")
            ax.set_ylabel("Pressure Change (Pa)")
            ax.grid(True, linestyle='--', alpha=0.7)
            ax.legend(loc='upper right')

            # 自动调整坐标轴范围
            if timestamps:
                ax.set_xlim(min(timestamps), max(timestamps))

            if sensor1_data or sensor2_data:
                all_data = list(sensor1_data)[-min_length:] + list(sensor2_data)[-min_length:]
                min_val = min(all_data)
                max_val = max(all_data)
                padding = max(0.1 * (max_val - min_val), 1.0)  # 至少1Pa的边距
                ax.set_ylim(min_val - padding, max_val + padding)

            # 在图表下方添加当前数值显示
            s1_pressure = latest_values["Sensor1"]["pressure"]
            s1_change = latest_values["Sensor1"]["change"]
            s2_pressure = latest_values["Sensor2"]["pressure"]
            s2_change = latest_values["Sensor2"]["change"]

            # 添加数据保存状态指示
            save_status = "SAVING" if SAVE_DATA else "NOT SAVING"
            status_color = "green" if SAVE_DATA else "red"

            info_text = (f"Sensor 1: {s1_pressure:.2f} Pa (Change: {s1_change:.2f} Pa)\n"
                         f"Sensor 2: {s2_pressure:.2f} Pa (Change: {s2_change:.2f} Pa)\n"
                         f"Data Status: [{save_status}]")

            ax.text(0.5, -0.2, info_text, transform=ax.transAxes,
                    fontsize=10, ha='center', va='top',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray', boxstyle='round,pad=0.5'),
                    color=status_color)

    return ax,


def start_server():
    """启动TCP服务器"""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(1)
    server.settimeout(1.0)  # 设置超时以便定期检查运行状态

    print(f"Server started, listening on {SERVER_HOST}:{SERVER_PORT}")

    # 如果设置了保存数据，初始化数据文件
    if SAVE_DATA and not os.path.exists(DATA_FILE):
        with open(DATA_FILE, "w") as f:
            f.write("timestamp,sensor_id,time_diff,pressure\n")

    try:
        while running:
            try:
                conn, addr = server.accept()
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
            except socket.timeout:
                continue  # 超时正常，继续检查运行状态
            except Exception as e:
                print(f"Server error: {e}")
                if not running:
                    break
    finally:
        server.close()
        print("Server closed")


def on_close(event):
    """窗口关闭事件处理"""
    global running
    running = False
    print("Closing program...")


if __name__ == "__main__":
    # 创建主图形和子图布局
    plt.style.use('ggplot')
    fig, ax = plt.subplots(figsize=(12, 7))

    # 连接关闭事件
    fig.canvas.mpl_connect('close_event', on_close)

    # 启动服务器线程
    server_thread = threading.Thread(target=start_server)
    server_thread.daemon = True
    server_thread.start()

    # 修复FuncAnimation警告
    # 设置一个较大的save_count值以避免缓存问题
    ani = FuncAnimation(
        fig,
        update_plot,
        interval=PLOT_REFRESH_INTERVAL,
        blit=True,
        save_count=1000  # 设置足够大的save_count值
    )

    # 添加一个空的plot以初始化图表
    if not sensor1_data:
        ax.plot([], [], 'b-', label='Sensor 1')
        ax.plot([], [], 'r-', label='Sensor 2')
        ax.set_title("Waiting for sensor data...")
        ax.set_xlabel("Time (ms)")
        ax.set_ylabel("Pressure Change (Pa)")
        ax.legend(loc='upper right')
        ax.grid(True)

    plt.tight_layout()
    plt.show()

    # 等待所有线程结束
    running = False
    server_thread.join(timeout=5.0)
    print("Program exited")