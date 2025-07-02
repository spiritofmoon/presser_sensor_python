import socket
import struct
import csv
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import matplotlib
matplotlib.use('TkAgg')

# Configuration
SERVER_IP = "0.0.0.0"  # Listen on all interfaces
SERVER_PORT = 5000
SAVE_DATA = True  # Set to True to save data to CSV
SAVE_FILE = "sensor_data.csv"
PLOT_WINDOW_SIZE = 10000  # 10 seconds of data at 100Hz
BUFFER_SIZE = 1024 * 8

# Data buffers
sensor1_data = deque(maxlen=PLOT_WINDOW_SIZE)
sensor2_data = deque(maxlen=PLOT_WINDOW_SIZE)
diff_pressure = deque(maxlen=PLOT_WINDOW_SIZE)  # 存储压力差值
diff_timestamp = deque(maxlen=PLOT_WINDOW_SIZE)  # 存储时间戳差值*10
last_packet_stats = {
    'sensor1': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0},
    'sensor2': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0}
}
lock = threading.Lock()

# Setup plot with dual axes
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 9
fig, ax1 = plt.subplots(figsize=(14, 8))
fig.canvas.manager.set_window_title('Dual-Channel Pressure Sensor Monitor')

# 主Y轴（左侧）用于原始传感器数据
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Pressure (hPa)', color='blue')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.tick_params(axis='y', labelcolor='blue')

# 创建第二个Y轴（右侧）用于差值数据
ax2 = ax1.twinx()
ax2.set_ylabel('Pressure Diff (S1-S2) hPa / Time Diff (x10)', color='green')
ax2.tick_params(axis='y', labelcolor='green')

# 创建第三个Y轴（右侧）用于时间戳差值
ax3 = ax1.twinx()
ax3.set_ylabel('Timestamp Diff (x10)', color='purple')
ax3.tick_params(axis='y', labelcolor='purple')
ax3.spines['right'].set_position(('outward', 60))  # 将第三个轴稍微移开

# 绘制传感器数据线
sensor1_line, = ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)

# 绘制差值线
diff_pressure_line, = ax2.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
diff_timestamp_line, = ax3.plot([], [], 'm-', label='Timestamp Diff (x10)', alpha=0.8)

# 合并图例
lines = [sensor1_line, sensor2_line, diff_pressure_line, diff_timestamp_line]
ax1.legend(lines, [l.get_label() for l in lines], loc='upper left')

# 添加统计信息文本
stats_text = ax1.text(0.98, 0.95, '', transform=ax1.transAxes,
                     verticalalignment='top', horizontalalignment='right',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# 设置标题
ax1.set_title('Real-time Pressure Monitoring with Differences')

# 设置初始范围
ax1.set_ylim(-10, 10)
ax2.set_ylim(-10, 10)
ax3.set_ylim(-10, 10)
ax1.set_xlim(0, 10000)

# Create CSV file if saving is enabled
if SAVE_DATA:
    with open(SAVE_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'sensor_id', 'pressure'])


def handle_client(conn, addr):
    print(f"Connection from {addr}")
    while True:
        try:
            # Read packet header (4 bytes - number of points)
            header = conn.recv(4)
            if not header:
                break

            num_points = struct.unpack("!I", header)[0]
            print(f"Receiving {num_points} points")

            # Read all data points
            data = b''
            while len(data) < num_points * 13:  # Each point is 13 bytes
                chunk = conn.recv(BUFFER_SIZE)
                if not chunk:
                    break
                data += chunk

            if len(data) < num_points * 13:
                print(f"Incomplete data received: {len(data)} bytes")
                break

            process_data(data, num_points)

        except (ConnectionResetError, BrokenPipeError):
            print(f"Client {addr} disconnected")
            break
        except Exception as e:
            print(f"Error handling client: {e}")
            break

    conn.close()
    print(f"Connection closed with {addr}")


def process_data(data, num_points):
    sensor1_points = []
    sensor2_points = []

    # Parse all data points
    for i in range(num_points):
        # Unpack: 1 byte sensor ID + 4 bytes timestamp + 4 bytes temp + 4 bytes pressure
        point_data = data[i * 13: (i + 1) * 13]
        sensor_id, timestamp, temp, pressure = struct.unpack("!BIff", point_data)

        if sensor_id == 1:
            sensor1_points.append((timestamp, pressure))
        elif sensor_id == 2:
            sensor2_points.append((timestamp, pressure))

    # Save to CSV
    if SAVE_DATA:
        with open(SAVE_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            for ts, p in sensor1_points:
                writer.writerow([ts, 1, p])
            for ts, p in sensor2_points:
                writer.writerow([ts, 2, p])

    # Calculate statistics for this packet
    if sensor1_points:
        pressures = [p for _, p in sensor1_points]
        min_p = min(pressures)
        max_p = max(pressures)
        diff = max_p - min_p
        times = [ts for ts, _ in sensor1_points]
        freq = len(times) / ((times[-1] - times[0]) / 1000) if len(times) > 1 else 0

        with lock:
            last_packet_stats['sensor1'] = {
                'min': min_p,
                'max': max_p,
                'diff': diff,
                'freq': freq
            }

    if sensor2_points:
        pressures = [p for _, p in sensor2_points]
        min_p = min(pressures)
        max_p = max(pressures)
        diff = max_p - min_p
        times = [ts for ts, _ in sensor2_points]
        freq = len(times) / ((times[-1] - times[0]) / 1000) if len(times) > 1 else 0

        with lock:
            last_packet_stats['sensor2'] = {
                'min': min_p,
                'max': max_p,
                'diff': diff,
                'freq': freq
            }

    # 计算差值并存储
    min_len = min(len(sensor1_points), len(sensor2_points))
    for i in range(min_len):
        ts1, p1 = sensor1_points[i]
        ts2, p2 = sensor2_points[i]
        # 压力差值 (sensor1 - sensor2)
        pressure_diff = p1 - p2
        # 时间戳差值 *10 (ts1 - ts2)*10
        ts_diff = (ts1 - ts2) * 10
        
        # 使用sensor1的时间戳作为横坐标
        with lock:
            diff_pressure.append((ts1, pressure_diff))
            diff_timestamp.append((ts1, ts_diff))

    # Add to global buffers
    with lock:
        sensor1_data.extend(sensor1_points)
        sensor2_data.extend(sensor2_points)
        
    print(f"Added {len(sensor1_points)} sensor1 points, {len(sensor2_points)} sensor2 points")
    if min_len > 0:
        print(f"Added {min_len} difference points (pressure diff range: {min([p for _, p in diff_pressure]):.4f} to {max([p for _, p in diff_pressure]):.4f}, timestamp diff range: {min([t for _, t in diff_timestamp]):.4f} to {max([t for _, t in diff_timestamp]):.4f})")


def update_plot(frame):
    with lock:
        # Get data for plotting
        s1_ts, s1_p = zip(*sensor1_data) if sensor1_data else ([], [])
        s2_ts, s2_p = zip(*sensor2_data) if sensor2_data else ([], [])
        diff_ts, diff_p = zip(*diff_pressure) if diff_pressure else ([], [])
        diff_ts_ts, diff_ts_val = zip(*diff_timestamp) if diff_timestamp else ([], [])

        # Update line data
        sensor1_line.set_data(s1_ts, s1_p)
        sensor2_line.set_data(s2_ts, s2_p)
        diff_pressure_line.set_data(diff_ts, diff_p)
        diff_timestamp_line.set_data(diff_ts_ts, diff_ts_val)

        # Update stats text
        stats = last_packet_stats
        text = (f"Sensor 1:\n"
                f"Min: {stats['sensor1']['min']:.2f} hPa\n"
                f"Max: {stats['sensor1']['max']:.2f} hPa\n"
                f"Diff: {stats['sensor1']['diff']:.2f} hPa\n"
                f"Freq: {stats['sensor1']['freq']:.1f} Hz\n\n"
                f"Sensor 2:\n"
                f"Min: {stats['sensor2']['min']:.2f} hPa\n"
                f"Max: {stats['sensor2']['max']:.2f} hPa\n"
                f"Diff: {stats['sensor2']['diff']:.2f} hPa\n"
                f"Freq: {stats['sensor2']['freq']:.1f} Hz\n\n"
                f"Pressure Diff:\n"
                f"Min: {min(diff_p) if diff_p else 0:.4f} hPa\n"
                f"Max: {max(diff_p) if diff_p else 0:.4f} hPa\n\n"
                f"Timestamp Diff:\n"
                f"Min: {min(diff_ts_val) if diff_ts_val else 0:.4f} ms\n"
                f"Max: {max(diff_ts_val) if diff_ts_val else 0:.4f} ms")
        stats_text.set_text(text)

        # 更新坐标轴范围
        all_ts = []
        if s1_ts: all_ts.extend(s1_ts)
        if s2_ts: all_ts.extend(s2_ts)
        if diff_ts: all_ts.extend(diff_ts)
        if diff_ts_ts: all_ts.extend(diff_ts_ts)
        
        if all_ts:
            min_ts = min(all_ts)
            max_ts = max(all_ts)
            ax1.set_xlim(min_ts - 100, max_ts + 100)
            
        # 更新左侧Y轴（原始传感器数据）的范围
        all_left = []
        if s1_p: all_left.extend(s1_p)
        if s2_p: all_left.extend(s2_p)
        
        if all_left:
            min_left = min(all_left)
            max_left = max(all_left)
            margin_left = (max_left - min_left) * 0.1
            ax1.set_ylim(min_left - margin_left, max_left + margin_left)
        
        # 更新右侧Y轴（压力差值）的范围
        if diff_p:
            min_right1 = min(diff_p)
            max_right1 = max(diff_p)
            margin_right1 = (max_right1 - min_right1) * 0.1
            if margin_right1 < 0.1: margin_right1 = 0.1
            ax2.set_ylim(min_right1 - margin_right1, max_right1 + margin_right1)
        
        # 更新右侧Y轴（时间戳差值）的范围
        if diff_ts_val:
            min_right2 = min(diff_ts_val)
            max_right2 = max(diff_ts_val)
            margin_right2 = (max_right2 - min_right2) * 0.1
            if margin_right2 < 0.1: margin_right2 = 0.1
            ax3.set_ylim(min_right2 - margin_right2, max_right2 + margin_right2)

    # 返回所有需要更新的对象
    return sensor1_line, sensor2_line, diff_pressure_line, diff_timestamp_line, stats_text


def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_IP, SERVER_PORT))
    server.listen(5)
    print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")

    # Start a thread to handle clients
    def accept_clients():
        while True:
            conn, addr = server.accept()
            print(f"New connection from {addr}")
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.daemon = True
            client_thread.start()

    accept_thread = threading.Thread(target=accept_clients)
    accept_thread.daemon = True
    accept_thread.start()

    # Start the plot animation
    ani = animation.FuncAnimation(
        fig, 
        update_plot, 
        interval=200,  # 更新间隔200ms
        blit=True, 
        cache_frame_data=False  # 禁用帧缓存，防止内存泄漏
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    start_server()
