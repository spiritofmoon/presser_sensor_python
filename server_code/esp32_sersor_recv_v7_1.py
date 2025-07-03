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
DISPLAY_PACKET_COUNT = 10  # NEW: Number of recent packets to display in the plot

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

# NEW: Track packet boundaries for each data buffer
packet_boundaries = {
    'sensor1': deque(maxlen=DISPLAY_PACKET_COUNT),
    'sensor2': deque(maxlen=DISPLAY_PACKET_COUNT),
    'diff_pressure': deque(maxlen=DISPLAY_PACKET_COUNT),
    'diff_timestamp': deque(maxlen=DISPLAY_PACKET_COUNT)
}

# 创建第一个图形窗口 - 原始传感器数据
fig1, ax1 = plt.subplots(figsize=(14, 8))
fig1.canvas.manager.set_window_title('Dual-Channel Pressure Sensor Monitor')

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

# 创建第二个图形窗口 - 专门显示压力差值
fig2, ax4 = plt.subplots(figsize=(14, 6))
fig2.canvas.manager.set_window_title('Pressure Difference Monitor')
ax4.set_xlabel('Time (ms)')
ax4.set_ylabel('Pressure Difference (S1-S2) hPa', color='green')
ax4.grid(True, linestyle='--', alpha=0.6)
ax4.tick_params(axis='y', labelcolor='green')
ax4.set_title('Pressure Difference (Sensor1 - Sensor2)')
ax4.set_ylim(-10, 10)
ax4.set_xlim(0, 10000)

# 在第二个窗口中绘制压力差值线
diff_pressure_line_only, = ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
ax4.legend(loc='upper right')

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
        # NEW: Track the starting index for this packet in each buffer
        start_index_s1 = len(sensor1_data)
        start_index_s2 = len(sensor2_data)
        start_index_diff_p = len(diff_pressure) - min_len
        start_index_diff_ts = len(diff_timestamp) - min_len

        sensor1_data.extend(sensor1_points)
        sensor2_data.extend(sensor2_points)

        # NEW: Store packet boundaries
        if sensor1_points:
            packet_boundaries['sensor1'].append((start_index_s1, len(sensor1_data)))
        if sensor2_points:
            packet_boundaries['sensor2'].append((start_index_s2, len(sensor2_data)))
        if min_len > 0:
            packet_boundaries['diff_pressure'].append((start_index_diff_p, len(diff_pressure)))
            packet_boundaries['diff_timestamp'].append((start_index_diff_ts, len(diff_timestamp)))

    print(f"Added {len(sensor1_points)} sensor1 points, {len(sensor2_points)} sensor2 points")
    if min_len > 0:
        print(f"Added {min_len} difference points (pressure diff range: {min([p for _, p in diff_pressure]):.4f} to {max([p for _, p in diff_pressure]):.4f}, timestamp diff range: {min([t for _, t in diff_timestamp]):.4f} to {max([t for _, t in diff_timestamp]):.4f})")


def get_recent_data(data_buffer, buffer_name):
    """NEW: Get only the data from the most recent DISPLAY_PACKET_COUNT packets"""
    with lock:
        if DISPLAY_PACKET_COUNT == 0:  # Show all data
            return list(data_buffer)

        # Get the boundaries of the most recent packets
        boundaries = list(packet_boundaries[buffer_name])
        if not boundaries:
            return []

        # Only keep the most recent DISPLAY_PACKET_COUNT packets
        boundaries = boundaries[-DISPLAY_PACKET_COUNT:]

        # Extract data from these boundaries
        recent_data = []
        for start, end in boundaries:
            if start < len(data_buffer) and end <= len(data_buffer):
                recent_data.extend(list(data_buffer)[start:end])

        return recent_data


def update_plot(frame):
    # NEW: Get only the recent data based on DISPLAY_PACKET_COUNT
    s1_recent = get_recent_data(sensor1_data, 'sensor1')
    s2_recent = get_recent_data(sensor2_data, 'sensor2')
    diff_p_recent = get_recent_data(diff_pressure, 'diff_pressure')
    diff_ts_recent = get_recent_data(diff_timestamp, 'diff_timestamp')

    with lock:
        # Get data for plotting from the recent data
        s1_ts = [x[0] for x in s1_recent] if s1_recent else []
        s1_p = [x[1] for x in s1_recent] if s1_recent else []
        s2_ts = [x[0] for x in s2_recent] if s2_recent else []
        s2_p = [x[1] for x in s2_recent] if s2_recent else []
        diff_ts = [x[0] for x in diff_p_recent] if diff_p_recent else []
        diff_p = [x[1] for x in diff_p_recent] if diff_p_recent else []
        diff_ts_ts = [x[0] for x in diff_ts_recent] if diff_ts_recent else []
        diff_ts_val = [x[1] for x in diff_ts_recent] if diff_ts_recent else []

        # Update line data for both windows
        sensor1_line.set_data(s1_ts, s1_p)
        sensor2_line.set_data(s2_ts, s2_p)
        diff_pressure_line.set_data(diff_ts, diff_p)
        diff_timestamp_line.set_data(diff_ts_ts, diff_ts_val)
        diff_pressure_line_only.set_data(diff_ts, diff_p)  # 更新第二个窗口的压力差值线

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
            # 更新两个窗口的X轴范围
            ax1.set_xlim(min_ts - 100, max_ts + 100)
            ax4.set_xlim(min_ts - 100, max_ts + 100)

        # 更新第一个窗口的Y轴范围
        if s1_p or s2_p:
            all_left = []
            if s1_p: all_left.extend(s1_p)
            if s2_p: all_left.extend(s2_p)
            min_left = min(all_left)
            max_left = max(all_left)
            margin_left = (max_left - min_left) * 0.1
            ax1.set_ylim(min_left - margin_left, max_left + margin_left)

        # 更新第一个窗口的压力差值Y轴范围
        if diff_p:
            min_right1 = min(diff_p)
            max_right1 = max(diff_p)
            margin_right1 = (max_right1 - min_right1) * 0.1
            if margin_right1 < 0.1: margin_right1 = 0.1
            ax2.set_ylim(min_right1 - margin_right1, max_right1 + margin_right1)

        # 更新第一个窗口的时间戳差值Y轴范围
        if diff_ts_val:
            min_right2 = min(diff_ts_val)
            max_right2 = max(diff_ts_val)
            margin_right2 = (max_right2 - min_right2) * 0.1
            if margin_right2 < 0.1: margin_right2 = 0.1
            ax3.set_ylim(min_right2 - margin_right2, max_right2 + margin_right2)

        # 更新第二个窗口的Y轴范围（压力差值）
        if diff_p:
            min_diff = min(diff_p)
            max_diff = max(diff_p)
            margin_diff = (max_diff - min_diff) * 0.1
            if margin_diff < 0.1: margin_diff = 0.1
            ax4.set_ylim(min_diff - margin_diff, max_diff + margin_diff)

    # 返回所有需要更新的对象
    return sensor1_line, sensor2_line, diff_pressure_line, diff_timestamp_line, stats_text, diff_pressure_line_only


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
        fig1,
        update_plot,
        interval=200,  # 更新间隔200ms
        blit=True,
        cache_frame_data=False  # 禁用帧缓存，防止内存泄漏
    )

    # 为第二个窗口创建动画
    ani2 = animation.FuncAnimation(
        fig2,
        update_plot,
        interval=200,
        blit=True,
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    start_server()