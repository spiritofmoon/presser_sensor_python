# pc_client.py
# 运行在本地PC上，负责数据处理、显示和保存

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
from matplotlib.widgets import Button

matplotlib.use('TkAgg')

# --- 配置 ---
# 修改为您的公网服务器IP地址
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001  # 连接中继服务器为PC准备的端口
SAVE_FILE = "sensor_data_from_relay.csv"
# 其他配置保持不变
PLOT_WINDOW_SIZE = 10000
BUFFER_SIZE = 1024 * 8
DISPLAY_PACKET_COUNT = 10
MAX_POINTS_PER_PACKET = 1000
# ------------

# 全局变量和数据缓冲区 (与原程序相同)
is_recording = False
sensor1_data = deque(maxlen=PLOT_WINDOW_SIZE)
sensor2_data = deque(maxlen=PLOT_WINDOW_SIZE)
diff_pressure = deque(maxlen=PLOT_WINDOW_SIZE)
last_packet_stats = {
    'sensor1': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0},
    'sensor2': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0}
}
lock = threading.Lock()
global_time_offset = 0
last_timestamp = 0
recent_packet_ranges = deque(maxlen=DISPLAY_PACKET_COUNT)

# --- Matplotlib绘图界面设置 (与原程序完全相同) ---
# (此处省略了与上一版本完全相同的绘图代码，直接复制即可)
# 创建第一个图形窗口 - 原始传感器数据
fig1, ax1 = plt.subplots(figsize=(14, 8))
fig1.canvas.manager.set_window_title('Dual-Channel Pressure Sensor Monitor (PC Client)')
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Pressure (hPa)', color='blue')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.tick_params(axis='y', labelcolor='blue')
sensor1_line, = ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
lines = [sensor1_line, sensor2_line]
ax1.legend(lines, [l.get_label() for l in lines], loc='upper left')
stats_text = ax1.text(0.98, 0.95, '', transform=ax1.transAxes, verticalalignment='top', horizontalalignment='right', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
packet_count_text = ax1.text(0.98, 0.05, f'Displaying {DISPLAY_PACKET_COUNT} recent packets', transform=ax1.transAxes, verticalalignment='bottom', horizontalalignment='right', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
recording_text = ax1.text(0.02, 0.05, 'Recording: OFF', transform=ax1.transAxes, verticalalignment='bottom', horizontalalignment='left', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5), color='red', fontweight='bold')
ax1.set_title('Real-time Pressure Monitoring')
fig2, ax4 = plt.subplots(figsize=(14, 6))
fig2.canvas.manager.set_window_title('Pressure Difference Monitor (PC Client)')
ax4.set_xlabel('Time (ms)')
ax4.set_ylabel('Pressure Difference (S1-S2) hPa', color='green')
ax4.grid(True, linestyle='--', alpha=0.6)
ax4.tick_params(axis='y', labelcolor='green')
ax4.set_title('Pressure Difference (Sensor1 - Sensor2)')
diff_pressure_line_only, = ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
ax4.legend(loc='upper right')
ax4.text(0.98, 0.05, f'Displaying {DISPLAY_PACKET_COUNT} recent packets', transform=ax4.transAxes, verticalalignment='bottom', horizontalalignment='right', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
ax1.set_ylim(-10, 10)
ax1.set_xlim(0, 10000)
ax4.set_ylim(-10, 10)
ax4.set_xlim(0, 10000)
ax_button = plt.axes([0.02, 0.9, 0.1, 0.05])
record_button = Button(ax_button, 'Start Recording', color='lightgray', hovercolor='0.8')
record_button.label.set_fontsize(10)


# ---------------------------------------------------


# toggle_recording 和 process_data 函数与原程序完全相同
def toggle_recording(event):
    global is_recording
    is_recording = not is_recording
    if is_recording:
        record_button.label.set_text('Stop Recording')
        recording_text.set_text('Recording: ON')
        recording_text.set_color('green')
        print("Recording started...")
    else:
        record_button.label.set_text('Start Recording')
        recording_text.set_text('Recording: OFF')
        recording_text.set_color('red')
        print("Recording stopped.")
    fig1.canvas.draw_idle()


record_button.on_clicked(toggle_recording)


def process_data(data, num_points):
    # 此函数与您上一版的代码完全相同，无需修改
    global global_time_offset, last_timestamp
    sensor1_points, sensor2_points = [], []
    packet_min_time, packet_max_time = float('inf'), -float('inf')

    for i in range(num_points):
        point_data = data[i * 13: (i + 1) * 13]
        sensor_id, timestamp, temp, pressure = struct.unpack("!BIff", point_data)
        if i == 0 and last_timestamp > 0:
            global_time_offset = last_timestamp - timestamp + 100
        adjusted_timestamp = timestamp + global_time_offset
        packet_min_time = min(packet_min_time, adjusted_timestamp)
        packet_max_time = max(packet_max_time, adjusted_timestamp)
        if sensor_id == 1:
            sensor1_points.append((adjusted_timestamp, pressure))
        elif sensor_id == 2:
            sensor2_points.append((adjusted_timestamp, pressure))

    with lock:
        recent_packet_ranges.append((packet_min_time, packet_max_time))

    if sensor1_points:
        last_timestamp = sensor1_points[-1][0]
    elif sensor2_points:
        last_timestamp = sensor2_points[-1][0]

    if is_recording:
        try:
            with open(SAVE_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows([[ts, 1, p] for ts, p in sensor1_points])
                writer.writerows([[ts, 2, p] for ts, p in sensor2_points])
        except Exception as e:
            print(f"Error saving data to CSV: {e}")

    if sensor1_points:
        pressures = [p for _, p in sensor1_points]
        times = [ts for ts, _ in sensor1_points]
        with lock: last_packet_stats['sensor1'] = {'min': min(pressures), 'max': max(pressures), 'diff': max(pressures) - min(pressures), 'freq': len(times) / ((times[-1] - times[0]) / 1000) if len(times) > 1 else 0}
    if sensor2_points:
        pressures = [p for _, p in sensor2_points]
        times = [ts for ts, _ in sensor2_points]
        with lock: last_packet_stats['sensor2'] = {'min': min(pressures), 'max': max(pressures), 'diff': max(pressures) - min(pressures), 'freq': len(times) / ((times[-1] - times[0]) / 1000) if len(times) > 1 else 0}

    min_len = min(len(sensor1_points), len(sensor2_points))
    diff_points = [(sensor1_points[i][0], sensor1_points[i][1] - sensor2_points[i][1]) for i in range(min_len)]

    with lock:
        sensor1_data.extend(sensor1_points)
        sensor2_data.extend(sensor2_points)
        diff_pressure.extend(diff_points)

    print(f"Processed {len(sensor1_points)} S1 points, {len(sensor2_points)} S2 points.")


# update_main_plot 和 update_diff_plot 函数与原程序完全相同
def update_main_plot(frame):
    # 此函数与您上一版的代码完全相同，无需修改
    with lock:
        if recent_packet_ranges:
            min_display_ts = min(r[0] for r in recent_packet_ranges)
            s1_data = [x for x in sensor1_data if x[0] >= min_display_ts]
            s1_ts, s1_p = [x[0] for x in s1_data], [x[1] for x in s1_data]
            s2_data = [x for x in sensor2_data if x[0] >= min_display_ts]
            s2_ts, s2_p = [x[0] for x in s2_data], [x[1] for x in s2_data]
        else:
            s1_ts, s1_p, s2_ts, s2_p = [], [], [], []

        sensor1_line.set_data(s1_ts, s1_p)
        sensor2_line.set_data(s2_ts, s2_p)

        stats = last_packet_stats
        text = (f"Sensor 1:\n" f"Min: {stats['sensor1']['min']:.2f} hPa\n" f"Max: {stats['sensor1']['max']:.2f} hPa\n" f"Diff: {stats['sensor1']['diff']:.2f} hPa\n" f"Freq: {stats['sensor1']['freq']:.1f} Hz\n\n" f"Sensor 2:\n" f"Min: {stats['sensor2']['min']:.2f} hPa\n" f"Max: {stats['sensor2']['max']:.2f} hPa\n" f"Diff: {stats['sensor2']['diff']:.2f} hPa\n" f"Freq: {stats['sensor2']['freq']:.1f} Hz")
        stats_text.set_text(text)

        all_ts = s1_ts + s2_ts
        if all_ts:
            min_ts, max_ts = min(all_ts), max(all_ts)
            ax1.set_xlim(min_ts - 100, max_ts + 1000)
            all_p = s1_p + s2_p
            if all_p:
                min_p, max_p = min(all_p), max(all_p)
                margin = max(0.1, (max_p - min_p) * 0.2)
                ax1.set_ylim(min_p - margin, max_p + margin)
    return sensor1_line, sensor2_line, stats_text, packet_count_text, recording_text


def update_diff_plot(frame):
    # 此函数与您上一版的代码完全相同，无需修改
    with lock:
        if recent_packet_ranges:
            min_display_ts = min(r[0] for r in recent_packet_ranges)
            diff_data = [x for x in diff_pressure if x[0] >= min_display_ts]
            diff_ts, diff_p_val = [x[0] for x in diff_data], [x[1] for x in diff_data]
        else:
            diff_ts, diff_p_val = [], []

        diff_pressure_line_only.set_data(diff_ts, diff_p_val)

        if diff_ts:
            min_ts, max_ts = min(diff_ts), max(diff_ts)
            ax4.set_xlim(min_ts - 100, max_ts + 1000)
        if diff_p_val:
            min_diff, max_diff = min(diff_p_val), max(diff_p_val)
            margin = max(0.1, (max_diff - min_diff) * 0.2)
            ax4.set_ylim(min_diff - margin, max_diff + margin)
    return diff_pressure_line_only,


def start_data_reception():
    """
    核心函数：连接到中继服务器并接收数据。
    取代了原程序中的 handle_client 和 start_server。
    """
    while True:  # 外层循环，用于断线重连
        try:
            print(f"Attempting to connect to relay server at {RELAY_SERVER_IP}:{RELAY_SERVER_PORT}...")
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
            print("Successfully connected to relay server. Waiting for data...")

            # 内层循环，处理来自中继服务器的数据流
            while True:
                # 1. 读取包头 (4字节，表示数据点数量)
                header = conn.recv(4)
                if not header:
                    print("Connection closed by relay server.")
                    break

                num_points = struct.unpack("!I", header)[0]

                # 2. 检查数据包是否异常
                if num_points > MAX_POINTS_PER_PACKET:
                    print(f"Received abnormal packet header with {num_points} points. Stream may be corrupt.")
                    # 由于我们不再能像服务器一样断开恶意客户端，这里我们只能选择断开与中继的连接来强制同步
                    print("Disconnecting to re-synchronize...")
                    break

                    # 3. 读取完整的数据体
                data = b''
                bytes_to_read = num_points * 13
                while len(data) < bytes_to_read:
                    chunk = conn.recv(min(BUFFER_SIZE, bytes_to_read - len(data)))
                    if not chunk:
                        raise ConnectionError("Relay connection lost while reading data packet.")
                    data += chunk

                # 4. 处理数据
                process_data(data, num_points)

        except ConnectionRefusedError:
            print("Connection refused. Is the relay server running? Retrying in 10 seconds...")
            time.sleep(10)
        except (socket.timeout, ConnectionError, ConnectionResetError, BrokenPipeError) as e:
            print(f"Connection to relay lost: {e}. Reconnecting in 5 seconds...")
            time.sleep(5)
        except Exception as e:
            print(f"An unexpected error occurred: {e}. Retrying in 10 seconds...")
            time.sleep(10)
        finally:
            if 'conn' in locals():
                conn.close()


if __name__ == "__main__":
    # 启动数据接收线程
    receiver_thread = threading.Thread(target=start_data_reception, daemon=True)
    receiver_thread.start()

    # 启动Matplotlib动画
    ani_main = animation.FuncAnimation(fig1, update_main_plot, interval=200, blit=True, cache_frame_data=False)
    ani_diff = animation.FuncAnimation(fig2, update_diff_plot, interval=200, blit=True, cache_frame_data=False)

    plt.tight_layout()
    plt.show()