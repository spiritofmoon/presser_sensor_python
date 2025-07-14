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
from matplotlib.widgets import Button  # 导入按钮组件

matplotlib.use('TkAgg')

# Configuration
SERVER_IP = "0.0.0.0"  # Listen on all interfaces
SERVER_PORT = 5000
SAVE_FILE = "sensor_data.csv"
PLOT_WINDOW_SIZE = 10000  # 10 seconds of data at 100Hz
BUFFER_SIZE = 1024 * 8
DISPLAY_PACKET_COUNT = 10  # 只显示最近N个数据包的数据
MAX_POINTS_PER_PACKET = 1000  # 正常单次发送不超过400点，设置1000为安全上限

# 添加记录状态标志
is_recording = False  # 初始状态为未记录

# Data buffers
sensor1_data = deque(maxlen=PLOT_WINDOW_SIZE)
sensor2_data = deque(maxlen=PLOT_WINDOW_SIZE)
diff_pressure = deque(maxlen=PLOT_WINDOW_SIZE)  # 存储压力差值
last_packet_stats = {
    'sensor1': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0},
    'sensor2': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0}
}
lock = threading.Lock()

# 全局时间戳偏移量，确保时间戳连续
global_time_offset = 0
last_timestamp = 0
recent_packet_ranges = deque(maxlen=DISPLAY_PACKET_COUNT)  # 存储最近N个数据包的时间范围

# 创建第一个图形窗口 - 原始传感器数据
fig1, ax1 = plt.subplots(figsize=(14, 8))
fig1.canvas.manager.set_window_title('Dual-Channel Pressure Sensor Monitor')

# 主Y轴（左侧）用于原始传感器数据
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Pressure (hPa)', color='blue')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.tick_params(axis='y', labelcolor='blue')

# 绘制传感器数据线
sensor1_line, = ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)

# 合并图例
lines = [sensor1_line, sensor2_line]
ax1.legend(lines, [l.get_label() for l in lines], loc='upper left')

# 添加统计信息文本
stats_text = ax1.text(0.98, 0.95, '', transform=ax1.transAxes,
                      verticalalignment='top', horizontalalignment='right',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# 添加数据包计数文本
packet_count_text = ax1.text(0.98, 0.05, f'Displaying {DISPLAY_PACKET_COUNT} recent packets',
                             transform=ax1.transAxes,
                             verticalalignment='bottom', horizontalalignment='right',
                             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))

# 添加记录状态文本
recording_text = ax1.text(0.02, 0.05, 'Recording: OFF', transform=ax1.transAxes,
                          verticalalignment='bottom', horizontalalignment='left',
                          bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5),
                          color='red', fontweight='bold')

# 设置标题
ax1.set_title('Real-time Pressure Monitoring')

# 创建第二个图形窗口 - 专门显示压力差值
fig2, ax4 = plt.subplots(figsize=(14, 6))
fig2.canvas.manager.set_window_title('Pressure Difference Monitor')
ax4.set_xlabel('Time (ms)')
ax4.set_ylabel('Pressure Difference (S1-S2) hPa', color='green')
ax4.grid(True, linestyle='--', alpha=0.6)
ax4.tick_params(axis='y', labelcolor='green')
ax4.set_title('Pressure Difference (Sensor1 - Sensor2)')

# 在第二个窗口中绘制压力差值线
diff_pressure_line_only, = ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
ax4.legend(loc='upper right')

# 第二个窗口的数据包计数文本
ax4.text(0.98, 0.05, f'Displaying {DISPLAY_PACKET_COUNT} recent packets',
         transform=ax4.transAxes,
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))

# 初始设置坐标轴范围（后续自动调整）
ax1.set_ylim(-10, 10)
ax1.set_xlim(0, 10000)
ax4.set_ylim(-10, 10)
ax4.set_xlim(0, 10000)

# 创建记录按钮
ax_button = plt.axes([0.02, 0.9, 0.1, 0.05])  # [left, bottom, width, height]
record_button = Button(ax_button, 'Start Recording', color='lightgray', hovercolor='0.8')
record_button.label.set_fontsize(10)


# 记录按钮点击事件处理
def toggle_recording(event):
    global is_recording
    is_recording = not is_recording

    # 更新按钮文本和记录状态显示
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

    # 更新图形界面
    fig1.canvas.draw_idle()


# 绑定按钮点击事件
record_button.on_clicked(toggle_recording)


def handle_client(conn, addr):
    print(f"Connection from {addr}")
    while True:
        try:
            # Read packet header (4 bytes - number of points)
            header = conn.recv(4)
            if not header:
                break

            num_points = struct.unpack("!I", header)[0]

            # 添加数据包长度检查
            if num_points > MAX_POINTS_PER_PACKET:
                print(f"异常数据包！点数过多: {num_points} (正常应 < {MAX_POINTS_PER_PACKET})，丢弃该数据包")

                # 丢弃该数据包的所有数据
                bytes_to_discard = num_points * 13
                while bytes_to_discard > 0:
                    # 每次最多丢弃BUFFER_SIZE字节
                    discard_size = min(bytes_to_discard, BUFFER_SIZE)
                    conn.recv(discard_size)
                    bytes_to_discard -= discard_size
                continue  # 跳过处理，进入下一个循环

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
    global global_time_offset, last_timestamp

    sensor1_points = []
    sensor2_points = []

    # 记录数据包的时间范围
    packet_min_time = float('inf')
    packet_max_time = -float('inf')

    # Parse all data points
    for i in range(num_points):
        # Unpack: 1 byte sensor ID + 4 bytes timestamp + 4 bytes temp + 4 bytes pressure
        point_data = data[i * 13: (i + 1) * 13]
        sensor_id, timestamp, temp, pressure = struct.unpack("!BIff", point_data)

        # 调整时间戳以确保连续性
        if i == 0 and last_timestamp > 0:
            # 计算时间戳偏移量
            global_time_offset = last_timestamp - timestamp + 100  # 添加100ms间隔

        adjusted_timestamp = timestamp + global_time_offset

        # 更新数据包时间范围
        if adjusted_timestamp < packet_min_time:
            packet_min_time = adjusted_timestamp
        if adjusted_timestamp > packet_max_time:
            packet_max_time = adjusted_timestamp

        if sensor_id == 1:
            sensor1_points.append((adjusted_timestamp, pressure))
        elif sensor_id == 2:
            sensor2_points.append((adjusted_timestamp, pressure))

    # 存储数据包时间范围
    with lock:
        recent_packet_ranges.append((packet_min_time, packet_max_time))

    # 更新最后时间戳
    if sensor1_points:
        last_timestamp = sensor1_points[-1][0]
    elif sensor2_points:
        last_timestamp = sensor2_points[-1][0]

    # 只有在记录状态下才保存到CSV
    if is_recording:
        try:
            with open(SAVE_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                for ts, p in sensor1_points:
                    writer.writerow([ts, 1, p])
                for ts, p in sensor2_points:
                    writer.writerow([ts, 2, p])
            # print(f"Saved {len(sensor1_points) + len(sensor2_points)} data points to CSV")
        except Exception as e:
            print(f"Error saving data to CSV: {e}")

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
        # 使用sensor1的时间戳作为横坐标
        with lock:
            diff_pressure.append((ts1, pressure_diff))

    # Add to global buffers
    with lock:
        sensor1_data.extend(sensor1_points)
        sensor2_data.extend(sensor2_points)

    print(f"Added {len(sensor1_points)} sensor1 points, {len(sensor2_points)} sensor2 points")
    if min_len > 0:
        print(f"Added {min_len} difference points (pressure diff range: {min([p for _, p in diff_pressure]):.4f} to {max([p for _, p in diff_pressure]):.4f})")


def update_main_plot(frame):
    with lock:
        # 只保留最近DISPLAY_PACKET_COUNT个数据包的数据
        if recent_packet_ranges:
            # 计算需要显示的最早时间戳
            min_display_ts = min([range[0] for range in recent_packet_ranges])

            # 过滤数据
            s1_data = [x for x in sensor1_data if x[0] >= min_display_ts]
            s1_ts = [x[0] for x in s1_data]
            s1_p = [x[1] for x in s1_data]

            s2_data = [x for x in sensor2_data if x[0] >= min_display_ts]
            s2_ts = [x[0] for x in s2_data]
            s2_p = [x[1] for x in s2_data]
        else:
            s1_ts = []
            s1_p = []
            s2_ts = []
            s2_p = []

        # Update line data for first window
        sensor1_line.set_data(s1_ts, s1_p)
        sensor2_line.set_data(s2_ts, s2_p)

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
                f"Freq: {stats['sensor2']['freq']:.1f} Hz")
        stats_text.set_text(text)

        # 更新坐标轴范围
        all_ts = []
        if s1_ts: all_ts.extend(s1_ts)
        if s2_ts: all_ts.extend(s2_ts)

        if all_ts:
            min_ts = min(all_ts)
            max_ts = max(all_ts)
            # 动态调整X轴范围
            ax1.set_xlim(min_ts - 100, max_ts + 1000)  # 添加100ms前缀和1000ms后缀

            # 更新Y轴范围
            if s1_p or s2_p:
                all_left = []
                if s1_p: all_left.extend(s1_p)
                if s2_p: all_left.extend(s2_p)
                min_left = min(all_left)
                max_left = max(all_left)
                margin_left = max(0.1, (max_left - min_left) * 0.2)  # 至少0.1的边距
                ax1.set_ylim(min_left - margin_left, max_left + margin_left)

    # 返回所有需要更新的对象
    return sensor1_line, sensor2_line, stats_text, packet_count_text, recording_text


def update_diff_plot(frame):
    with lock:
        # 只保留最近DISPLAY_PACKET_COUNT个数据包的数据
        if recent_packet_ranges:
            # 计算需要显示的最早时间戳
            min_display_ts = min([range[0] for range in recent_packet_ranges])

            # 过滤数据
            diff_data = [x for x in diff_pressure if x[0] >= min_display_ts]
            diff_ts = [x[0] for x in diff_data]
            diff_p_val = [x[1] for x in diff_data]
        else:
            diff_ts = []
            diff_p_val = []

        # 更新第二个窗口的压力差值线
        diff_pressure_line_only.set_data(diff_ts, diff_p_val)

        # 更新第二个窗口的坐标轴范围
        if diff_ts:
            min_ts = min(diff_ts)
            max_ts = max(diff_ts)
            # 动态调整X轴范围
            ax4.set_xlim(min_ts - 100, max_ts + 1000)  # 添加100ms前缀和1000ms后缀

        if diff_p_val:
            min_diff = min(diff_p_val)
            max_diff = max(diff_p_val)
            margin_diff = max(0.1, (max_diff - min_diff) * 0.2)  # 至少0.1的边距
            ax4.set_ylim(min_diff - margin_diff, max_diff + margin_diff)

    return diff_pressure_line_only,


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

    # Start the plot animations
    ani_main = animation.FuncAnimation(
        fig1,
        update_main_plot,
        interval=200,  # 更新间隔200ms
        blit=True,
        cache_frame_data=False  # 禁用帧缓存，防止内存泄漏
    )

    ani_diff = animation.FuncAnimation(
        fig2,
        update_diff_plot,
        interval=200,
        blit=True,
        cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    start_server()
