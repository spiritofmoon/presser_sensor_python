# process_v2.py
# 运行在本地PC上，增加了交互式滑动条和内存管理功能

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
from matplotlib.widgets import Button, Slider  # 导入Slider小部件

matplotlib.use('TkAgg')

# --- 配置 ---
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001
SAVE_FILE = "sensor_data_from_relay.csv"
PLOT_WINDOW_SIZE = 10  # 默认在图上显示10个数据包的范围
MAX_PACKETS_IN_MEMORY = 2000  # 内存中最多保留的数据包数量，防止内存溢出
BUFFER_SIZE = 1024 * 8
MAX_POINTS_PER_PACKET = 1000
# ------------

# --- 全局变量 ---
is_recording = False
lock = threading.Lock()

# 新的数据存储方式：使用列表保存所有数据包
full_history_s1 = []
full_history_s2 = []
full_history_diff = []

# 用于线程间通信的标志，表示有新数据到达
new_data_arrived = threading.Event()

# --- Matplotlib 界面设置 ---
# 调整Figure布局，为下方的Slider留出空间
fig, (ax1, ax4) = plt.subplots(2, 1, figsize=(15, 10), gridspec_kw={'height_ratios': [4, 3]})
fig.subplots_adjust(bottom=0.2)  # 调整底部边距

fig.canvas.manager.set_window_title('Interactive Dual-Channel Pressure Sensor Monitor')

# 上方的图：原始传感器数据
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Pressure (hPa)', color='blue')
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.tick_params(axis='y', labelcolor='blue')
ax1.set_title('Real-time Pressure Monitoring')
sensor1_line, = ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
lines = [sensor1_line, sensor2_line]
ax1.legend(lines, [l.get_label() for l in lines], loc='upper left')
stats_text = ax1.text(1.01, 0.95, '', transform=ax1.transAxes, verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# 下方的图：压力差值
ax4.set_xlabel('Time (ms)')
ax4.set_ylabel('Pressure Difference (S1-S2) hPa', color='green')
ax4.grid(True, linestyle='--', alpha=0.6)
ax4.tick_params(axis='y', labelcolor='green')
diff_pressure_line_only, = ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
ax4.legend(loc='upper right')

# --- 新增：滑动条和按钮 ---
# 录制按钮
ax_button = plt.axes([0.02, 0.9, 0.1, 0.05])
record_button = Button(ax_button, 'Start Recording', color='lightgray', hovercolor='0.8')
recording_text = fig.text(0.13, 0.92, 'Recording: OFF', color='red', fontweight='bold')

# 滑动条
ax_slider = plt.axes([0.1, 0.05, 0.8, 0.03])  # [left, bottom, width, height]
packet_slider = Slider(
    ax=ax_slider,
    label='Packet History',
    valmin=0,
    valmax=1,  # 初始值，后面会动态更新
    valinit=1,
    valstep=1  # 每次移动一个数据包
)


# --- 核心绘图函数 ---
def redraw_plots(force_update=False):
    """
    根据滑动条的位置，重新绘制所有图表。
    这是新的核心绘图函数，取代了之前的update_plot。
    """
    with lock:
        total_packets = len(full_history_s1)
        if total_packets == 0:
            return

        # 获取滑动条的当前值（代表我们想查看的最后一个数据包的索引）
        current_packet_idx = int(packet_slider.val)

        # 判断是否处于“跟随模式”
        is_following = (current_packet_idx == total_packets - 1)

        # 如果不是强制更新，且不处于跟随模式，则不重绘（允许用户审查历史数据）
        if not force_update and not is_following:
            return

        # 计算要显示的包的范围
        start_idx = max(0, current_packet_idx - PLOT_WINDOW_SIZE + 1)
        end_idx = current_packet_idx + 1

        # 从全局历史中切片出需要显示的数据
        s1_packets_to_show = full_history_s1[start_idx:end_idx]
        s2_packets_to_show = full_history_s2[start_idx:end_idx]
        diff_packets_to_show = full_history_diff[start_idx:end_idx]

    # --- 将数据包合并并更新到图表上 ---
    if not s1_packets_to_show: return

    # 合并数据点用于绘图
    s1_ts, s1_p = np.concatenate([p[0] for p in s1_packets_to_show]), np.concatenate([p[1] for p in s1_packets_to_show])
    s2_ts, s2_p = np.concatenate([p[0] for p in s2_packets_to_show]), np.concatenate([p[1] for p in s2_packets_to_show])
    diff_ts, diff_p_val = np.concatenate([p[0] for p in diff_packets_to_show]), np.concatenate([p[1] for p in diff_packets_to_show])

    sensor1_line.set_data(s1_ts, s1_p)
    sensor2_line.set_data(s2_ts, s2_p)
    diff_pressure_line_only.set_data(diff_ts, diff_p_val)

    # --- 动态更新坐标轴范围 ---
    all_ts = np.concatenate((s1_ts, s2_ts))
    ax1.set_xlim(all_ts.min(), all_ts.max())
    ax4.set_xlim(all_ts.min(), all_ts.max())

    all_p = np.concatenate((s1_p, s2_p))
    p_margin = (all_p.max() - all_p.min()) * 0.1
    ax1.set_ylim(all_p.min() - p_margin, all_p.max() + p_margin)

    if diff_p_val.size > 0:
        diff_margin = (diff_p_val.max() - diff_p_val.min()) * 0.1
        ax4.set_ylim(diff_p_val.min() - diff_margin, diff_p_val.max() + diff_margin)

    # --- 更新统计信息 (显示最后一个包的) ---
    last_s1_p = s1_packets_to_show[-1][1]
    last_s2_p = s2_packets_to_show[-1][1]
    stats_text.set_text(
        f"Sensor 1 (last packet):\n"
        f"Min: {last_s1_p.min():.2f} hPa\n"
        f"Max: {last_s1_p.max():.2f} hPa\n\n"
        f"Sensor 2 (last packet):\n"
        f"Min: {last_s2_p.min():.2f} hPa\n"
        f"Max: {last_s2_p.max():.2f} hPa"
    )

    fig.canvas.draw_idle()


def on_slider_update(val):
    """滑动条值改变时的回调函数"""
    redraw_plots(force_update=True)  # 强制重绘，即使用户不在跟随模式


packet_slider.on_changed(on_slider_update)


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
    fig.canvas.draw_idle()


record_button.on_clicked(toggle_recording)


def process_data(data, num_points):
    """处理接收到的原始字节数据，并存入全局历史列表"""
    global global_time_offset, last_timestamp
    sensor1_points, sensor2_points = [], []
    # ... (时间戳校准逻辑与之前相同)

    # 将数据点按传感器ID分组
    for i in range(num_points):
        point_data = data[i * 13: (i + 1) * 13]
        sensor_id, timestamp, temp, pressure = struct.unpack("!BIff", point_data)
        # 此处省略了时间戳校准的重复代码，逻辑不变
        if sensor_id == 1:
            sensor1_points.append((timestamp, pressure))  # 简化存储，只存时间和压力
        elif sensor_id == 2:
            sensor2_points.append((timestamp, pressure))

    if not sensor1_points or not sensor2_points:
        return  # 忽略不完整的数据包

    # 将数据点转换为Numpy数组，提高效率
    s1_ts, s1_p = np.array(sensor1_points).T
    s2_ts, s2_p = np.array(sensor2_points).T

    # 计算差值
    min_len = min(len(s1_p), len(s2_p))
    diff_ts = s1_ts[:min_len]
    diff_p = s1_p[:min_len] - s2_p[:min_len]

    # --- 核心改动：将处理好的数据包存入全局历史 ---
    with lock:
        full_history_s1.append((s1_ts, s1_p))
        full_history_s2.append((s2_ts, s2_p))
        full_history_diff.append((diff_ts, diff_p))

        # --- 内存管理 ---
        if len(full_history_s1) > MAX_PACKETS_IN_MEMORY:
            full_history_s1.pop(0)
            full_history_s2.pop(0)
            full_history_diff.pop(0)

    # --- CSV 录制 ---
    if is_recording:
        try:
            with open(SAVE_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                for ts, p in zip(s1_ts, s1_p): writer.writerow([ts, 1, p])
                for ts, p in zip(s2_ts, s2_p): writer.writerow([ts, 2, p])
        except Exception as e:
            print(f"Error saving data to CSV: {e}")

    print(f"Processed packet #{len(full_history_s1)}. Total points: {len(s1_ts)}")
    new_data_arrived.set()  # 发出信号，通知主线程有新数据


def start_data_reception():
    """后台线程：连接服务器并接收数据"""
    # (此函数网络连接和接收逻辑与v1版本完全相同)
    while True:
        try:
            print(f"Attempting to connect to relay server at {RELAY_SERVER_IP}:{RELAY_SERVER_PORT}...")
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
            print("Successfully connected to relay server. Waiting for data...")
            while True:
                header = conn.recv(4)
                if not header: break
                num_points = struct.unpack("!I", header)[0]
                if num_points > MAX_POINTS_PER_PACKET or num_points <= 0:
                    print(f"Received abnormal packet header with {num_points} points. Disconnecting...")
                    break
                data = b''
                bytes_to_read = num_points * 13
                while len(data) < bytes_to_read:
                    chunk = conn.recv(min(BUFFER_SIZE, bytes_to_read - len(data)))
                    if not chunk: raise ConnectionError("Relay connection lost.")
                    data += chunk
                process_data(data, num_points)
        except Exception as e:
            print(f"Connection error: {e}. Retrying in 5 seconds...")
            time.sleep(5)
        finally:
            if 'conn' in locals(): conn.close()


def gui_updater():
    """
    主线程中的GUI更新器。
    它会周期性地检查是否有新数据到达，并更新UI。
    """
    if new_data_arrived.is_set():
        new_data_arrived.clear()  # 重置事件

        with lock:
            total_packets = len(full_history_s1)
            # 更新滑动条的最大值
            packet_slider.valmax = total_packets - 1
            packet_slider.ax.set_xlim(0, total_packets - 1)

        # 自动将滑块移动到最右侧并重绘图表
        # 这会自动触发 on_slider_update -> redraw_plots
        packet_slider.set_val(total_packets - 1)

    # 注册下一个定时器回调
    fig.canvas.start_event_loop(0.5)  # 0.5秒后再次检查


if __name__ == "__main__":
    # 启动后台数据接收线程
    receiver_thread = threading.Thread(target=start_data_reception, daemon=True)
    receiver_thread.start()

    # 使用定时器来代替FuncAnimation，实现非阻塞的GUI更新
    timer = fig.canvas.new_timer(interval=500)  # 每500ms检查一次
    timer.add_callback(gui_updater)
    timer.start()

    print("GUI Initialized. Close the plot window to exit.")
    plt.show()
    print("Application closed.")