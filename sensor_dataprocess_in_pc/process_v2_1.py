# process_v5.py
# 修复了接收数据后UI卡死（停止更新）的问题

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
from matplotlib.widgets import Button, RangeSlider

matplotlib.use('TkAgg')

# --- 配置 ---
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001
SAVE_FILE = "sensor_data_from_relay.csv"
MAX_PACKETS_IN_MEMORY = 2000
BUFFER_SIZE = 1024 * 8
MAX_POINTS_PER_PACKET = 1000
DEFAULT_PLOT_WINDOW_PACKETS = 20
# ------------

# --- 全局变量 ---
is_recording = False
lock = threading.Lock()
full_history_s1 = []
full_history_s2 = []
full_history_diff = []
new_data_arrived = threading.Event()
global_time_offset = 0
last_packet_max_timestamp = 0
# ---

# --- Matplotlib 界面设置 ---
fig, (ax1, ax4) = plt.subplots(2, 1, figsize=(15, 10), gridspec_kw={'height_ratios': [4, 3]})
fig.subplots_adjust(bottom=0.2)
fig.canvas.manager.set_window_title('Interactive Dual-Channel Pressure Sensor Monitor (v5 - Fixed)')
ax1.set_xlabel('Time (ms)');
ax1.set_ylabel('Pressure (hPa)', color='blue');
ax1.grid(True, linestyle='--');
ax1.tick_params(axis='y', labelcolor='blue');
ax1.set_title('Real-time Pressure Monitoring')
sensor1_line, = ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
ax1.legend(loc='upper left')
stats_text = ax1.text(1.01, 0.95, '', transform=ax1.transAxes, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
ax4.set_xlabel('Time (ms)');
ax4.set_ylabel('Pressure Difference (S1-S2) hPa', color='green');
ax4.grid(True, linestyle='--');
ax4.tick_params(axis='y', labelcolor='green')
diff_pressure_line_only, = ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
ax4.legend(loc='upper right')
ax_button = plt.axes([0.02, 0.9, 0.1, 0.05])
record_button = Button(ax_button, 'Start Recording', color='lightgray', hovercolor='0.8')
recording_text = fig.text(0.13, 0.92, 'Recording: OFF', color='red', fontweight='bold')
ax_slider = plt.axes([0.1, 0.05, 0.8, 0.03])
packet_range_slider = RangeSlider(ax=ax_slider, label='Packet Range', valmin=0, valmax=1, valinit=(0, 1), valstep=1)


# ---

def redraw_plots():
    """根据范围滑块选择的区间，重新绘制所有图表。"""
    with lock:
        total_packets = len(full_history_s1)
        if total_packets == 0: return

        min_idx, max_idx = packet_range_slider.val
        start_idx, end_idx = int(min_idx), int(max_idx)

        # 确保索引有效
        if start_idx > max_idx or start_idx >= total_packets: return

        # 切片时，结束索引需要+1
        s1_packets_to_show = full_history_s1[start_idx: end_idx + 1]
        s2_packets_to_show = full_history_s2[start_idx: end_idx + 1]
        diff_packets_to_show = full_history_diff[start_idx: end_idx + 1]

    if not s1_packets_to_show: return

    s1_ts, s1_p = np.concatenate([p[0] for p in s1_packets_to_show]), np.concatenate([p[1] for p in s1_packets_to_show])
    s2_ts, s2_p = np.concatenate([p[0] for p in s2_packets_to_show]), np.concatenate([p[1] for p in s2_packets_to_show])
    diff_ts, diff_p_val = np.concatenate([p[0] for p in diff_packets_to_show]), np.concatenate([p[1] for p in diff_packets_to_show])

    sensor1_line.set_data(s1_ts, s1_p);
    sensor2_line.set_data(s2_ts, s2_p);
    diff_pressure_line_only.set_data(diff_ts, diff_p_val)

    all_ts = np.concatenate((s1_ts, s2_ts))
    if all_ts.size > 0:
        ax1.set_xlim(all_ts.min(), all_ts.max());
        ax4.set_xlim(all_ts.min(), all_ts.max())

    all_p = np.concatenate((s1_p, s2_p))
    if all_p.size > 0:
        p_margin = max((all_p.max() - all_p.min()) * 0.1, 0.1);
        ax1.set_ylim(all_p.min() - p_margin, all_p.max() + p_margin)

    if diff_p_val.size > 0:
        diff_margin = max((diff_p_val.max() - diff_p_val.min()) * 0.1, 0.1);
        ax4.set_ylim(diff_p_val.min() - diff_margin, diff_p_val.max() + diff_margin)

    last_s1_p, last_s2_p = s1_packets_to_show[-1][1], s2_packets_to_show[-1][1]
    stats_text.set_text(f"Sensor 1 (last pkt):\nMin: {last_s1_p.min():.2f}\nMax: {last_s1_p.max():.2f}\n\nSensor 2 (last pkt):\nMin: {last_s2_p.min():.2f}\nMax: {last_s2_p.max():.2f}")

    fig.canvas.draw_idle()


def on_range_slider_update(val):
    redraw_plots()


packet_range_slider.on_changed(on_range_slider_update)


def toggle_recording(event):
    global is_recording;
    is_recording = not is_recording
    if is_recording:
        record_button.label.set_text('Stop Recording'); recording_text.set_text('Recording: ON'); recording_text.set_color('green')
    else:
        record_button.label.set_text('Start Recording'); recording_text.set_text('Recording: OFF'); recording_text.set_color('red')
    fig.canvas.draw_idle()


record_button.on_clicked(toggle_recording)


def process_data(data, num_points):
    # (此函数与v4版本完全相同，无需修改)
    global global_time_offset, last_packet_max_timestamp
    _id, first_timestamp_in_packet, _, _ = struct.unpack("!BIff", data[0:13])
    if last_packet_max_timestamp > 0: global_time_offset = last_packet_max_timestamp - first_timestamp_in_packet + 20
    sensor1_points_adjusted, sensor2_points_adjusted = [], []
    current_packet_max_ts = 0
    for i in range(num_points):
        point_data = data[i * 13: (i + 1) * 13];
        sensor_id, timestamp, temp, pressure = struct.unpack("!BIff", point_data)
        adjusted_timestamp = timestamp + global_time_offset
        (sensor1_points_adjusted if sensor_id == 1 else sensor2_points_adjusted).append((adjusted_timestamp, pressure))
        if adjusted_timestamp > current_packet_max_ts: current_packet_max_ts = adjusted_timestamp
    if current_packet_max_ts > 0: last_packet_max_timestamp = current_packet_max_ts
    if not sensor1_points_adjusted or not sensor2_points_adjusted: return
    s1_ts, s1_p = np.array(sensor1_points_adjusted).T;
    s2_ts, s2_p = np.array(sensor2_points_adjusted).T
    min_len = min(len(s1_p), len(s2_p));
    diff_ts, diff_p = s1_ts[:min_len], s1_p[:min_len] - s2_p[:min_len]
    with lock:
        full_history_s1.append((s1_ts, s1_p));
        full_history_s2.append((s2_ts, s2_p));
        full_history_diff.append((diff_ts, diff_p))
        if len(full_history_s1) > MAX_PACKETS_IN_MEMORY:
            full_history_s1.pop(0);
            full_history_s2.pop(0);
            full_history_diff.pop(0)
    print(f"Processed packet #{len(full_history_s1)}. Global time ends at: {last_packet_max_timestamp} ms")
    new_data_arrived.set()


def start_data_reception():
    # (此函数与v4版本完全相同，无需修改)
    while True:
        try:
            print(f"Connecting to relay server at {RELAY_SERVER_IP}:{RELAY_SERVER_PORT}...")
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
            conn.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
            print("Successfully connected. Waiting for data...")
            while True:
                header = conn.recv(4)
                if not header: break
                num_points = struct.unpack("!I", header)[0]
                if not (0 < num_points <= MAX_POINTS_PER_PACKET):
                    print(f"Abnormal packet size: {num_points}. Disconnecting...");
                    break
                data = b'';
                bytes_to_read = num_points * 13
                while len(data) < bytes_to_read:
                    chunk = conn.recv(min(BUFFER_SIZE, bytes_to_read - len(data)))
                    if not chunk: raise ConnectionError("Relay connection lost.")
                    data += chunk
                process_data(data, num_points)
        except Exception as e:
            print(f"Connection error: {e}. Retrying in 5s..."); time.sleep(5)
        finally:
            if 'conn' in locals(): conn.close()


# ===================================================================
# =====          vvv   关键修正区域 vvv                       =====
# ===================================================================
def gui_updater():
    """主线程中的GUI更新器，适配RangeSlider。"""
    if new_data_arrived.is_set():
        new_data_arrived.clear()

        with lock:
            total_packets = len(full_history_s1)
            if total_packets == 0:
                # 重新注册定时器并返回
                fig.canvas.new_timer(interval=500).add_callback(gui_updater)
                return

            # --- BUG FIX 1: 修正滑块最大值的计算 ---
            new_valmax = total_packets - 1
            if new_valmax < packet_range_slider.val[1]:  # 确保max不小于min
                new_valmax = packet_range_slider.val[1]

            packet_range_slider.valmax = new_valmax
            packet_range_slider.ax.set_xlim(0, new_valmax)

            current_min, current_max = packet_range_slider.val
            # 放宽“跟随模式”的判断，只要右侧滑块接近末端即可
            is_following = (current_max >= packet_range_slider.valmax - 1.0)

            if is_following:
                window_width = current_max - current_min
                new_max = total_packets - 1
                new_min = max(0, new_max - window_width)

                # 更新滑块值会通过on_changed事件自动触发重绘
                # 增加一个cid（回调ID）的管理，可以防止在程序化更新时触发事件，但目前的设计可以接受事件触发
                packet_range_slider.set_val((new_min, new_max))
            else:
                # 即使不跟随，也需要刷新一下画布以正确显示更新后的滑块范围
                fig.canvas.draw_idle()

    # --- BUG FIX 2: 重新注册定时器以实现循环执行 ---
    # 无论是否有新数据，都注册下一次的回调
    fig.canvas.new_timer(interval=500).add_callback(gui_updater)


# ===================================================================
# =====          ^^^   关键修正区域 ^^^                       =====
# ===================================================================

if __name__ == "__main__":
    receiver_thread = threading.Thread(target=start_data_reception, daemon=True)
    receiver_thread.start()

    # --- 修正: 启动第一个定时器 ---
    # 使用fig.canvas.new_timer来创建一个一次性的定时器，
    # 它的回调函数(gui_updater)会负责创建下一个定时器，形成循环。
    print("GUI Initialized. Close the plot window to exit.")
    fig.canvas.new_timer(interval=500, callbacks=[(gui_updater, [], {})]).start()

    plt.show()
    print("Application closed.")