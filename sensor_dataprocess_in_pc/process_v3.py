# main_qt_v2.py
# 修复了只绘制一条折线的问题，并完善了UI更新逻辑

import sys
import socket
import struct
import threading
import time
import numpy as np
from PySide6 import QtWidgets, QtCore, QtGui

# Matplotlib 与 Qt 的集成组件
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- 配置 ---
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001
MAX_PACKETS_IN_MEMORY = 2000
BUFFER_SIZE = 1024 * 8
MAX_POINTS_PER_PACKET = 1000


# ------------

# --- 使用 Qt 的信号与槽进行线程通信 ---
class WorkerSignals(QtCore.QObject):
    """定义一个信号类，用于从后台线程向主GUI线程发送数据。"""
    new_packet = QtCore.Signal(tuple)


class NetworkWorker(QtCore.QRunnable):
    """网络接收线程。"""

    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.running = True

    @QtCore.Slot()
    def run(self):
        while self.running:
            try:
                print("Connecting to relay server...")
                conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                conn.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
                print("Successfully connected.")
                while self.running:
                    header = conn.recv(4)
                    if not header: break
                    num_points = struct.unpack("!I", header)[0]
                    if not (0 < num_points <= MAX_POINTS_PER_PACKET): break
                    data = b'';
                    bytes_to_read = num_points * 13
                    while len(data) < bytes_to_read:
                        chunk = conn.recv(bytes_to_read - len(data))
                        if not chunk: raise ConnectionError("Relay connection lost.")
                        data += chunk
                    self.signals.new_packet.emit((data, num_points))
            except Exception as e:
                print(f"Connection error: {e}. Retrying in 5s...")
                time.sleep(5)
            finally:
                if 'conn' in locals(): conn.close()

    def stop(self):
        self.running = False


class MplCanvas(FigureCanvas):
    """一个自定义的 Matplotlib 画布 Widget"""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, constrained_layout=True)
        # 定义两个子图，ax1在上，ax4在下
        gs = self.fig.add_gridspec(2, 1, height_ratios=[4, 3])
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax4 = self.fig.add_subplot(gs[1, 0], sharex=self.ax1)  # 共享X轴
        super().__init__(self.fig)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt + Matplotlib Sensor Monitor (Fixed)")
        self.setGeometry(100, 100, 1200, 800)

        # 数据存储
        self.full_history_s1 = []
        self.full_history_s2 = []
        self.full_history_diff = []
        self.global_time_offset = 0
        self.last_packet_max_timestamp = 0

        # 初始化UI
        self._setup_ui()

        # 初始化网络线程
        self.threadpool = QtCore.QThreadPool()
        self.start_network_worker()

    def _setup_ui(self):
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        self.canvas = MplCanvas(self)
        main_layout.addWidget(self.canvas)

        # 提示：真实的范围选择需要自定义一个双滑块控件，这里为了演示，功能简化
        # Qt原生的QSlider只有一个滑块
        # ... 可以添加按钮、滑块等控件 ...

        # --- 初始化绘图元素 ---
        self.canvas.ax1.set_title('Real-time Pressure Monitoring')
        self.canvas.ax1.set_ylabel('Pressure (hPa)')
        self.canvas.ax1.grid(True, linestyle='--')

        self.sensor1_line, = self.canvas.ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
        # --- FIX 1: 修正ax2错字为ax1 ---
        self.sensor2_line, = self.canvas.ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
        self.canvas.ax1.legend()

        self.canvas.ax4.set_xlabel('Time (ms)')
        self.canvas.ax4.set_ylabel('Pressure Difference (S1-S2) hPa')
        self.canvas.ax4.grid(True, linestyle='--')
        self.diff_line, = self.canvas.ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
        self.canvas.ax4.legend()


    def start_network_worker(self):
        self.network_worker = NetworkWorker()
        self.network_worker.signals.new_packet.connect(self.process_packet_slot)
        self.threadpool.start(self.network_worker)

    @QtCore.Slot(tuple)
    def process_packet_slot(self, packet_data):
        """这个槽函数在主GUI线程中被安全地调用。"""
        data, num_points = packet_data

        _id, first_ts, _, _ = struct.unpack("!BIff", data[0:13])
        if self.last_packet_max_timestamp > 0:
            self.global_time_offset = self.last_packet_max_timestamp - first_ts + 20

        s1_adj, s2_adj, max_ts = [], [], 0
        for i in range(num_points):
            _d = data[i * 13:(i + 1) * 13];
            sid, ts, _, p = struct.unpack("!BIff", _d)
            adj_ts = ts + self.global_time_offset
            (s1_adj if sid == 1 else s2_adj).append((adj_ts, p))
            if adj_ts > max_ts: max_ts = adj_ts
        if max_ts > 0: self.last_packet_max_timestamp = max_ts

        if not s1_adj or not s2_adj: return

        s1_ts, s1_p = np.array(s1_adj).T;
        s2_ts, s2_p = np.array(s2_adj).T
        min_len = min(len(s1_p), len(s2_p));
        diff_ts, diff_p = s1_ts[:min_len], s1_p[:min_len] - s2_p[:min_len]

        self.full_history_s1.append((s1_ts, s1_p));
        self.full_history_s2.append((s2_ts, s2_p));
        self.full_history_diff.append((diff_ts, diff_p))

        if len(self.full_history_s1) > MAX_PACKETS_IN_MEMORY:
            self.full_history_s1.pop(0);
            self.full_history_s2.pop(0);
            self.full_history_diff.pop(0)

        self.redraw_plots()  # 收到新数据后直接重绘

    # ===================================================================
    # =====          vvv   关键修正区域 vvv                       =====
    # ===================================================================
    def redraw_plots(self):
        """重绘所有图表，此函数现在包含所有折线的更新逻辑"""
        if not self.full_history_s1: return

        # --- FIX 2: 补全所有数据的准备工作 ---
        s1_ts, s1_p = np.concatenate([p[0] for p in self.full_history_s1]), np.concatenate([p[1] for p in self.full_history_s1])
        s2_ts, s2_p = np.concatenate([p[0] for p in self.full_history_s2]), np.concatenate([p[1] for p in self.full_history_s2])
        diff_ts, diff_p = np.concatenate([p[0] for p in self.full_history_diff]), np.concatenate([p[1] for p in self.full_history_diff])

        # --- FIX 3: 补全所有折线的set_data调用 ---
        self.sensor1_line.set_data(s1_ts, s1_p)
        self.sensor2_line.set_data(s2_ts, s2_p)
        self.diff_line.set_data(diff_ts, diff_p)

        # --- FIX 4: 补全对两个子图坐标轴的更新 ---
        # 更新上方图(ax1)的坐标轴
        self.canvas.ax1.relim()
        self.canvas.ax1.autoscale_view()

        # 更新下方图(ax4)的坐标轴
        self.canvas.ax4.relim()
        self.canvas.ax4.autoscale_view()

        # 统一重绘整个画布
        self.canvas.draw()

    # ===================================================================
    # =====          ^^^   关键修正区域 ^^^                       =====
    # ===================================================================

    def closeEvent(self, event):
        """确保关闭窗口时，后台线程也能被干净地停止"""
        print("Closing application...")
        self.network_worker.stop()
        self.threadpool.waitForDone()  # 等待线程池完成
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())