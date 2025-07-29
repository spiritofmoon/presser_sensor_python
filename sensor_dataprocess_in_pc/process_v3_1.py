# main_qt_v3_interactive.py
# 修复了 NameError: name 'lock' is not defined 的问题

import sys
import socket
import struct
import threading
import time
import numpy as np
from PySide6 import QtWidgets, QtCore, QtGui

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- 配置 ---
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001
MAX_PACKETS_IN_MEMORY = 2000
BUFFER_SIZE = 1024 * 8
MAX_POINTS_PER_PACKET = 1000


# ------------

# --- 线程通信与网络工作线程 (无变动) ---
class WorkerSignals(QtCore.QObject):
    new_packet = QtCore.Signal(tuple)


class NetworkWorker(QtCore.QRunnable):
    def __init__(self):
        super().__init__()
        self.signals = WorkerSignals()
        self.running = True

    @QtCore.Slot()
    def run(self):
        while self.running:
            try:
                print("Connecting to relay server...")
                conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
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
                print(f"Connection error: {e}. Retrying in 5s...");
                time.sleep(5)
            finally:
                if 'conn' in locals(): conn.close()

    def stop(self):
        self.running = False


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, constrained_layout=True)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[4, 3])
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax4 = self.fig.add_subplot(gs[1, 0], sharex=self.ax1)
        super().__init__(self.fig)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interactive Sensor Monitor with Controls")
        self.setGeometry(100, 100, 1400, 800)

        self.full_history_s1, self.full_history_s2, self.full_history_diff = [], [], []
        self.global_time_offset, self.last_packet_max_timestamp = 0, 0
        self.is_paused = False

        self._setup_ui()
        self.threadpool = QtCore.QThreadPool()
        self.start_network_worker()

    def _setup_ui(self):
        # (UI布局代码无变动)
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        self.canvas = MplCanvas(self)
        main_layout.addWidget(self.canvas, 8)
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        control_panel.setFixedWidth(250)
        main_layout.addWidget(control_panel, 2)
        window_group = QtWidgets.QGroupBox("Display Window")
        window_layout = QtWidgets.QFormLayout(window_group)
        self.window_size_spinbox = QtWidgets.QSpinBox()
        self.window_size_spinbox.setRange(1, 500)
        self.window_size_spinbox.setValue(20)
        window_layout.addRow("Packets to Show:", self.window_size_spinbox)
        control_layout.addWidget(window_group)
        scroll_group = QtWidgets.QGroupBox("History Scroll")
        scroll_layout = QtWidgets.QVBoxLayout(scroll_group)
        self.scroll_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.scroll_slider.setRange(0, 0)
        scroll_layout.addWidget(self.scroll_slider)
        control_layout.addWidget(scroll_group)
        control_layout.addStretch()
        self.window_size_spinbox.valueChanged.connect(self.update_slider_range_and_redraw)
        self.scroll_slider.valueChanged.connect(self.redraw_plots)
        self.canvas.ax1.set_title('Real-time Pressure Monitoring')
        self.canvas.ax1.set_ylabel('Pressure (hPa)')
        self.canvas.ax1.grid(True, linestyle='--')
        self.sensor1_line, = self.canvas.ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
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
        # (数据处理逻辑无变动)
        data, num_points = packet_data
        _id, first_ts, _, _ = struct.unpack("!BIff", data[0:13]);
        if self.last_packet_max_timestamp > 0: self.global_time_offset = self.last_packet_max_timestamp - first_ts + 20
        s1_adj, s2_adj, max_ts = [], [], 0
        for i in range(num_points):
            _d = data[i * 13:(i + 1) * 13];
            sid, ts, _, p = struct.unpack("!BIff", _d)
            adj_ts = ts + self.global_time_offset;
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

        self.update_ui_on_new_packet()

    def update_ui_on_new_packet(self):
        was_following = (self.scroll_slider.value() == self.scroll_slider.maximum())
        self.update_slider_range()
        if was_following:
            self.scroll_slider.setValue(self.scroll_slider.maximum())

    def update_slider_range_and_redraw(self):
        self.update_slider_range()
        self.redraw_plots()

    def update_slider_range(self):
        with QtCore.QSignalBlocker(self.scroll_slider):
            total_packets = len(self.full_history_s1)
            window_size = self.window_size_spinbox.value()
            new_max = max(0, total_packets - window_size)
            self.scroll_slider.setMaximum(new_max)

    def redraw_plots(self):
        """根据控件当前状态，切片数据并重绘图表"""
        # --- 关键修正：移除不再需要的 with lock: ---
        if not self.full_history_s1: return

        window_size = self.window_size_spinbox.value()
        start_idx = self.scroll_slider.value()
        end_idx = start_idx + window_size

        total_packets = len(self.full_history_s1)
        if start_idx >= total_packets: return
        end_idx = min(end_idx, total_packets)

        s1_slice = self.full_history_s1[start_idx:end_idx]
        s2_slice = self.full_history_s2[start_idx:end_idx]
        diff_slice = self.full_history_diff[start_idx:end_idx]

        if not s1_slice: return

        s1_ts, s1_p = np.concatenate([p[0] for p in s1_slice]), np.concatenate([p[1] for p in s1_slice])
        s2_ts, s2_p = np.concatenate([p[0] for p in s2_slice]), np.concatenate([p[1] for p in s2_slice])
        diff_ts, diff_p = np.concatenate([p[0] for p in diff_slice]), np.concatenate([p[1] for p in diff_slice])

        self.sensor1_line.set_data(s1_ts, s1_p);
        self.sensor2_line.set_data(s2_ts, s2_p);
        self.diff_line.set_data(diff_ts, diff_p)

        self.canvas.ax1.relim();
        self.canvas.ax1.autoscale_view()
        self.canvas.ax4.relim();
        self.canvas.ax4.autoscale_view()
        self.canvas.draw()

    def closeEvent(self, event):
        print("Closing application...");
        self.network_worker.stop();
        self.threadpool.waitForDone();
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())