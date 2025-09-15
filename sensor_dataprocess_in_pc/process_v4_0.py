# process_v4_0_with_mic.py
# 在v3.2版本基础上，增加了第三个图表，用于可视化两个ICS43434麦克风的音频数据

import sys
import socket
import struct
import csv
import threading
import time
import numpy as np
from PySide6 import QtWidgets, QtCore, QtGui

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- 配置 (无变动) ---
RELAY_SERVER_IP = "117.72.10.210"
RELAY_SERVER_PORT = 5001
MAX_PACKETS_IN_MEMORY = 2000
BUFFER_SIZE = 1024 * 8
MAX_POINTS_PER_PACKET = 1000  # 注意：服务器转发的数据包理论上可能很大，这里用作数据校验


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
                conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                conn.connect((RELAY_SERVER_IP, RELAY_SERVER_PORT))
                print("Successfully connected.")
                while self.running:
                    header = conn.recv(4)
                    if not header: break
                    num_points = struct.unpack("!I", header)[0]
                    # 增加一个更宽松的校验，防止num_points异常大导致内存问题
                    if not (0 < num_points <= MAX_POINTS_PER_PACKET * 400): break
                    data = b''
                    bytes_to_read = num_points * 13  # 每个数据点 1B ID + 4B TS + 4B F1 + 4B F2 = 13 Bytes
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
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, constrained_layout=True)
        # ===================================================================
        # =====          vvv   修改UI：增加一个图表区域 vvv             =====
        # ===================================================================
        # 将网格从2x1改为3x1，为麦克风数据腾出空间
        gs = self.fig.add_gridspec(3, 1, height_ratios=[4, 3, 3])
        self.ax1 = self.fig.add_subplot(gs[0, 0])  # 压力图
        self.ax4 = self.fig.add_subplot(gs[1, 0], sharex=self.ax1)  # 压力差图
        self.ax_mic = self.fig.add_subplot(gs[2, 0], sharex=self.ax1)  # 新增的麦克风图
        # ===================================================================
        # =====          ^^^   修改UI：增加一个图表区域 ^^^             =====
        # ===================================================================
        super().__init__(self.fig)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interactive Sensor Monitor with Export (4 Channels)")
        self.setGeometry(100, 100, 1400, 900)  # 增加了窗口高度以容纳新图表

        # ===================================================================
        # =====       vvv   修改数据存储：为麦克风增加历史列表 vvv        =====
        # ===================================================================
        self.full_history_s1, self.full_history_s2, self.full_history_diff = [], [], []
        self.full_history_s3, self.full_history_s4 = [], []  # 新增S3, S4 (麦克风)
        # ===================================================================
        # =====       ^^^   修改数据存储：为麦克风增加历史列表 ^^^        =====
        # ===================================================================
        self.global_time_offset, self.last_packet_max_timestamp = 0, 0

        self._setup_ui()
        self.threadpool = QtCore.QThreadPool()
        self.start_network_worker()

    def _setup_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)

        self.canvas = MplCanvas(self)
        main_layout.addWidget(self.canvas, 8)

        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        control_panel.setFixedWidth(250)
        main_layout.addWidget(control_panel, 2)

        # (控制面板UI无变动)
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

        export_group = QtWidgets.QGroupBox("Data Export")
        export_layout = QtWidgets.QVBoxLayout(export_group)
        self.save_button = QtWidgets.QPushButton("Save Current View to CSV")
        self.save_button.setToolTip("Saves the exact data currently shown on the plots to a new CSV file.")
        export_layout.addWidget(self.save_button)
        control_layout.addWidget(export_group)
        self.save_button.clicked.connect(self.save_current_view)
        control_layout.addStretch()

        self.window_size_spinbox.valueChanged.connect(self.update_slider_range_and_redraw)
        self.scroll_slider.valueChanged.connect(self.redraw_plots)

        # --- 初始化绘图元素 ---
        # 压力图 (ax1)
        self.canvas.ax1.set_title('Real-time Pressure Monitoring')
        self.canvas.ax1.set_ylabel('Pressure (hPa)')
        self.canvas.ax1.grid(True, linestyle='--')
        self.sensor1_line, = self.canvas.ax1.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
        self.sensor2_line, = self.canvas.ax1.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
        self.canvas.ax1.legend()
        # 压力差图 (ax4)
        self.canvas.ax4.set_ylabel('Pressure Diff (hPa)')
        self.canvas.ax4.grid(True, linestyle='--')
        self.diff_line, = self.canvas.ax4.plot([], [], 'g-', label='Pressure Diff (S1-S2)', alpha=0.8)
        self.canvas.ax4.legend()

        # ===================================================================
        # =====       vvv   修改UI：初始化新的麦克风图表 vvv            =====
        # ===================================================================
        # 麦克风图 (ax_mic)
        self.canvas.ax_mic.set_title('Microphone Audio Signal')
        self.canvas.ax_mic.set_ylabel('Amplitude')
        self.canvas.ax_mic.set_xlabel('Time (ms)')  # X轴标签移至最下方的图表
        self.canvas.ax_mic.grid(True, linestyle='--')
        self.mic1_line, = self.canvas.ax_mic.plot([], [], 'c-', label='Mic 1 (S3)', alpha=0.8)
        self.mic2_line, = self.canvas.ax_mic.plot([], [], 'm-', label='Mic 2 (S4)', alpha=0.8)
        self.canvas.ax_mic.legend()
        # ===================================================================
        # =====       ^^^   修改UI：初始化新的麦克风图表 ^^^            =====
        # ===================================================================


    def start_network_worker(self):
        self.network_worker = NetworkWorker()
        self.network_worker.signals.new_packet.connect(self.process_packet_slot)
        self.threadpool.start(self.network_worker)

    @QtCore.Slot(tuple)
    def process_packet_slot(self, packet_data):
        data, num_points = packet_data
        _id, first_ts, _, _ = struct.unpack("!BIff", data[0:13]);
        if self.last_packet_max_timestamp > 0: self.global_time_offset = self.last_packet_max_timestamp - first_ts + 20

        # ===================================================================
        # =====   vvv   修改数据处理：增加对S3, S4数据的分类 vvv        =====
        # ===================================================================
        s1_adj, s2_adj, s3_adj, s4_adj, max_ts = [], [], [], [], 0
        for i in range(num_points):
            _d = data[i * 13:(i + 1) * 13]
            sid, ts, _, p = struct.unpack("!BIff", _d)
            adj_ts = ts + self.global_time_offset

            if sid == 1:
                s1_adj.append((adj_ts, p))
            elif sid == 2:
                s2_adj.append((adj_ts, p))
            elif sid == 3:
                s3_adj.append((adj_ts, p))  # p 此处是音频振幅
            elif sid == 4:
                s4_adj.append((adj_ts, p))  # p 此处是音频振幅

            if adj_ts > max_ts: max_ts = adj_ts

        if max_ts > 0: self.last_packet_max_timestamp = max_ts

        # 为了保证所有历史列表长度一致（对齐滑动条），每个数据包都必须为所有传感器追加数据
        # 如果某个传感器在此数据包中无数据，则追加一个空的数据点元组
        for history_list, adj_list in [
            (self.full_history_s1, s1_adj), (self.full_history_s2, s2_adj),
            (self.full_history_s3, s3_adj), (self.full_history_s4, s4_adj)
        ]:
            if adj_list:
                ts_arr, p_arr = np.array(adj_list).T
                history_list.append((ts_arr, p_arr))
            else:
                history_list.append((np.array([]), np.array([])))

        # 压力差的计算逻辑保持不变，但需检查s1和s2是否有数据
        if s1_adj and s2_adj:
            s1_ts, s1_p = self.full_history_s1[-1]
            s2_ts, s2_p = self.full_history_s2[-1]
            min_len = min(len(s1_p), len(s2_p))
            diff_ts, diff_p = s1_ts[:min_len], s1_p[:min_len] - s2_p[:min_len]
            self.full_history_diff.append((diff_ts, diff_p))
        else:
            self.full_history_diff.append((np.array([]), np.array([])))

        # 限制内存中的数据包数量
        if len(self.full_history_s1) > MAX_PACKETS_IN_MEMORY:
            self.full_history_s1.pop(0)
            self.full_history_s2.pop(0)
            self.full_history_diff.pop(0)
            self.full_history_s3.pop(0)
            self.full_history_s4.pop(0)
        # ===================================================================
        # =====   ^^^   修改数据处理：增加对S3, S4数据的分类 ^^^        =====
        # ===================================================================

        self.update_ui_on_new_packet()

    def update_ui_on_new_packet(self):  # (无变动)
        was_following = (self.scroll_slider.value() == self.scroll_slider.maximum())
        self.update_slider_range()
        if was_following:
            self.scroll_slider.setValue(self.scroll_slider.maximum())

    def update_slider_range_and_redraw(self):  # (无变动)
        self.update_slider_range()
        self.redraw_plots()

    def update_slider_range(self):  # (无变动)
        with QtCore.QSignalBlocker(self.scroll_slider):
            total_packets = len(self.full_history_s1)
            window_size = self.window_size_spinbox.value()
            new_max = max(0, total_packets - window_size)
            self.scroll_slider.setMaximum(new_max)

    def redraw_plots(self):
        if not self.full_history_s1: return
        window_size = self.window_size_spinbox.value()
        start_idx = self.scroll_slider.value()
        end_idx = start_idx + window_size
        total_packets = len(self.full_history_s1)
        if start_idx >= total_packets: return
        end_idx = min(end_idx, total_packets)

        # ===================================================================
        # =====       vvv   修改绘图：获取并绘制麦克风数据 vvv          =====
        # ===================================================================
        s1_slice = self.full_history_s1[start_idx:end_idx]
        s2_slice = self.full_history_s2[start_idx:end_idx]
        diff_slice = self.full_history_diff[start_idx:end_idx]
        s3_slice = self.full_history_s3[start_idx:end_idx]  # 获取麦克风数据切片
        s4_slice = self.full_history_s4[start_idx:end_idx]

        # 安全地连接数据，即使某些包为空
        s1_ts, s1_p = np.concatenate([p[0] for p in s1_slice if p[0].size > 0]), np.concatenate([p[1] for p in s1_slice if p[1].size > 0])
        s2_ts, s2_p = np.concatenate([p[0] for p in s2_slice if p[0].size > 0]), np.concatenate([p[1] for p in s2_slice if p[1].size > 0])
        diff_ts, diff_p = np.concatenate([p[0] for p in diff_slice if p[0].size > 0]), np.concatenate([p[1] for p in diff_slice if p[1].size > 0])
        s3_ts, s3_p = np.concatenate([p[0] for p in s3_slice if p[0].size > 0]), np.concatenate([p[1] for p in s3_slice if p[1].size > 0])
        s4_ts, s4_p = np.concatenate([p[0] for p in s4_slice if p[0].size > 0]), np.concatenate([p[1] for p in s4_slice if p[1].size > 0])

        self.sensor1_line.set_data(s1_ts, s1_p)
        self.sensor2_line.set_data(s2_ts, s2_p)
        self.diff_line.set_data(diff_ts, diff_p)
        self.mic1_line.set_data(s3_ts, s3_p)  # 更新麦克风曲线
        self.mic2_line.set_data(s4_ts, s4_p)

        self.canvas.ax1.relim()
        self.canvas.ax1.autoscale_view()
        self.canvas.ax4.relim()
        self.canvas.ax4.autoscale_view()
        self.canvas.ax_mic.relim()
        self.canvas.ax_mic.autoscale_view()  # 自动调整新图表的坐标轴
        # ===================================================================
        # =====       ^^^   修改绘图：获取并绘制麦克风数据 ^^^          =====
        # ===================================================================
        self.canvas.draw()

    @QtCore.Slot()
    def save_current_view(self):
        default_filename = f"sensor_view_{int(time.time())}.csv"
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Current View as CSV", default_filename, "CSV Files (*.csv);;All Files (*)")
        if not file_path:
            print("Save cancelled by user.")
            return

        window_size = self.window_size_spinbox.value()
        start_idx = self.scroll_slider.value()
        end_idx = start_idx + window_size
        total_packets = len(self.full_history_s1)
        if start_idx >= total_packets:
            QtWidgets.QMessageBox.warning(self, "Warning", "No data in the current view to save.")
            return
        end_idx = min(end_idx, total_packets)

        # ===================================================================
        # =====       vvv   修改保存：将麦克风数据写入CSV vvv           =====
        # ===================================================================
        s1_slice = self.full_history_s1[start_idx:end_idx]
        s2_slice = self.full_history_s2[start_idx:end_idx]
        s3_slice = self.full_history_s3[start_idx:end_idx]  # 获取麦克风数据
        s4_slice = self.full_history_s4[start_idx:end_idx]

        try:
            print(f"Saving current view to {file_path}...")
            # 安全地展开数据点列表
            s1_points = np.concatenate([np.column_stack(p) for p in s1_slice if p[0].size > 0]) if any(p[0].size > 0 for p in s1_slice) else np.array([])
            s2_points = np.concatenate([np.column_stack(p) for p in s2_slice if p[0].size > 0]) if any(p[0].size > 0 for p in s2_slice) else np.array([])
            s3_points = np.concatenate([np.column_stack(p) for p in s3_slice if p[0].size > 0]) if any(p[0].size > 0 for p in s3_slice) else np.array([])
            s4_points = np.concatenate([np.column_stack(p) for p in s4_slice if p[0].size > 0]) if any(p[0].size > 0 for p in s4_slice) else np.array([])

            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 使用更通用的表头 "value"，因为它现在既代表压力也代表振幅
                writer.writerow(['timestamp', 'sensor_id', 'value'])

                for row in s1_points: writer.writerow([row[0], 1, row[1]])
                for row in s2_points: writer.writerow([row[0], 2, row[1]])
                for row in s3_points: writer.writerow([row[0], 3, row[1]])
                for row in s4_points: writer.writerow([row[0], 4, row[1]])

            QtWidgets.QMessageBox.information(self, "Success", f"Data successfully saved to:\n{file_path}")
            print("Save complete.")

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save file.\nError: {e}")
            print(f"Error during save: {e}")
        # ===================================================================
        # =====       ^^^   修改保存：将麦克风数据写入CSV ^^^           =====
        # ===================================================================

    def closeEvent(self, event):  # (无变动)
        print("Closing application...")
        self.network_worker.stop()
        self.threadpool.waitForDone()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())