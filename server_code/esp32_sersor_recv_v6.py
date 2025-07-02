import socket
import struct
import csv
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.font_manager import FontProperties
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
last_packet_stats = {
    'sensor1': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0},
    'sensor2': {'min': 0, 'max': 0, 'diff': 0, 'freq': 0}
}
lock = threading.Lock()

# Setup plot
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 9
fig, ax = plt.subplots(figsize=(12, 6))
fig.canvas.manager.set_window_title('Dual-Channel Pressure Sensor Monitor')
ax.set_xlabel('Time (ms)')
ax.set_ylabel('Pressure (hPa)')
ax.grid(True, linestyle='--', alpha=0.6)
sensor1_line, = ax.plot([], [], 'b-', label='Sensor 1', alpha=0.8)
sensor2_line, = ax.plot([], [], 'r-', label='Sensor 2', alpha=0.8)
stats_text = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                     verticalalignment='top', horizontalalignment='right',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
ax.legend(loc='upper left')
ax.set_title('Real-time Pressure Monitoring')

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
            # print(f"Receiving {num_points} points")

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

    # Add to global buffers
    with lock:
        sensor1_data.extend(sensor1_points)
        sensor2_data.extend(sensor2_points)


def update_plot(frame):
    with lock:
        # Get data for plotting
        s1_ts, s1_p = zip(*sensor1_data) if sensor1_data else ([], [])
        s2_ts, s2_p = zip(*sensor2_data) if sensor2_data else ([], [])

        # Update line data
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

        # Update plot limits
        all_ts = list(s1_ts) + list(s2_ts)
        all_p = list(s1_p) + list(s2_p)

        if all_ts:
            min_ts = min(all_ts)
            max_ts = max(all_ts)
            ax.set_xlim(min_ts, max_ts)

            min_p = min(all_p) * 0.995
            max_p = max(all_p) * 1.005
            ax.set_ylim(min_p, max_p)

    return sensor1_line, sensor2_line, stats_text


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
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.daemon = True
            client_thread.start()

    accept_thread = threading.Thread(target=accept_clients)
    accept_thread.daemon = True
    accept_thread.start()

    # Start the plot animation
    ani = animation.FuncAnimation(fig, update_plot, interval=200, blit=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    start_server()
