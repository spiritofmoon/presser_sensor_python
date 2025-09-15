import socket
import struct
import time
import random
import math
import threading
import sys

# 服务器配置
SERVER_IP = "117.72.10.210"  # 修改为服务器IP
# SERVER_IP = "127.0.0.1"  # 修改为服务器IP
SERVER_PORT = 5000

# 模拟参数
SAMPLE_RATE = 100  # 100Hz采样率
SEND_INTERVAL = 0.1  # 每0.1秒发送一次数据
BUFFER_SIZE = 100  # 每次发送的数据点数（每个传感器）
PRESSURE_BASELINE = 101325.0  # 海平面标准大气压(Pa)
PRESSURE_AMPLITUDE = 100.0  # 气压变化幅度

# 全局变量
running = True


def generate_sensor_data(sensor_id):
    """生成模拟传感器数据"""
    data_points = []
    start_time = time.time()

    # 生成随机相位偏移
    phase_shift = random.uniform(0, 2 * math.pi)

    for i in range(BUFFER_SIZE):
        # 计算时间差（毫秒）
        time_diff = int((time.time() - start_time) * 1000)

        # 生成温度数据 (20-30°C之间波动)
        temperature = 25.0 + 5.0 * math.sin(time_diff / 1000 + phase_shift)

        # 生成气压数据（在基准值附近波动）
        pressure = PRESSURE_BASELINE + PRESSURE_AMPLITUDE * math.sin(time_diff / 500 + phase_shift)
        pressure += random.uniform(-5, 5)  # 添加随机噪声

        # 添加到数据点
        data_points.append((sensor_id, time_diff, temperature, pressure))

        # 等待达到采样率
        time.sleep(1 / SAMPLE_RATE)

    return data_points


def pack_data(sensor1_points, sensor2_points):
    """打包数据用于发送"""
    # 合并两个传感器的数据点
    all_points = sensor1_points + sensor2_points

    # 打包数据
    # 包头：4字节总数据点数
    data = struct.pack("!I", len(all_points))

    # 打包每个数据点
    for sensor_id, time_diff, temperature, pressure in all_points:
        # 每个点：1字节传感器ID + 4字节时间差 + 4字节温度 + 4字节气压
        data += struct.pack("!BIff", sensor_id, time_diff, temperature, pressure)

    return data


def send_data_thread():
    """数据发送线程"""
    print(f"Connecting to server at {SERVER_IP}:{SERVER_PORT}")

    try:
        while running:
            try:
                # 创建TCP套接字
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((SERVER_IP, SERVER_PORT))

                print("Connected to server. Starting data transmission...")

                while running:
                    # 生成传感器1数据
                    sensor1_data = generate_sensor_data(1)

                    # 生成传感器2数据
                    sensor2_data = generate_sensor_data(2)

                    # 打包数据
                    packed_data = pack_data(sensor1_data, sensor2_data)

                    # 发送数据
                    sock.sendall(packed_data)
                    print(f"Sent {len(packed_data)} bytes of data")

                    # 等待下一个发送周期
                    time.sleep(SEND_INTERVAL)

            except (socket.error, ConnectionResetError) as e:
                print(f"Connection error: {e}. Reconnecting in 3 seconds...")
                time.sleep(3)
            finally:
                if 'sock' in locals():
                    sock.close()

    except KeyboardInterrupt:
        print("Data transmission stopped by user")


def print_help():
    """打印帮助信息"""
    print("Dual-Channel Pressure Sensor Simulator")
    print("Usage: python sensor_simulator.py [options]")
    print("Options:")
    print("  --ip <address>   Set server IP address (default: 127.0.0.1)")
    print("  --port <port>    Set server port (default: 5000)")
    print("  --rate <hz>      Set sample rate (default: 100)")
    print("  --interval <s>   Set send interval (default: 0.1)")
    print("  --amplitude <pa> Set pressure amplitude (default: 100)")
    print("  --help           Show this help message")


def main():
    global SERVER_IP, SERVER_PORT, SAMPLE_RATE, SEND_INTERVAL, PRESSURE_AMPLITUDE

    # 解析命令行参数
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--ip" and i + 1 < len(args):
            SERVER_IP = args[i + 1]
            i += 2
        elif args[i] == "--port" and i + 1 < len(args):
            SERVER_PORT = int(args[i + 1])
            i += 2
        elif args[i] == "--rate" and i + 1 < len(args):
            SAMPLE_RATE = int(args[i + 1])
            i += 2
        elif args[i] == "--interval" and i + 1 < len(args):
            SEND_INTERVAL = float(args[i + 1])
            i += 2
        elif args[i] == "--amplitude" and i + 1 < len(args):
            PRESSURE_AMPLITUDE = float(args[i + 1])
            i += 2
        elif args[i] == "--help":
            print_help()
            return
        else:
            print(f"Unknown option: {args[i]}")
            print_help()
            return

    print("Starting sensor simulator...")
    print(f"Server: {SERVER_IP}:{SERVER_PORT}")
    print(f"Sample rate: {SAMPLE_RATE} Hz")
    print(f"Send interval: {SEND_INTERVAL:.2f} s")
    print(f"Pressure amplitude: {PRESSURE_AMPLITUDE} Pa")
    print("Press Ctrl+C to stop")
    global running
    try:
        # 启动发送线程
        sender_thread = threading.Thread(target=send_data_thread)
        sender_thread.daemon = True
        sender_thread.start()

        # 主线程等待
        while running:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping simulator...")

        running = False
        sender_thread.join(timeout=1)
        print("Simulator stopped")


if __name__ == "__main__":
    main()