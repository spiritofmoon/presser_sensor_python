import socket
import struct
import time
from datetime import datetime
import threading

# 服务器配置
HOST = '0.0.0.0'  # 监听所有接口
PORT = 5000
BUFFER_SIZE = 4096  # 数据包最大尺寸


def unpack_data(data):
    """解包从ESP32接收的数据（双传感器格式）"""
    # 读取数据点数（4字节）
    num_points = struct.unpack("!I", data[:4])[0]
    sensor1_samples = []
    sensor2_samples = []

    # 每个样本13字节 (1字节传感器ID + 4字节时间差 + 4字节温度 + 4字节气压)
    sample_size = 13
    offset = 4  # 跳过包头

    # 检查数据长度是否足够
    expected_size = 4 + num_points * sample_size
    if len(data) < expected_size:
        print(f"Warning: Incomplete data packet. Expected {expected_size} bytes, got {len(data)} bytes")
        # 尽可能读取可用数据
        num_points = (len(data) - 4) // sample_size

    # 解析每个数据点
    for i in range(num_points):
        start = offset + i * sample_size
        end = start + sample_size
        sensor_id, time_diff, temp, press = struct.unpack("!BIff", data[start:end])

        # 根据传感器ID分类存储
        if sensor_id == 1:
            sensor1_samples.append((time_diff, temp, press))
        elif sensor_id == 2:
            sensor2_samples.append((time_diff, temp, press))
        else:
            print(f"Warning: Unknown sensor ID {sensor_id}")

    return sensor1_samples, sensor2_samples


def save_to_database(receive_time, sensor1_samples, sensor2_samples):
    """打印所有接收到的样本数据"""
    print(f"[{receive_time}] Received {len(sensor1_samples)} samples from sensor1, {len(sensor2_samples)} from sensor2")

    def print_sensor_data(samples, sensor_name):
        if not samples:
            print(f"\nNo data from {sensor_name}")
            return

        # 打印所有样本的详细信息
        print(f"\nDetailed Samples from {sensor_name}:")
        print("-" * 60)
        print(f"{'Index':<6} {'Time Diff (ms)':<15} {'Temp (C)':<15} {'Pressure (Pa)':<15}")
        print("-" * 60)

        for i, (time_diff, temp, press) in enumerate(samples):
            print(f"{i + 1:<6} {time_diff:<15} {temp:<15.2f} {press:<15.2f}")

        print("-" * 60)

        # 计算统计数据
        start_time = samples[0][0]
        end_time = samples[-1][0]
        time_range = end_time - start_time

        print(f"\n{senor_name} Summary Statistics:")
        print(f"Time range: {start_time}ms - {end_time}ms ({time_range}ms)")
        print(f"First sample: TimeDiff={samples[0][0]}ms, Temp={samples[0][1]:.2f}C, Press={samples[0][2]:.2f}Pa")
        print(f"Last sample: TimeDiff={samples[-1][0]}ms, Temp={samples[-1][1]:.2f}C, Press={samples[-1][2]:.2f}Pa")

        # 计算平均值
        avg_temp = sum(s[1] for s in samples) / len(samples)
        avg_press = sum(s[2] for s in samples) / len(samples)
        print(f"Avg Temp: {avg_temp:.2f}C, Avg Press: {avg_press:.2f}Pa")

        # 计算每秒采样率
        if time_range > 0:
            sample_rate = len(samples) / (time_range / 1000)
            print(f"Sample rate: {sample_rate:.2f} Hz")

        print("=" * 60 + "\n")

    # 分别打印两个传感器的数据
    print_sensor_data(sensor1_samples, "Sensor1")
    print_sensor_data(sensor2_samples, "Sensor2")

    # 如果有两个传感器的数据，计算它们之间的差异
    if sensor1_samples and sensor2_samples:
        print("\nComparison between Sensor1 and Sensor2:")
        print("-" * 60)

        # 计算平均温度差
        avg_temp1 = sum(s[1] for s in sensor1_samples) / len(sensor1_samples)
        avg_temp2 = sum(s[1] for s in sensor2_samples) / len(sensor2_samples)
        temp_diff = avg_temp1 - avg_temp2

        # 计算平均气压差
        avg_press1 = sum(s[2] for s in sensor1_samples) / len(sensor1_samples)
        avg_press2 = sum(s[2] for s in sensor2_samples) / len(sensor2_samples)
        press_diff = avg_press1 - avg_press2

        print(f"Avg Temp Difference (Sensor1 - Sensor2): {temp_diff:.2f}°C")
        print(f"Avg Pressure Difference (Sensor1 - Sensor2): {press_diff:.2f}Pa")
        print("=" * 60 + "\n")


def handle_client(conn, addr):
    """处理客户端连接"""
    print(f"New connection from {addr}")
    buffer = b''  # 用于存储不完整的数据包

    try:
        while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                print(f"Client {addr} disconnected")
                break

            receive_time = datetime.now()

            # 将新数据添加到缓冲区
            buffer += data

            # 处理所有完整的数据包
            while len(buffer) >= 4:
                # 读取数据点数（前4字节）
                num_points = struct.unpack("!I", buffer[:4])[0]

                # 计算完整数据包大小
                packet_size = 4 + num_points * 13  # 包头 + 所有数据点

                # 检查是否有足够数据
                if len(buffer) < packet_size:
                    break  # 等待更多数据

                # 提取完整数据包
                packet = buffer[:packet_size]
                buffer = buffer[packet_size:]  # 移除已处理的数据

                try:
                    sensor1_samples, sensor2_samples = unpack_data(packet)
                    save_to_database(receive_time, sensor1_samples, sensor2_samples)
                except struct.error as e:
                    print(f"Unpack error: {e}")
                except Exception as e:
                    print(f"Processing error: {e}")

    except (ConnectionResetError, BrokenPipeError):
        print(f"Client {addr} disconnected unexpectedly")
    finally:
        conn.close()
        print(f"Connection to {addr} closed")


def start_server():
    """启动TCP服务器"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)  # 允许最多5个并发连接
        print(f"Server listening on {HOST}:{PORT}")

        try:
            while True:
                conn, addr = s.accept()
                # 为每个客户端创建新线程
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
                print(f"Active connections: {threading.active_count() - 1}")

        except KeyboardInterrupt:
            print("Server shutting down...")
        finally:
            s.close()


if __name__ == "__main__":
    start_server()