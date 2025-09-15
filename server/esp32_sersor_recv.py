import socket
import struct
import time
from datetime import datetime

# 服务器配置
HOST = '0.0.0.0'  # 监听所有接口
PORT = 5000
BUFFER_SIZE = 4096  # 数据包最大尺寸
SAMPLES_PER_SECOND = 100  # 100Hz

def unpack_data(data):
    """解包从ESP32接收的数据"""
    # 第一个4字节是时间戳
    timestamp = struct.unpack("!I", data[:4])[0]
    samples = []
    
    # 每个样本12字节 (3个float)
    sample_size = 12
    num_samples = (len(data) - 4) // sample_size
    
    for i in range(num_samples):
        start = 4 + i * sample_size
        end = start + sample_size
        temp, press, hum = struct.unpack("!fff", data[start:end])
        samples.append((temp, press, hum))
    
    return timestamp, samples

def save_to_database(timestamp, samples):
    """将数据保存到数据库（示例函数）"""
    # 这里添加实际的数据库存储逻辑
    # 示例: 打印前5个样本
    print(f"[{datetime.fromtimestamp(timestamp)}] Received {len(samples)} samples")
    for i in range(min(5, len(samples))):
        t, p, h = samples[i]
        print(f"Sample {i+1}: Temp={t:.2f}C, Press={p:.2f}Pa, Hum={h:.2f}%")

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Server listening on {HOST}:{PORT}")
        
        while True:
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            
            try:
                while True:
                    data = conn.recv(BUFFER_SIZE)
                    if not data:
                        break
                    
                    try:
                        timestamp, samples = unpack_data(data)
                        save_to_database(timestamp, samples)
                    except struct.error as e:
                        print(f"Unpack error: {e}")
                    except Exception as e:
                        print(f"Processing error: {e}")
            
            except (ConnectionResetError, BrokenPipeError):
                print("Client disconnected")
            finally:
                conn.close()
                print("Connection closed")

if __name__ == "__main__":
    start_server()
