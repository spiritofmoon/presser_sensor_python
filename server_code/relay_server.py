# relay_server.py
# 部署在公网服务器上，负责数据中转

import socket
import threading
import time

# --- 配置 ---
SENSOR_LISTEN_IP = "0.0.0.0"  # 监听所有网络接口
SENSOR_LISTEN_PORT = 5000  # ESP32连接此端口
PC_LISTEN_IP = "0.0.0.0"  # 监听所有网络接口
PC_LISTEN_PORT = 5001  # PC客户端连接此端口
# ------------

# 使用线程锁来安全地管理PC客户端的socket对象
pc_client_socket_lock = threading.Lock()
pc_client_socket = None


def handle_sensor_device(sensor_conn, sensor_addr):
    """
    处理单个传感器连接的线程函数。
    它会循环接收数据，并尝试转发给PC客户端。
    """
    global pc_client_socket
    print(f"[SENSOR] New connection from {sensor_addr}")

    while True:
        try:
            # 从传感器接收数据，一次最多读取4KB
            data = sensor_conn.recv(4096)
            if not data:
                # 传感器主动断开连接
                break

            # 锁定全局变量，检查PC客户端是否存在并发送数据
            with pc_client_socket_lock:
                if pc_client_socket:
                    try:
                        pc_client_socket.sendall(data)
                    except (socket.error, BrokenPipeError):
                        # PC客户端连接已断开
                        print("[PC] PC client has disconnected. Stopping data forwarding.")
                        pc_client_socket = None  # 清理无效的socket
                # else:
                #     print("[DEBUG] No PC client connected, discarding data from sensor.")

        except ConnectionResetError:
            # 传感器连接被重置
            break
        except Exception as e:
            print(f"[SENSOR] Error with {sensor_addr}: {e}")
            break

    print(f"[SENSOR] Connection closed with {sensor_addr}")
    sensor_conn.close()


def wait_for_pc_client():
    """
    一个独立的线程，专门用于监听和管理PC客户端的连接。
    """
    global pc_client_socket

    pc_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pc_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    pc_server_socket.bind((PC_LISTEN_IP, PC_LISTEN_PORT))
    pc_server_socket.listen(1)
    print(f"[PC] Waiting for PC client to connect on port {PC_LISTEN_PORT}...")

    while True:
        # 接受来自PC的新连接
        conn, addr = pc_server_socket.accept()
        print(f"[PC] PC client connected from {addr}")

        # 锁定并更新全局的PC socket
        with pc_client_socket_lock:
            pc_client_socket = conn

        # 循环检查PC连接是否仍然有效
        while True:
            time.sleep(1)
            with pc_client_socket_lock:
                if pc_client_socket is None:
                    # 如果连接在handle_sensor中被置为None，说明已断开
                    print("[PC] PC connection was lost. Waiting for a new connection...")
                    break  # 跳出内层循环，等待新的accept()


def main():
    """
    主函数，启动所有服务。
    """
    # 启动PC客户端监听线程
    pc_thread = threading.Thread(target=wait_for_pc_client, daemon=True)
    pc_thread.start()

    # 在主线程中开始监听传感器
    sensor_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sensor_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sensor_server_socket.bind((SENSOR_LISTEN_IP, SENSOR_LISTEN_PORT))
    sensor_server_socket.listen(10)  # 最多处理10个等待中的传感器连接
    print(f"[SENSOR] Relay server is listening for sensors on port {SENSOR_LISTEN_PORT}...")

    while True:
        # 接受来自传感器的新连接
        conn, addr = sensor_server_socket.accept()
        # 为每个传感器创建一个独立的线程来处理
        sensor_thread = threading.Thread(target=handle_sensor_device, args=(conn, addr), daemon=True)
        sensor_thread.start()


if __name__ == "__main__":
    main()