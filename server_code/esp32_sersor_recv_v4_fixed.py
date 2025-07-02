
import socket
import struct
import time
from datetime import datetime
import threading

HOST = '0.0.0.0'
PORT = 5000
BUFFER_SIZE = 4096

def unpack_data(data):
    num_points = struct.unpack("!I", data[:4])[0]
    sensor1_samples = []
    sensor2_samples = []

    sample_size = 13
    offset = 4

    expected_size = 4 + num_points * sample_size
    if len(data) < expected_size:
        print(f"Warning: Incomplete data packet. Expected {expected_size} bytes, got {len(data)} bytes")
        num_points = (len(data) - 4) // sample_size

    for i in range(num_points):
        start = offset + i * sample_size
        end = start + sample_size
        sensor_id, time_diff, temp, press = struct.unpack("!BIff", data[start:end])
        if sensor_id == 1:
            sensor1_samples.append((time_diff, temp, press))
        elif sensor_id == 2:
            sensor2_samples.append((time_diff, temp, press))
        else:
            print(f"Warning: Unknown sensor ID {sensor_id}")

    return sensor1_samples, sensor2_samples

def save_to_database(receive_time, sensor1_samples, sensor2_samples):
    print(f"[{receive_time}] Received {len(sensor1_samples)} samples from sensor1, {len(sensor2_samples)} from sensor2")

    def print_sensor_data(samples, sensor_name):
        if not samples:
            print(f"\nNo data from {sensor_name}")
            return

        print(f"\nDetailed Samples from {sensor_name}:")
        print("-" * 60)
        print(f"{'Index':<6} {'Time Diff (ms)':<15} {'Temp (C)':<15} {'Pressure (Pa)':<15}")
        print("-" * 60)

        for i, (time_diff, temp, press) in enumerate(samples):
            print(f"{i + 1:<6} {time_diff:<15} {temp:<15.2f} {press:<15.2f}")

        print("-" * 60)

        start_time = samples[0][0]
        end_time = samples[-1][0]
        time_range = end_time - start_time

        print(f"\n{sensor_name} Summary Statistics:")
        print(f"Time range: {start_time}ms - {end_time}ms ({time_range}ms)")
        print(f"First sample: TimeDiff={samples[0][0]}ms, Temp={samples[0][1]:.2f}C, Press={samples[0][2]:.2f}Pa")
        print(f"Last sample: TimeDiff={samples[-1][0]}ms, Temp={samples[-1][1]:.2f}C, Press={samples[-1][2]:.2f}Pa")

        avg_temp = sum(s[1] for s in samples) / len(samples)
        avg_press = sum(s[2] for s in samples) / len(samples)
        print(f"Avg Temp: {avg_temp:.2f}C, Avg Press: {avg_press:.2f}Pa")

        if time_range > 0:
            sample_rate = len(samples) / (time_range / 1000)
            print(f"Sample rate: {sample_rate:.2f} Hz")

        print("=" * 60 + "\n")

    print_sensor_data(sensor1_samples, "Sensor1")
    print_sensor_data(sensor2_samples, "Sensor2")

    if sensor1_samples and sensor2_samples:
        print("\nComparison between Sensor1 and Sensor2:")
        print("-" * 60)
        avg_temp1 = sum(s[1] for s in sensor1_samples) / len(sensor1_samples)
        avg_temp2 = sum(s[1] for s in sensor2_samples) / len(sensor2_samples)
        temp_diff = avg_temp1 - avg_temp2

        avg_press1 = sum(s[2] for s in sensor1_samples) / len(sensor1_samples)
        avg_press2 = sum(s[2] for s in sensor2_samples) / len(sensor2_samples)
        press_diff = avg_press1 - avg_press2

        print(f"Avg Temp Difference (Sensor1 - Sensor2): {temp_diff:.2f}°C")
        print(f"Avg Pressure Difference (Sensor1 - Sensor2): {press_diff:.2f}Pa")
        print("=" * 60 + "\n")

def handle_client(conn, addr):
    print(f"New connection from {addr}")
    buffer = b''

    try:
        while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                print(f"Client {addr} disconnected or no data")
                break

            receive_time = datetime.now()
            buffer += data

            while len(buffer) >= 4:
                num_points = struct.unpack("!I", buffer[:4])[0]
                packet_size = 4 + num_points * 13
                if len(buffer) < packet_size:
                    break

                packet = buffer[:packet_size]
                buffer = buffer[packet_size:]

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
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)
        print(f"Server listening on {HOST}:{PORT}")

        try:
            while True:
                conn, addr = s.accept()
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
