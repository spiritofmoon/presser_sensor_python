# 双通道气压传感器（200Hz高速优化完整版）
import network
import socket
import time
import struct
import math
import _thread
from machine import Pin, SPI

# ===== 用户配置区 =====
WIFI_SSID = "your_wifi_ssid"  # 替换为您的WiFi名称
WIFI_PASSWORD = "your_wifi_password"  # 替换为您的WiFi密码
SERVER_IP = "117.72.10.210"  # 服务器IP地址
SERVER_PORT = 5000  # 服务器端口
SAMPLE_RATE = 200  # 采样率 (Hz)
BUFFER_SIZE = 1000  # 缓冲区大小
# =====================

# 传感器缓冲区
sensor_data_buffer1 = []
sensor_data_buffer2 = []

# BME280寄存器地址 (手册第26页)
REG_ID = 0xD0
REG_RESET = 0xE0
REG_CTRL_HUM = 0xF2
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_PRESS = 0xF7
REG_TEMP = 0xFA
REG_HUM = 0xFD
REG_CALIB = 0x88


class BME280:
    def __init__(self, spi_bus, cs_pin, sck_pin, mosi_pin, miso_pin):
        # 初始化SPI接口 (手册第34-35页)
        self.spi = SPI(
            spi_bus,
            sck=Pin(sck_pin),
            mosi=Pin(mosi_pin),
            miso=Pin(miso_pin),
            polarity=0,
            phase=0,
            baudrate=10_000_000
        )
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.t_fine = 0
        self.fixed_t_fine = 0
        self.last_temp_update = time.ticks_ms()

        # 验证设备ID (手册第27页)
        if self.read_byte(REG_ID) != 0x60:
            raise Exception("BME280 not found")

        # 软复位 (手册第27页)
        self.write_byte(REG_RESET, 0xB6)
        time.sleep_ms(10)

        # 读取完整的校准参数 (手册第24页)
        self.read_calibration()

        # 配置高速采样模式 (手册第19-20页)
        self.write_byte(REG_CTRL_HUM, 0x00)  # 跳过湿度检测
        self.write_byte(REG_CTRL_MEAS, 0x27)  # 温度x1+压力x1+正常模式
        self.write_byte(REG_CONFIG, 0x00)  # 0.5ms待机+关闭滤波器
        time.sleep_ms(50)  # 等待稳定

    def read_byte(self, reg):
        self.cs(0)
        self.spi.write(bytes([reg | 0x80]))
        result = self.spi.read(1)
        self.cs(1)
        return result[0]

    def read_bytes(self, reg, length):
        self.cs(0)
        self.spi.write(bytes([reg | 0x80]))
        result = bytearray(self.spi.read(length))
        self.cs(1)
        return result

    def write_byte(self, reg, value):
        self.cs(0)
        self.spi.write(bytes([reg & 0x7F, value]))
        self.cs(1)

    def read_calibration(self):
        """完整读取所有校准参数 (手册第24页)"""
        # 读取温度/压力校准参数 (0x88~0x9F)
        calib = self.read_bytes(REG_CALIB, 24)

        # 解析校准参数
        self.dig_T1 = struct.unpack("<H", calib[0:2])[0]
        self.dig_T2 = struct.unpack("<h", calib[2:4])[0]
        self.dig_T3 = struct.unpack("<h", calib[4:6])[0]
        self.dig_P1 = struct.unpack("<H", calib[6:8])[0]
        self.dig_P2 = struct.unpack("<h", calib[8:10])[0]
        self.dig_P3 = struct.unpack("<h", calib[10:12])[0]
        self.dig_P4 = struct.unpack("<h", calib[12:14])[0]
        self.dig_P5 = struct.unpack("<h", calib[14:16])[0]
        self.dig_P6 = struct.unpack("<h", calib[16:18])[0]
        self.dig_P7 = struct.unpack("<h", calib[18:20])[0]
        self.dig_P8 = struct.unpack("<h", calib[20:22])[0]
        self.dig_P9 = struct.unpack("<h", calib[22:24])[0]

        # 读取湿度校准参数 (0xE1~0xE7)
        self.dig_H1 = self.read_byte(0xA1)
        calib_h = self.read_bytes(0xE1, 7)

        self.dig_H2 = struct.unpack("<h", calib_h[0:2])[0]
        self.dig_H3 = calib_h[2]
        self.dig_H4 = struct.unpack("<h", bytes([calib_h[3], calib_h[4] & 0x0F]))[0]
        self.dig_H5 = struct.unpack("<h", bytes([calib_h[5], calib_h[4] >> 4]))[0]
        self.dig_H6 = struct.unpack("<b", bytes([calib_h[6]]))[0]

        # 打印校准参数
        print("Calibration Parameters:")
        print(f"T1:{self.dig_T1}, T2:{self.dig_T2}, T3:{self.dig_T3}")
        print(f"P1:{self.dig_P1}, P2:{self.dig_P2}, P3:{self.dig_P3}")
        print(f"P4:{self.dig_P4}, P5:{self.dig_P5}, P6:{self.dig_P6}")
        print(f"P7:{self.dig_P7}, P8:{self.dig_P8}, P9:{self.dig_P9}")
        print(f"H1:{self.dig_H1}, H2:{self.dig_H2}, H3:{self.dig_H3}")
        print(f"H4:{self.dig_H4}, H5:{self.dig_H5}, H6:{self.dig_H6}")

    def read_pressure_highspeed(self):
        """高速气压读取 (手册第30页)"""
        # 直接读取压力寄存器
        data = self.read_bytes(REG_PRESS, 3)
        press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

        # 定期更新温度补偿 (每2秒)
        if time.ticks_diff(time.ticks_ms(), self.last_temp_update) > 2000:
            self.update_temperature_compensation()

        return self.compensate_pressure_fast(press_raw)

    def update_temperature_compensation(self):
        """更新温度补偿值 (手册第25页)"""
        data = self.read_bytes(REG_TEMP, 3)
        temp_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

        # 温度补偿计算
        var1 = (temp_raw / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((temp_raw / 131072.0 - self.dig_T1 / 8192.0) *
                (temp_raw / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3
        self.fixed_t_fine = var1 + var2
        self.last_temp_update = time.ticks_ms()

    def compensate_pressure_fast(self, raw_press):
        """快速气压补偿 (手册第25页)"""
        var1 = self.fixed_t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1

        if var1 == 0:
            return 0

        p = 1048576.0 - raw_press
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        return p + (var1 + var2 + self.dig_P7) / 16.0


def connect_wifi():
    """连接WiFi网络"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        # 等待连接超时20秒
        timeout = time.time() + 20
        while not wlan.isconnected() and time.time() < timeout:
            time.sleep(0.5)

    if not wlan.isconnected():
        raise RuntimeError("WiFi connection failed")

    print("WiFi connected:", wlan.ifconfig())
    return wlan


def create_tcp_socket():
    """创建TCP套接字"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    return sock


def pack_data(sensor1_points, sensor2_points):
    """打包传感器数据 (手册第4页数据格式)"""
    num_points = len(sensor1_points) + len(sensor2_points)
    data = struct.pack("!I", num_points)  # 数据点数量

    # 传感器1数据
    for time_diff, _, press, _ in sensor1_points:
        data += struct.pack("!BIff", 1, time_diff, 0, press)

    # 传感器2数据
    for time_diff, _, press, _ in sensor2_points:
        data += struct.pack("!BIff", 2, time_diff, 0, press)

    return data


def get_sensor_thread(sensor1, sensor2):
    """传感器数据采集线程"""
    global sensor_data_buffer1, sensor_data_buffer2

    start_time = time.ticks_ms()
    sample_interval = 1 / SAMPLE_RATE  # 采样间隔

    while True:
        loop_start = time.ticks_us()

        # 计算时间戳
        time_diff = time.ticks_diff(time.ticks_ms(), start_time)

        # 高速读取双传感器
        press1 = sensor1.read_pressure_highspeed()
        press2 = sensor2.read_pressure_highspeed()

        # 添加到缓冲区
        if len(sensor_data_buffer1) < BUFFER_SIZE:
            sensor_data_buffer1.append((time_diff, 0, press1, 1))

        if len(sensor_data_buffer2) < BUFFER_SIZE:
            sensor_data_buffer2.append((time_diff, 0, press2, 2))

        # 精确时间控制
        elapsed = time.ticks_diff(time.ticks_us(), loop_start) / 1_000_000
        sleep_time = max(0, sample_interval - elapsed)
        time.sleep(sleep_time)


def main():
    """主程序"""
    # 连接网络
    wlan = connect_wifi()

    # 初始化传感器 (根据实际接线修改引脚)
    sensor1 = BME280(
        spi_bus=1,
        cs_pin=7,
        sck_pin=8,
        mosi_pin=9,
        miso_pin=10
    )

    sensor2 = BME280(
        spi_bus=2,
        cs_pin=3,
        sck_pin=4,
        mosi_pin=5,
        miso_pin=6
    )

    print("Sensors initialized in HIGH SPEED MODE")

    # 创建TCP连接
    sock = create_tcp_socket()
    while True:
        try:
            print(f"Connecting to {SERVER_IP}:{SERVER_PORT}")
            sock.connect((SERVER_IP, SERVER_PORT))
            print("Server connection established")
            break
        except Exception as e:
            print(f"Connection failed: {e}, retrying in 5s")
            time.sleep(5)

    # 启动传感器采集线程
    _thread.start_new_thread(get_sensor_thread, (sensor1, sensor2))

    last_send_time = time.ticks_ms()
    send_interval = 1000  # 50ms发送间隔

    while True:
        current_time = time.ticks_ms()

        # 定时发送机制
        if time.ticks_diff(current_time, last_send_time) > send_interval:
            # 准备发送数据
            send_buf1 = sensor_data_buffer1.copy()
            send_buf2 = sensor_data_buffer2.copy()
            sensor_data_buffer1.clear()
            sensor_data_buffer2.clear()

            if send_buf1 or send_buf2:
                try:
                    # 打包并发送数据
                    packed = pack_data(send_buf1, send_buf2)
                    sock.sendall(packed)
                    print(f"Sent {len(send_buf1 + send_buf2)} samples")
                except Exception as e:
                    print(f"Send error: {e}")
                    # 重新连接逻辑
                    try:
                        sock.close()
                        sock = create_tcp_socket()
                        sock.connect((SERVER_IP, SERVER_PORT))
                        print("Reconnected to server")
                    except:
                        print("Reconnection failed")

            last_send_time = current_time

        time.sleep_ms(5)  # 降低CPU占用


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Critical error:", e)
        # 重启设备
        print("Rebooting in 10 seconds...")
        time.sleep(10)
        import machine

        machine.reset()
