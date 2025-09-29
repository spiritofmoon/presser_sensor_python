# 四通道传感器数据采集（2路BME280气压 + 2路ICS43434麦克风）
import network
import socket
import time
import struct
import math
import _thread
from machine import Pin, SPI, I2S
import gc

# ===== 用户配置区 =====
WIFI_SSID = "chen"
WIFI_PASSWORD = "98765432"
SERVER_IP = "117.72.10.210"
SERVER_PORT = 5000
BUFFER_SIZE = 2048  # 每个传感器的独立缓冲区大小

# --- BME280 SPI 引脚配置 ---
# BME280 传感器 1
BME1_SPI_BUS = 1
BME1_CS_PIN = 7
BME1_SCK_PIN = 8
BME1_MOSI_PIN = 9
BME1_MISO_PIN = 10
# BME280 传感器 2
BME2_SPI_BUS = 2
BME2_CS_PIN = 3
BME2_SCK_PIN = 4
BME2_MOSI_PIN = 5
BME2_MISO_PIN = 6

# --- ICS43434 I2S 引脚配置 (请根据您的实际接线修改) ---
# ICS43434 麦克风 1
ICS1_SCK_PIN = 12
ICS1_WS_PIN = 11
ICS1_SD_PIN = 13
# ICS43434 麦克风 2
ICS2_SCK_PIN = 17
ICS2_WS_PIN = 15
ICS2_SD_PIN = 41
# ICS43434 麦克风 3
ICS3_SCK_PIN = 2
ICS3_WS_PIN = 14
ICS3_SD_PIN = 1

# --- I2S 参数 ---
I2S_SAMPLE_RATE = 16000  # 采样率 (Hz)
I2S_BUFFER_SAMPLES = 1600  # I2S读取缓冲区中的样本数
# =====================

# 传感器数据缓冲区
sensor_data_buffer1 = []  # BME280 1
sensor_data_buffer2 = []  # BME280 2
sensor_data_buffer3 = []  # ICS43434 1
sensor_data_buffer4 = []  # ICS43434 2

# BME280寄存器地址 (代码无变化)
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
        # 初始化SPI接口
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

        if self.read_byte(REG_ID) != 0x60:
            raise Exception("BME280 not found")

        self.write_byte(REG_RESET, 0xB6)
        time.sleep_ms(10)
        self.read_calibration()
        self.write_byte(REG_CTRL_HUM, 0x00)
        self.write_byte(REG_CTRL_MEAS, 0x27)
        self.write_byte(REG_CONFIG, 0x00)
        time.sleep_ms(50)

    def read_byte(self, reg):
        self.cs(0);
        self.spi.write(bytes([reg | 0x80]));
        result = self.spi.read(1);
        self.cs(1)
        return result[0]

    def read_bytes(self, reg, length):
        self.cs(0);
        self.spi.write(bytes([reg | 0x80]));
        result = bytearray(self.spi.read(length));
        self.cs(1)
        return result

    def write_byte(self, reg, value):
        self.cs(0);
        self.spi.write(bytes([reg & 0x7F, value]));
        self.cs(1)

    def read_calibration(self):
        calib = self.read_bytes(REG_CALIB, 24)
        self.dig_T1 = struct.unpack("<H", calib[0:2])[0];
        self.dig_T2 = struct.unpack("<h", calib[2:4])[0];
        self.dig_T3 = struct.unpack("<h", calib[4:6])[0]
        self.dig_P1 = struct.unpack("<H", calib[6:8])[0];
        self.dig_P2 = struct.unpack("<h", calib[8:10])[0];
        self.dig_P3 = struct.unpack("<h", calib[10:12])[0];
        self.dig_P4 = struct.unpack("<h", calib[12:14])[0];
        self.dig_P5 = struct.unpack("<h", calib[14:16])[0];
        self.dig_P6 = struct.unpack("<h", calib[16:18])[0];
        self.dig_P7 = struct.unpack("<h", calib[18:20])[0];
        self.dig_P8 = struct.unpack("<h", calib[20:22])[0];
        self.dig_P9 = struct.unpack("<h", calib[22:24])[0]

    def read_pressure_highspeed(self):
        data = self.read_bytes(REG_PRESS, 3)
        press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        if time.ticks_diff(time.ticks_ms(), self.last_temp_update) > 2000:
            self.update_temperature_compensation()
        return self.compensate_pressure_fast(press_raw)

    def update_temperature_compensation(self):
        data = self.read_bytes(REG_TEMP, 3)
        temp_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        var1 = (temp_raw / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((temp_raw / 131072.0 - self.dig_T1 / 8192.0) * (temp_raw / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3
        self.fixed_t_fine = var1 + var2
        self.last_temp_update = time.ticks_ms()

    def compensate_pressure_fast(self, raw_press):
        var1 = self.fixed_t_fine / 2.0 - 64000.0;
        var2 = var1 * var1 * self.dig_P6 / 32768.0;
        var2 = var2 + var1 * self.dig_P5 * 2.0;
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0: return 0
        p = 1048576.0 - raw_press;
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0;
        var2 = p * self.dig_P8 / 32768.0
        return p + (var1 + var2 + self.dig_P7) / 16.0


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        timeout = time.time() + 20
        while not wlan.isconnected() and time.time() < timeout: time.sleep(0.5)
    if not wlan.isconnected(): raise RuntimeError("WiFi connection failed")
    print("WiFi connected:", wlan.ifconfig())
    return wlan


def create_tcp_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    return sock


def pack_data(s1_pts, s2_pts, s3_pts, s4_pts):
    """
    【优化版】打包来自所有四个传感器的数据点。
    使用列表和 join() 方法避免低效的 bytes 拼接。
    """
    num_points = len(s1_pts) + len(s2_pts) + len(s3_pts) + len(s4_pts)
    
    # 1. 创建一个列表来存放所有数据块，而不是直接创建bytes对象
    data_chunks = []
    
    # 2. 将所有打包好的数据块 append 到列表中（这个操作非常快）
    # BME280 传感器 1
    for time_diff, _, press, _ in s1_pts:
        data_chunks.append(struct.pack("!BIff", 1, time_diff, 0, press))
    # BME280 传感器 2
    for time_diff, _, press, _ in s2_pts:
        data_chunks.append(struct.pack("!BIff", 2, time_diff, 0, press))
    # ICS43434 麦克风 1
    for time_diff, _, sample, _ in s3_pts:
        data_chunks.append(struct.pack("!BIff", 3, time_diff, 0, float(sample)))
    # ICS43434 麦克风 2
    for time_diff, _, sample, _ in s4_pts:
        data_chunks.append(struct.pack("!BIff", 4, time_diff, 0, float(sample)))
        
    # 3. 在最后，使用 b''.join() 方法一次性、高效地拼接所有数据块
    payload = b''.join(data_chunks)
    
    # 4. 加上4字节的数据点头
    header = struct.pack("!I", num_points)
    
    return header + payload


# ===================================================================
# =====          vvv   传感器数据采集线程 vvv                  =====
# ===================================================================

def bme280_thread(sensor1, sensor2):
    """BME280压力传感器数据采集线程。"""
    global sensor_data_buffer1, sensor_data_buffer2
    start_time = time.ticks_ms()
    while True:
        time_diff = time.ticks_diff(time.ticks_ms(), start_time)
        press1 = sensor1.read_pressure_highspeed()
        press2 = sensor2.read_pressure_highspeed()
        if len(sensor_data_buffer1) < BUFFER_SIZE:
            sensor_data_buffer1.append((time_diff, 0, press1, 1))
        if len(sensor_data_buffer2) < BUFFER_SIZE:
            sensor_data_buffer2.append((time_diff, 0, press2, 2))

        # 关键！加入微小延时，将CPU控制权让给其他线程，避免线程饥饿。
        time.sleep_ms(1)


def ics43434_thread(mic1, mic2):
    """ICS43434麦克风数据采集线程。"""
    global sensor_data_buffer3, sensor_data_buffer4

    # 创建一个缓冲区来存储I2S读取的原始字节数据
    # 每个样本32位 = 4字节
    mic_samples_buffer = bytearray(I2S_BUFFER_SAMPLES * 4)

    start_time = time.ticks_ms()

    while True:
        try:
            # 读取麦克风1的数据
            num_bytes_read1 = mic1.readinto(mic_samples_buffer)
            if num_bytes_read1 > 0:
                time_diff = time.ticks_diff(time.ticks_ms(), start_time)
                # 将字节数据解包成32位整数样本
                samples = struct.unpack('<%di' % (num_bytes_read1 // 4), mic_samples_buffer)
                for sample in samples:
                    # ICS43434是24位数据，通常存储在32位样本的高位，需要右移8位以获得正确的值
                    sample_24bit = sample >> 8
                    if len(sensor_data_buffer3) < BUFFER_SIZE:
                        sensor_data_buffer3.append((time_diff, 0, sample_24bit, 3))

            # 读取麦克风2的数据
            num_bytes_read2 = mic2.readinto(mic_samples_buffer)
            if num_bytes_read2 > 0:
                time_diff = time.ticks_diff(time.ticks_ms(), start_time)
                samples = struct.unpack('<%di' % (num_bytes_read2 // 4), mic_samples_buffer)
                for sample in samples:
                    sample_24bit = sample >> 8
                    if len(sensor_data_buffer4) < BUFFER_SIZE:
                        sensor_data_buffer4.append((time_diff, 0, sample_24bit, 4))

        except Exception as e:
            print(f"ICS43434 thread error: {e}")
        time.sleep_ms(1)


# ===================================================================
# =====          ^^^   传感器数据采集线程 ^^^                  =====
# ===================================================================

def main():
    wlan = connect_wifi()

    # 初始化 BME280 传感器
    sensor1 = BME280(BME1_SPI_BUS, BME1_CS_PIN, BME1_SCK_PIN, BME1_MOSI_PIN, BME1_MISO_PIN)
    sensor2 = BME280(BME2_SPI_BUS, BME2_CS_PIN, BME2_SCK_PIN, BME2_MOSI_PIN, BME2_MISO_PIN)
    print("BME280 sensors initialized in MAX SPEED MODE")

    # 初始化 ICS43434 麦克风
    # I2S.MONO 模式，每个样本32位
    mic1 = I2S(0, sck=Pin(ICS1_SCK_PIN), ws=Pin(ICS1_WS_PIN), sd=Pin(ICS1_SD_PIN), mode=I2S.RX, bits=32, format=I2S.MONO, rate=I2S_SAMPLE_RATE, ibuf=8192)
    # mic2 = mic1
    mic2 = I2S(1, sck=Pin(ICS2_SCK_PIN), ws=Pin(ICS2_WS_PIN), sd=Pin(ICS2_SD_PIN), mode=I2S.RX, bits=32, format=I2S.MONO, rate=I2S_SAMPLE_RATE, ibuf=8192)
    print("ICS43434 microphones initialized")

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
    _thread.start_new_thread(bme280_thread, (sensor1, sensor2))
    _thread.start_new_thread(ics43434_thread, (mic1, mic2))

    last_send_time = time.ticks_ms()
    send_interval = 200  # 保持1秒的发送间隔

    while True:
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_send_time) > send_interval:
            
            # --- 开始性能分析 ---
            profile_start_time = time.ticks_ms()

            # 步骤 1: 复制和清空缓冲区
            send_buf1 = sensor_data_buffer1.copy()
            send_buf2 = sensor_data_buffer2.copy()
            send_buf3 = sensor_data_buffer3.copy()
            send_buf4 = sensor_data_buffer4.copy()
            sensor_data_buffer1.clear()
            sensor_data_buffer2.clear()
            sensor_data_buffer3.clear()
            sensor_data_buffer4.clear()
            
            profile_after_copy_time = time.ticks_ms()
            copy_duration = time.ticks_diff(profile_after_copy_time, profile_start_time)

            total_samples = len(send_buf1) + len(send_buf2) + len(send_buf3) + len(send_buf4)
            
            if total_samples > 0:
                try:
                    # 步骤 2: 数据打包
                    packed = pack_data(send_buf1, send_buf2, send_buf3, send_buf4)
                    
                    profile_after_pack_time = time.ticks_ms()
                    pack_duration = time.ticks_diff(profile_after_pack_time, profile_after_copy_time)

                    # 步骤 3: 网络发送
                    sock.sendall(packed)
                    
                    profile_after_send_time = time.ticks_ms()
                    send_duration = time.ticks_diff(profile_after_send_time, profile_after_pack_time)

                    # --- 打印分析结果 ---
                    total_duration = time.ticks_diff(profile_after_send_time, profile_start_time)
                    print("--- Profiling Report ---")
                    print(f"Total Samples: {total_samples}")
                    print(f"[PROFILE] Copy/Clear took: {copy_duration} ms")
                    print(f"[PROFILE] pack_data() took:  {pack_duration} ms")
                    print(f"[PROFILE] sock.sendall() took: {send_duration} ms")
                    print(f"------------------------ Total cycle time: {total_duration} ms")

                except Exception as e:
                    print(f"Send error: {e}")
                    # ... (重连逻辑不变) ...
            
            # 使用 time.ticks_add() 来避免累积延迟
            last_send_time = time.ticks_add(last_send_time, send_interval)
            # 如果处理时间已经超过了发送间隔，立即准备下一次检查
            if time.ticks_diff(time.ticks_ms(), last_send_time) > 0:
                last_send_time = time.ticks_ms()

        # 主循环的短暂延时，让出CPU
        time.sleep_ms(5)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Critical error:", e)
        print("Rebooting in 10 seconds...")
        time.sleep(10)
        import machine

        machine.reset()
