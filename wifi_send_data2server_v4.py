#双通道气压传感器
import network
import socket
import time
from machine import Pin, SPI
import struct
import math
import _thread


# WiFi配置
WIFI_SSID = "chen"
WIFI_PASSWORD = "98765432"
SERVER_IP = "117.72.10.210"  # 替换为服务器IP
SERVER_PORT = 5000
sensor_data_buffer_max_len = 200

# 在全局变量部分添加第二个传感器的缓冲区
sensor_data_buffer1_1 = []  # 传感器1的缓冲区1
sensor_data_buffer1_2 = []  # 传感器1的缓冲区2
sensor_data_buffer2_1 = []  # 传感器2的缓冲区1
sensor_data_buffer2_2 = []  # 传感器2的缓冲区2

# 对应的状态标志
sensor1_buffer1_is_full = False
sensor1_buffer2_is_full = False
sensor2_buffer1_is_full = False
sensor2_buffer2_is_full = False

# BME280寄存器地址
REG_ID      = 0xD0
REG_RESET   = 0xE0
REG_CTRL_HUM= 0xF2
REG_STATUS  = 0xF3
REG_CTRL_MEAS=0xF4
REG_CONFIG  = 0xF5
REG_PRESS   = 0xF7  # 压力数据起始地址 (3字节)
REG_TEMP    = 0xFA  # 温度数据起始地址 (3字节)
REG_HUM     = 0xFD  # 湿度数据起始地址 (2字节)
REG_CALIB   = 0x88  # 校准参数起始地址

# 传感器配置
SAMPLE_RATE = 100  # 100Hz采样率
delay_time = 1/SAMPLE_RATE
BUFFER_SIZE = SAMPLE_RATE * 3  # 100个点的3个参数(温度、气压、湿度)

class BME280:
    def __init__(self, spi_bus, cs_pin, sck_pin, mosi_pin, miso_pin):
        # 初始化SPI接口 (尝试两种模式)
        try:
            # 先尝试模式0
            print('model0')
            print('init',spi_bus,cs_pin,sck_pin,mosi_pin,miso_pin)
            self.spi = SPI(
                spi_bus,
                sck=Pin(sck_pin),
                mosi=Pin(mosi_pin),
                miso=Pin(miso_pin),
                polarity=0,
                phase=0,
                baudrate=1000000
            )
            self.spi_mode = 0
        except:
            # 如果失败尝试模式3
            print('model3')
            self.spi = SPI(
                spi_bus,
                sck=Pin(sck_pin),
                mosi=Pin(mosi_pin),
                miso=Pin(miso_pin),
                polarity=1,
                phase=1,
                baudrate=1000000
            )
            self.spi_mode = 3
        
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.t_fine = 0
        
        # 验证设备ID
        chip_id = self.read_byte(REG_ID)
        if chip_id != 0x60:
            raise Exception("BME280 not found. ID: 0x{:02X}".format(chip_id))
        
        # 软复位
        self.write_byte(REG_RESET, 0xB6)
        time.sleep_ms(10)
        
        # 读取校准参数
        self.read_calibration()
        
        # 配置传感器
        self.write_byte(REG_CTRL_HUM, 0x01)  # 湿度采样x1
        self.write_byte(REG_CTRL_MEAS, 0x27) # 温度/压力采样x1, 正常模式
        self.write_byte(REG_CONFIG, 0x00)    # 滤波器关闭, 待机时间0.5ms
        time.sleep_ms(50)  # 等待传感器稳定
    
    def read_byte(self, reg):
        self.cs(0)
        self.spi.write(bytes([reg | 0x80]))  # 设置最高位为1表示读操作
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
        self.spi.write(bytes([reg & 0x7F, value]))  # 最高位0表示写
        self.cs(1)
    
    def read_calibration(self):
        # 读取温度/压力校准参数 (0x88~0x9F)
        calib = self.read_bytes(REG_CALIB, 24)
        
        # 使用struct解析有符号值
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
        
        # 调试输出校准参数
        print("Calibration Parameters:")
        print(f"T1: {self.dig_T1}, T2: {self.dig_T2}, T3: {self.dig_T3}")
        print(f"P1: {self.dig_P1}, P2: {self.dig_P2}, P3: {self.dig_P3}")
        print(f"P4: {self.dig_P4}, P5: {self.dig_P5}, P6: {self.dig_P6}")
        print(f"P7: {self.dig_P7}, P8: {self.dig_P8}, P9: {self.dig_P9}")
        print(f"H1: {self.dig_H1}, H2: {self.dig_H2}, H3: {self.dig_H3}")
        print(f"H4: {self.dig_H4}, H5: {self.dig_H5}, H6: {self.dig_H6}")
    
    def read_raw_data(self):
        # 检查数据是否就绪
        while (self.read_byte(REG_STATUS) & 0x08) != 0:
            time.sleep_ms(1)
        
        # 一次性读取所有数据寄存器
        data = self.read_bytes(REG_PRESS, 8)
        
        # 解析原始数据 (修正20位数据解析)
        press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw  = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw   = (data[6] << 8) | data[7]
        
        # 调试输出原始数据
        #print(f"Raw data: P={press_raw}, T={temp_raw}, H={hum_raw}")
        
        return temp_raw, press_raw, hum_raw
    
    def compensate_temperature(self, raw_temp):
        # 温度补偿公式 (修正计算)
        var1 = (raw_temp / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((raw_temp / 131072.0 - self.dig_T1 / 8192.0) * 
               (raw_temp / 131072.0 - self.dig_T1 / 8192.0)) * self.dig_T3
        self.t_fine = var1 + var2
        return self.t_fine / 5120.0
    
    def compensate_pressure(self, raw_press):
        # 压力补偿公式 (修正计算)
        var1 = self.t_fine / 2.0 - 64000.0
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
    
    def compensate_humidity(self, raw_hum):
        # 湿度补偿公式 (修正计算)
        var1 = self.t_fine - 76800.0
        var1 = (raw_hum - (self.dig_H4 * 64.0 + self.dig_H5 / 16384.0 * var1))
        var1 = var1 * (self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * var1 * 
                                              (1.0 + self.dig_H3 / 67108864.0 * var1)))
        var1 = var1 * (1.0 - self.dig_H1 * var1 / 524288.0)
        return max(0.0, min(100.0, var1))
    
    def read_compensated_data(self):
        t, p, h = self.read_raw_data()
        temp = self.compensate_temperature(t)
        press = self.compensate_pressure(p)
        hum = self.compensate_humidity(h)
        return temp, press, hum

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        for _ in range(20):  # 等待最多20秒
            if wlan.isconnected():
                break
            time.sleep(1)
    if not wlan.isconnected():
        raise RuntimeError("WiFi connection failed")
    print("WiFi connected:", wlan.ifconfig())
    return wlan

def create_tcp_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    return sock


def pack_data(sensor1_points, sensor2_points):
    """修改后的数据打包函数，处理双传感器数据"""
    num_points = len(sensor1_points) + len(sensor2_points)
    data = struct.pack("!I", num_points)  # 4字节包头（总数据点数）
    
    # 打包传感器1数据
    for point in sensor1_points:
        time_diff, temp, press, _ = point
        # 每个点：1字节传感器ID + 4字节时间差 + 4字节温度 + 4字节气压
        data += struct.pack("!BIff", 1, time_diff, temp, press)
    
    # 打包传感器2数据
    for point in sensor2_points:
        time_diff, temp, press, _ = point
        data += struct.pack("!BIff", 2, time_diff, temp, press)
    
    return data

def get_sensor_thread(sensor, sensor_num):
    """修改后的传感器读取线程，支持指定传感器编号"""
    global sensor_data_buffer1_1, sensor_data_buffer1_2
    global sensor_data_buffer2_1, sensor_data_buffer2_2
    global sensor1_buffer1_is_full, sensor1_buffer2_is_full
    global sensor2_buffer1_is_full, sensor2_buffer2_is_full
    
    # 根据传感器编号选择正确的缓冲区
    if sensor_num == 1:
        buf1 = sensor_data_buffer1_1
        buf2 = sensor_data_buffer1_2
        full_flag1 = sensor1_buffer1_is_full
        full_flag2 = sensor1_buffer2_is_full
    else:
        buf1 = sensor_data_buffer2_1
        buf2 = sensor_data_buffer2_2
        full_flag1 = sensor2_buffer1_is_full
        full_flag2 = sensor2_buffer2_is_full
    
    first_get_sensor_data_time_ms = time.ticks_ms()
    while True:
        try:
            current_time_difference = time.ticks_ms()-first_get_sensor_data_time_ms
            temp, press, hum = sensor.read_compensated_data()
            sensor_data_arr = (current_time_difference, temp, press, sensor_num)  # 添加传感器编号
            
            if len(buf1) < sensor_data_buffer_max_len:
                buf1.append(sensor_data_arr)
            elif len(buf1) >= sensor_data_buffer_max_len and len(buf2) < sensor_data_buffer_max_len:
                buf2.append(sensor_data_arr)
                full_flag1 = True
            elif len(buf1) >= sensor_data_buffer_max_len and len(buf2) >= sensor_data_buffer_max_len:
                full_flag1 = True
                full_flag2 = True
                print(f"Sensor {sensor_num} buffer all full")
                
        except Exception as e:
            print(f"Sensor {sensor_num} read error:", e)
            # 尝试重新初始化传感器
            try:
                sensor = BME280()
            except:
                pass

def main():
    # 初始化网络
    wlan = connect_wifi()
    
    # 初始化传感器
    sensor = BME280(spi_bus=1,cs_pin=7, sck_pin=8, mosi_pin=9, miso_pin=10)
    sensor1 = BME280(spi_bus=1, cs_pin=3, sck_pin=4, mosi_pin=5, miso_pin=6)
    print("Sensor initialized")
    
    global sensor_data_buffer1
    global sensor_data_buffer2
    global sensor_data_buffer1_is_full
    global sensor_data_buffer2_is_full
    global sensor2_buffer1_is_full
    global sensor2_buffer1_is_full

    
    # 创建TCP套接字
    sock = create_tcp_socket()

    # 尝试连接服务器
    while True:
        try:
            print("Connecting to server...")
            sock.connect((SERVER_IP, SERVER_PORT))
            print("Connected to server")
            break
        except Exception as e:
            print("Connection failed:", e)
            time.sleep(5)
    
    last_second = time.time()
        # 创建两个读取线程
    _thread.start_new_thread(get_sensor_thread, (sensor1, 1))
    _thread.start_new_thread(get_sensor_thread, (sensor2, 2))
    while True:
        try:
            current_time = time.time()
            elapsed = current_time - last_second
            
            if elapsed >= 0.1:
                # 处理传感器1的数据
                send_buf1_1, send_buf1_2 = None, None
                if len(sensor_data_buffer1_1) >= sensor_data_buffer_max_len/2:
                    send_buf1_1 = sensor_data_buffer1_1[:]
                    sensor_data_buffer1_1 = []
                    sensor1_buffer1_is_full = False
                
                if len(sensor_data_buffer1_2) >= sensor_data_buffer_max_len/2:
                    send_buf1_2 = sensor_data_buffer1_2[:]
                    sensor_data_buffer1_2 = []
                    sensor1_buffer2_is_full = False
                
                # 处理传感器2的数据
                send_buf2_1, send_buf2_2 = None, None
                if len(sensor_data_buffer2_1) >= sensor_data_buffer_max_len/2:
                    send_buf2_1 = sensor_data_buffer2_1[:]
                    sensor_data_buffer2_1 = []
                    sensor2_buffer1_is_full = False
                
                if len(sensor_data_buffer2_2) >= sensor_data_buffer_max_len/2:
                    send_buf2_2 = sensor_data_buffer2_2[:]
                    sensor_data_buffer2_2 = []
                    sensor2_buffer2_is_full = False
                
                # 发送数据（优先发送主缓冲区）
                if send_buf1_1 or send_buf2_1:
                    try:
                        packed = pack_data(send_buf1_1 if send_buf1_1 else [], 
                                          send_buf2_1 if send_buf2_1 else [])
                        sock.sendall(packed)
                        print(f"Sent {len(send_buf1_1 if send_buf1_1 else []) + len(send_buf2_1 if send_buf2_1 else [])} samples")
                    except Exception as e:
                        print("Send error:", e)
                        # 重新连接处理...
                
                elif send_buf1_2 or send_buf2_2:
                    try:
                        packed = pack_data(send_buf1_2 if send_buf1_2 else [], 
                                          send_buf2_2 if send_buf2_2 else [])
                        sock.sendall(packed)
                        print(f"Sent {len(send_buf1_2 if send_buf1_2 else []) + len(send_buf2_2 if send_buf2_2 else [])} samples")
                    except Exception as e:
                        print("Send error:", e)
                        # 重新连接处理...
                
                last_second = current_time
            
            time.sleep(0.5)
            
        except Exception as e:
            print("Main loop error:", e)
            time.sleep(1)
            # 尝试重新初始化传感器
            try:
                sensor = BME280()
            except:
                pass
            # 尝试重新连接网络
            if not wlan.isconnected():
                wlan = connect_wifi()
            # 尝试重新连接socket
            try:
                sock.close()
            except:
                pass
            sock = create_tcp_socket()
            sock.connect((SERVER_IP, SERVER_PORT))

if __name__ == "__main__":
    main()

