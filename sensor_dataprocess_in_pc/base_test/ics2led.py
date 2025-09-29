# mic_neopixel_test.py
# 一个用于测试单个ICS43434麦克风并用 NeoPixel RGB LED 实时反馈音量的诊断程序
import machine
from machine import Pin, I2S
import neopixel
import time
import struct
import math

# --- 配置区 ---

# 1. 确认您的第一个ICS43434麦克风的I2S引脚
ICS1_SCK_PIN = 12
ICS1_WS_PIN = 11
ICS1_SD_PIN = 13

# 2. 确认您的 NeoPixel RGB LED 连接引脚和数量
#    根据您的示例，ESP32-S3-Zero 的板载 RGB LED 连接到 GPIO 21
LED_PIN = 21
NUM_LEDS = 1

# 3. I2S参数
I2S_SAMPLE_RATE = 16000
I2S_BUFFER_SAMPLES = 512 

# 4. 音量到颜色的映射参数 (*** 您可能需要根据实际情况调整这两个值 ***)
#    这两个值代表了程序期望听到的最小和最大音量（对数值）
#    如果LED颜色变化不明显，尝试调整 MAX_LOUDNESS
#    如果LED在安静时也不是蓝色，尝试调整 MIN_LOUDNESS
MIN_LOUDNESS = 8.0  # 对应LED为纯蓝色的音量阈值
MAX_LOUDNESS = 14.0 # 对应LED为纯红色的音量阈值

# --- 初始化 ---

print("Initializing NeoPixel LED on Pin {}...".format(LED_PIN))
# 初始化 NeoPixel 对象
np = neopixel.NeoPixel(machine.Pin(LED_PIN), NUM_LEDS)

print("Initializing ICS43434 Microphone...")
# 初始化I2S麦克风
mic = I2S(
    0, 
    sck=Pin(ICS1_SCK_PIN), 
    ws=Pin(ICS1_WS_PIN), 
    sd=Pin(ICS1_SD_PIN), 
    mode=I2S.RX, 
    bits=32,
    format=I2S.MONO, 
    rate=I2S_SAMPLE_RATE, 
    ibuf=I2S_BUFFER_SAMPLES * 4 * 2
)
print("Initialization complete. Starting audio loop...")

mic_samples_buffer = bytearray(I2S_BUFFER_SAMPLES * 4)

def map_loudness_to_color(level):
    """将 0.0 到 1.0 的音量级别映射为 蓝->绿->红 的颜色渐变"""
    level = max(0, min(1, level)) # 限制范围
    
    # 将一个级别分成两半
    if level < 0.5:
        # 从 蓝 (0,0,255) 到 绿 (0,255,0)
        r = 0
        g = int(255 * (level * 2))
        b = int(255 * (1 - (level * 2)))
    else:
        # 从 绿 (0,255,0) 到 红 (255,0,0)
        r = int(255 * ((level - 0.5) * 2))
        g = int(255 * (1 - ((level - 0.5) * 2)))
        b = 0
    return (r, g, b)

# --- 主循环 ---
try:
    while True:
        # 1. 读取音频数据
        num_bytes_read = mic.readinto(mic_samples_buffer)

        if num_bytes_read > 0:
            num_samples = num_bytes_read // 4
            samples_32bit = struct.unpack('<%di' % num_samples, mic_samples_buffer)

            # 2. 计算音量 (RMS)
            sum_sq = 0
            for sample in samples_32bit:
                sample_24bit = sample >> 8
                sum_sq += sample_24bit * sample_24bit
            
            mean_sq = sum_sq / num_samples
            rms = math.sqrt(mean_sq)
            log_rms = math.log(rms + 1)
            
            # (可选) 查看实时音量值
            # print("Log RMS: {:.2f}".format(log_rms))

            # 3. 将音量映射到 0.0 - 1.0 的级别
            scaled_loudness = (log_rms - MIN_LOUDNESS) / (MAX_LOUDNESS - MIN_LOUDNESS)

            # 4. 根据级别计算颜色
            color = map_loudness_to_color(scaled_loudness)

            # 5. 更新LED颜色
            np[0] = color
            np.write()

except KeyboardInterrupt:
    print("程序被中断")
finally:
    # 程序结束或中断时，关闭 LED
    print("关闭 LED")
    np[0] = (0, 0, 0)
    np.write()
