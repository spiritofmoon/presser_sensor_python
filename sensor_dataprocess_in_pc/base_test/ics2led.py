# mic_led_test.py
# 一个用于测试单个ICS43434麦克风并用LED灯实时反馈音量的诊断程序

from machine import Pin, I2S, PWM
import time
import struct
import math

# --- 配置区 ---

# 1. 确认您的第一个ICS43434麦克风的I2S引脚
ICS1_SCK_PIN = 12
ICS1_WS_PIN = 11
ICS1_SD_PIN = 13

# 2. 确认您的RGB LED连接在哪个引脚
#    这里我们假设您使用GPIO 21来控制灯的亮度
LED_PIN = 21

# 3. I2S参数
I2S_SAMPLE_RATE = 16000
# 减小缓冲区大小，因为我们是实时处理，不需要缓存太多数据
I2S_BUFFER_SAMPLES = 512

# 4. 音量到亮度的映射参数 (*** 您可能需要根据实际情况调整这两个值 ***)
#    这两个值代表了程序期望听到的最小和最大音量（对数值）
#    如果LED一直很暗，尝试减小 MAX_LOUDNESS
#    如果LED一直很亮，尝试增大 MAX_LOUDNESS 和 MIN_LOUDNESS
MIN_LOUDNESS = 8.0  # 对应LED熄灭的音量阈值 (对数)
MAX_LOUDNESS = 14.0  # 对应LED最亮的音量阈值 (对数)

# --- 程序初始化 ---

print("Initializing LED on Pin {}...".format(LED_PIN))
# 初始化PWM，用于控制LED亮度
# 频率可以设置在1000Hz，人眼不会察觉到闪烁
led = PWM(Pin(LED_PIN), freq=1000, duty_u16=0)

print("Initializing ICS43434 Microphone...")
# 初始化I2S麦克风
mic = I2S(
    0,
    sck=Pin(ICS1_SCK_PIN),
    ws=Pin(ICS1_WS_PIN),
    sd=Pin(ICS1_SD_PIN),
    mode=I2S.RX,
    bits=32,  # 使用32位容器接收24位数据
    format=I2S.MONO,
    rate=I2S_SAMPLE_RATE,
    ibuf=I2S_BUFFER_SAMPLES * 4 * 2  # ibuf需要足够大
)
print("Initialization complete. Starting audio loop...")

# 创建一个缓冲区来存储I2S读取的原始字节数据
# 每个样本32位 = 4字节
mic_samples_buffer = bytearray(I2S_BUFFER_SAMPLES * 4)

# --- 主循环 ---

while True:
    try:
        # 1. 从I2S麦克风读取音频数据
        num_bytes_read = mic.readinto(mic_samples_buffer)

        if num_bytes_read > 0:
            # 2. 将字节数据解包成32位整数样本
            num_samples = num_bytes_read // 4
            samples_32bit = struct.unpack('<%di' % num_samples, mic_samples_buffer)

            # 3. 计算音量 (RMS - 均方根)
            # RMS是衡量交流信号（如音频）能量/幅度的标准方法
            sum_sq = 0
            for sample in samples_32bit:
                # 首先，将32位样本转换为正确的24位值
                sample_24bit = sample >> 8
                # 累加样本的平方
                sum_sq += sample_24bit * sample_24bit

            mean_sq = sum_sq / num_samples
            rms = math.sqrt(mean_sq)

            # 使用对数刻度来更好地匹配人耳对音量的感知
            # +1是为了避免计算log(0)
            log_rms = math.log(rms + 1)

            # (可选) 取消下面的注释来查看实时音量值，方便您调整上面的MIN/MAX_LOUDNESS
            # print("RMS: {:.2f}, Log RMS: {:.2f}".format(rms, log_rms))

            # 4. 将音量映射到LED亮度 (0 - 65535)
            # 首先，将音量值归一化到 0.0 - 1.0 的范围
            # 低于MIN_LOUDNESS的音量视为0，高于MAX_LOUDNESS的音量视为1
            scaled_loudness = (log_rms - MIN_LOUDNESS) / (MAX_LOUDNESS - MIN_LOUDNESS)
            scaled_loudness = max(0, min(1, scaled_loudness))  # 确保值在0和1之间

            # 将归一化后的音量转换为16位的PWM占空比
            duty_value = int(scaled_loudness * 65535)

            # 5. 更新LED亮度
            led.duty_u16(duty_value)

    except Exception as e:
        print("Error:", e)
        time.sleep_ms(100)