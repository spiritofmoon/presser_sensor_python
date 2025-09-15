# main.py - 双麦克风冲击检测程序 (适用于 2x ICS-43434)

from machine import Pin, I2S
import array
import time

# --- I2S 引脚定义 ---
SCK_PIN = Pin(12)  # 位时钟 (BCLK / SCK)
WS_PIN = Pin(11)   # 声道选择 (LRCLK / WS)
SD_PIN = Pin(13)   # 串行数据 (SD)

# --- I2S 参数 (根据 ICS-43434 手册配置) ---
# 1. 模式选择: 采样率设置为48kHz，激活高性能模式 [cite: 11, 37]
SAMPLE_RATE_IN_HZ = 48000

# 2. 数据格式: 手册要求每个立体声帧有64个SCK周期 [cite: 67]。
#    设置 bits=32, format=STEREO 满足此要求 (32 bits * 2 channels = 64)
BITS_PER_SAMPLE = 32
FORMAT = I2S.STEREO

# 3. I2S 内部环形缓冲区大小
BUFFER_SIZE_IN_BYTES = 8192

# --- 冲击检测阈值 ---
# 这是一个经验值，需要您通过实验来调整。
# 初始可以设置一个较高的值，然后根据实际冲击时的读数来降低。
IMPACT_THRESHOLD = 500000 

# --- 初始化 I2S ---
i2s_in = None
try:
    i2s_in = I2S(
        0,                      # I2S 外设 ID
        sck=SCK_PIN,
        ws=WS_PIN,
        sd=SD_PIN,
        mode=I2S.RX,
        bits=BITS_PER_SAMPLE,
        format=FORMAT,
        rate=SAMPLE_RATE_IN_HZ,
        ibuf=BUFFER_SIZE_IN_BYTES
    )
    print("I2S 初始化成功，采样率: {} Hz (高性能模式)".format(SAMPLE_RATE_IN_HZ))

except ValueError as e:
    print(f"I2S 初始化失败: {e}")
    print("请检查配置参数。")
    
# --- 主循环 ---
def run_impact_test():
    """运行冲击检测主循环"""
    # 创建一个数组来存储读取的音频样本 ('l' = 32位有符号长整型)
    samples_buffer = array.array('l', [0] * 256)
    
    print(f"\n冲击检测已启动，阈值: {IMPACT_THRESHOLD}")
    print("请对纸箱进行碰撞、挤压或跌落测试...")

    while True:
        # 从I2S总线读取数据，填满缓冲区
        num_bytes_read = i2s_in.readinto(samples_buffer)

        # 寻找每个通道的最大振幅
        max_a = 0
        max_b = 0

        if num_bytes_read > 0:
            for i in range(0, len(samples_buffer), 2):
                # 数据格式为 [左样本, 右样本, 左样本, 右样本, ...]
                # Mic A (L/R=GND) 是左声道, Mic B (L/R=3V3) 是右声道
                sample_a_raw = samples_buffer[i]
                sample_b_raw = samples_buffer[i+1]
                
                # ICS-43434 是24位麦克风，数据在32位样本的高位 [cite: 2, 12, 75]。
                # 需要右移8位来对齐，得到真实的24位有符号整数值。
                val_a = abs(sample_a_raw >> 8)
                val_b = abs(sample_b_raw >> 8)
                
                if val_a > max_a: max_a = val_a
                if val_b > max_b: max_b = val_b
        
        # 检查是否有通道超过阈值
        triggered = False
        if max_a > IMPACT_THRESHOLD:
            print(f"--- 冲击! [麦克风 A] 峰值: {max_a} ---")
            triggered = True
        if max_b > IMPACT_THRESHOLD:
            print(f"--- 冲击! [麦克风 B] 峰值: {max_b} ---")
            triggered = True
        
        # 如果触发了，可以暂停一下，避免重复打印
        if triggered:
            time.sleep_ms(200)

# --- 程序入口 ---
if __name__ == "__main__":
    if i2s_in:
        try:
            run_impact_test()
        except KeyboardInterrupt:
            print("\n测试停止。")
        finally:
            # 释放I2S资源
            i2s_in.deinit()
            print("I2S资源已释放。")