import pandas as pd
import matplotlib.pyplot as plt

# 1. 读取数据并统一格式
df1 = pd.read_csv('sensor1.csv', header=None, names=['timestamp', 'sensor_id', 'value'])
df2 = pd.read_csv('sensor2.csv', header=None, names=['timestamp', 'sensor_id', 'value'])

# 2. 合并数据（按时间戳对齐）
combined = pd.merge(
    df1, df2,
    on='timestamp',
    suffixes=('_sensor1', '_sensor2'),  # 区分列名
    how='outer'  # 保留所有时间点[8](@ref)
).sort_values('timestamp')  # 按时间排序

# 3. 时间戳转换（确保为datetime类型）
combined['timestamp'] = pd.to_datetime(combined['timestamp'])

# 4. 绘制双传感器时序图
plt.figure(figsize=(15, 7))

# 绘制传感器1数据（蓝色实线）
plt.plot(
    combined['timestamp'],
    combined['value_sensor1'],
    color='blue',
    linewidth=1,
    label='Sensor 1'
)

# 绘制传感器2数据（红色虚线）
plt.plot(
    combined['timestamp'],
    combined['value_sensor2'],
    color='red',
    linestyle='--',
    linewidth=1,
    label='Sensor 2'
)

# 5. 图表美化
plt.title('Dual-Sensor Time Series Comparison', fontsize=14)
plt.xlabel('Timestamp', fontsize=12)
plt.ylabel('Sensor Value', fontsize=12)
plt.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.6)
plt.xticks(rotation=45)  # 避免时间戳重叠[3](@ref)

# 6. 保存高清图像
plt.tight_layout()
plt.savefig('sensor_comparison.png', dpi=300)
plt.show()