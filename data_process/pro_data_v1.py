import csv

sensor1_list = []
sensor2_list = []
with open('sensor_data.csv', 'r', encoding='utf-8') as file:
    reader = csv.reader(file)
    for row in reader:
        if row[1] == "1" and 80000 <= float(row[2]) <= 120000:
            sensor1_list.append(row)
        elif row[1] == "2" and 80000 <= float(row[2]) <= 120000:
            sensor2_list.append(row)
with open("sensor1.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerows(sensor1_list)  # 批量写入多行[1,4,6](@ref)
with open("sensor2.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerows(sensor2_list)  # 批量写入多行[1,4,6](@ref)