import _thread 
import _queue
import time 
result_queue = queue.Queue()

def worker():
    result = 42
    result_queue.put(result)  # 存储结果

_thread.start_new_thread(worker, ())

# 主线程获取结果
while result_queue.empty():
    time.sleep(0.1)
print(f"Result: {result_queue.get()}")