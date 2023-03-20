#!/usr/bin/python3
'''
    执行命令，并获取命令的stdout
'''
import time
import subprocess
import json
import threading

result = [None]*5


def excute_test(index: int):
    cmd = f'cd ./Robot && ./Robot -m ./maps/{index}.txt ../build/main'
    print(f"cmd: {cmd} start")
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    data = p.stdout.readlines()
    try:
        _data = data[-2]
        print(f"index {index}: ", _data)
        res = json.loads(_data)
        result[index] = res['score']
    except Exception as e:
        print(f"task {index} error: ", e)


# #
# a = b'{"status":"Successful","score":819974}'
# 并发执行4个进程，并获取每个进程的stdout
tasks = [threading.Thread(target=excute_test, args=(i,)) for i in range(1, 5)]
[t.start() for t in tasks]

# 计时进程, 每个10s输出一次


def time_coutn():
    time_now = 0
    while time_now < 180:
        time.sleep(10)
        time_now += 10
        print(f"now time: {time_now}")
    print("time out")


# 时间线程主线程推出后，会自动退出
time_thread = threading.Thread(target=time_coutn)
time_thread.start()
[t.join() for t in tasks]

sum_score = sum([score for score in result if score != None])
print(f"result: {result[1:]}")
print(f"sum score: {sum_score}")
