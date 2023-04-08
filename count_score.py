#!/usr/bin/python3
'''
    执行命令，并获取命令的stdout
'''
import time
import subprocess
import json
import threading
import os
# 重新编译 cd build && cmake .. && make
os.system("cd build && cmake .. && make")
result = [None] * 5
skip_frame = [None] * 5
# 判断系统是macos还是linux


def judge_os():
    import platform
    system = platform.system()
    if system == "Darwin":
        return "macos"
    elif system == "Linux":
        return "linux"
    else:
        return "windows"


cnt = []


def excute_test(index: int):
    if judge_os() == "macos":
        cmd = f'cd ./Robot && ./Mac-Robot -f -m ./maps/{index}.txt ../build/main'
    elif judge_os() == "linux":
        cmd = f'cd ./Robot && ./Robot -f -m ./maps/{index}.txt ../build/main'
    else:
        raise Exception("not support os")
    print(f"cmd: {cmd} start")
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    data = p.stdout.readlines()

    cnt.append(index)
    try:
        for d in data:
            if b"score" in d:
                _data = d
            if b'skipped frames' in d:
                _data_2 = d
        print(f"index {index}: {_data}")
        res = json.loads(_data)
        result[index] = res['score']
        if _data_2:
            skip_frame[index] = str(_data_2).split(":")[-1][:-3]
            # 二进制转int
            skip_frame[index] = int(skip_frame[index])
            print(f'index {index}: skip frame: {skip_frame[index]}')
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
        time.sleep(1)
        time_now += 1
        if (time_now % 10) == 0:
            print(f"now time: {time_now}")
            if len(cnt) == 4:
                print("over")
                break
    else:
        print("time out")


# 时间线程主线程推出后，会自动退出
time_thread = threading.Thread(target=time_coutn)
time_thread.start()
[t.join() for t in tasks]

sum_score = sum([score for score in result if score != None])
print(f"result: {result[1:]}")
print(f"sum score: {sum_score}")
print(f"skip frame: {skip_frame[1:]}")
