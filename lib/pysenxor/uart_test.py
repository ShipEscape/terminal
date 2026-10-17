# coding=utf-8
import serial
import time
import os

os.system('sudo uhubctl -l 2 -a 1')
os.system('sudo uhubctl -l 4 -a 1')

control_com = "/dev/radar_uart_0"
data_com = "/dev/radar_uart_1"

control_ser = serial.Serial("%s" % control_com, 115200, timeout=0.5)  # 定义串口对象
data_ser = serial.Serial("%s" % data_com, 921600, timeout=0.5)  # 定义串口对象

while True:  # 判断串口是否正常连接，且防止报错
    # noinspection PyBroadException
    try:
        control_ser = serial.Serial("%s" % control_com, 115200, timeout=0.5)  # 定义串口对象
        if control_ser.isOpen():  # 判断串口是否成功打开
            print("%s连接成功。" % control_com)
            os.system('sudo chmod 777 %s' % control_com)
            break
        else:
            print("%s连接失败。" % control_com)
    except:
        print("毫米波雷达未接入或端口号错误。")
        os.system('sudo uhubctl -l 2 -a 1')
        os.system('sudo uhubctl -l 4 -a 1')
        time.sleep(2)
while True:  # 判断串口是否正常连接，且防止报错
    # noinspection PyBroadException
    try:
        data_ser = serial.Serial("%s" % data_com, 921600, timeout=0.5)  # 定义串口对象
        if data_ser.isOpen():  # 判断串口是否成功打开
            print("%s连接成功。" % data_com)
            os.system('sudo chmod 777 %s' % data_com)
            break
        else:
            print("%s连接失败。" % data_com)
    except:
        print("毫米波雷达未接入或端口号错误。")
        os.system('sudo uhubctl -l 2 -a 1')
        os.system('sudo uhubctl -l 4 -a 1')
        time.sleep(2)
print('串口对象初始化完成')
