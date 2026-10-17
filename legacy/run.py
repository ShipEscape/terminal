# coding=utf-8
import socket
import json
import os
import time
import threading
from threading import Event
import smbus
import pymysql


# -------------------------------------------------- #

# lock = threading.Lock()

reboot_num = 0

def is_online():
  # return os.system("ping -c1 -W2 192.168.0.1 >/dev/null 2>&1") == 0
  return os.system("ping -c1 -W2 127.0.0.1 >/dev/null 2>&1") == 0

while not is_online():
  time.sleep(1)
  reboot_num += 1
  if reboot_num >= 60:
    os.system("sudo reboot")
print("wlan0")

# 创建套接字
s = socket.socket(socket.AF_INET, socket.SO_REUSEADDR, 0)
# s.bind(("192.168.0.168", 8080))
s.bind(("0.0.0.0", 8080))

pc_ip = "192.168.0.200"

add_min_file_path = "/home/xzt/pysenxor/add_minutes.txt"
dev_num_file_path = "/home/xzt/pysenxor/dev_num.txt"

if os.path.isfile(add_min_file_path) and os.path.getsize(add_min_file_path) != 0:
  add_min_file = open(add_min_file_path, 'r')
else:
  add_min_file = open(add_min_file_path, 'w+')
  add_min_file.write(str(1))
  add_min_file.close()
  time.sleep(1)
  add_min_file = open(add_min_file_path, 'r')
add_minutes = int(add_min_file.read())
# add_minutes = 10    # 关机时间
print(add_minutes, type(add_minutes))

# cat /proc/cpuinfo | grep Serial
# RP-[项目代号]-[批次号]-[cpu序列号后8位]
dev_num_file = open(dev_num_file_path, 'r')
dev_num = dev_num_file.read()
print(dev_num)

# 创建数据库连接
conn = pymysql.connect(
    host=pc_ip,
    port=3306,
    user="remote_user",
    password="remote_user",
    database="firealert"
)

# 获取mysql服务信息（测试连接，会输出MySQL版本号）
# print(conn.get_server_info())

from smp import smp_run
from smp import danger_stop_func
from smp import send_to_pc
from smp import alarm_func
from smp import mi48_sleep
while_num = 6
MOD = 2
d_mod = 0
danger_th_start = False
normal_start = False
'''
MOD :
0  normal->job
1  normal,number->normal_min
2  danger
3  dangerstop'''

def ear_job():
  # global lock
  global while_num
  global pc_ip
  global s
  global dev_num
  global MOD
  global add_minutes
  global danger_th_start
  global normal_start
  global normal_job_schedule
  global d_mod
  
  while True:
    '''data, ip = s.recvfrom(1024)
    if ip[0] == pc_ip:
      data_list = data.decode('utf-8').split('-')
      if data_list[0] == dev_num:
        command = data_list[1]
        number = data_list[2]
        # print(command, number)
        if command == '3':    # RP...-dangerstop-num
          if danger_th_start == True:
            MOD = 3
        elif command == '2':  # RP...-danger-num
          MOD = 2
          os.system('sudo uhubctl -l 1-1 -a 1')
        elif command == '1':  # RP...-normal-num
          MOD = 1
          add_minutes = int(number)
          add_min_file = open(add_min_file_path, 'w+')
          add_min_file.write(str(add_minutes))
          add_min_file.close()'''
    # lock.acquire()
    cursor = conn.cursor()
    # 执行查询操作
    cursor.execute('SELECT * FROM communication')
    # 获取查询结果，返回元组
    result = cursor.fetchall()
    for res in result:
      if res[0] == dev_num:
        # print(res)
        if res[3] == 1:
          if res[1] == 1:
            # normal
            MOD = 1
            add_minutes = int(res[2])
            add_min_file = open(add_min_file_path, 'w+')
            add_min_file.write(str(add_minutes))
            add_min_file.close()
          elif res[1] == 2:
            # danger
            MOD = 2
            d_mod = 1
            danger_th_start = False
            os.system('sudo uhubctl -l 2 -a 1')
            os.system('sudo uhubctl -l 4 -a 1')
          elif res[1] == 3:
            # dangerstop
            if danger_th_start == True:
              MOD = 3
          sql = "UPDATE communication SET revise=%s WHERE device_code = %s"
          cursor.execute(sql, (0, dev_num))
          conn.commit()
    cursor.close()
    # lock.release()
    send_to_pc(s, pc_ip)
    
    # alarm_func(MOD, d_mod, danger_th_start)


def danger_job():
  global lock
  global while_num
  global dev_num
  global pc_ip
  global s
  while True:
    print('while_num:', while_num)
    if danger_th_start == True:
      smp_run(while_num, dev_num, s, pc_ip)  # danger_stop_func()终止运行
    elif danger_th_start == False:
      break


def normal_job():
  global while_num
  global dev_num
  global pc_ip
  global s
  global add_minutes

  # print('\n\n666\n\n')
  os.system('sudo uhubctl -l 2 -a 1')
  os.system('sudo uhubctl -l 4 -a 1')
  time.sleep(1)
  smp_run(while_num, dev_num, s, pc_ip)
  os.system('sudo uhubctl -l 2 -a 0')
  os.system('sudo uhubctl -l 4 -a 0')
  time.sleep(1)

  # set_time(add_minutes)
  # os.system("sudo poweroff")

ear_thread = threading.Thread(target=ear_job, name='T1')

danger_thread = threading.Thread(target=danger_job, name='T1')

ear_thread.start()

while_num = 0
os.system('sudo uhubctl -l 2 -a 1')
os.system('sudo uhubctl -l 4 -a 1')
time.sleep(1)
smp_run(while_num, dev_num, s, pc_ip)
os.system('sudo uhubctl -l 2 -a 0')
os.system('sudo uhubctl -l 4 -a 0')
time.sleep(1)

while True:
  # print(danger_thread.is_alive())
  # print(MOD)
  if MOD == 0:
    if d_mod == 1:
      MOD = 2
      d_mod = 0
    else:
      time.sleep(1)
      mi48_sleep()
      time.sleep(1)
      # MOD = 1
      # break
      os.system("echo +60 | sudo tee /sys/class/rtc/rtc0/wakealarm")
      os.system("sudo halt")
      # os.system("sudo poweroff")
  elif MOD == 1:
    while_num = 10
    # time.sleep(18)
    normal_job()
    os.system('sudo uhubctl -l 2 -a 0')
    os.system('sudo uhubctl -l 4 -a 0')
    MOD = 0
  elif MOD == 2:
    while_num = 1000
    time.sleep(1)
    if danger_th_start == False:
      danger_thread.start()
      danger_th_start = True
    else:
      pass
  elif MOD == 3:
    danger_stop_func()
    danger_th_start = False
    os.system("echo +60 | sudo tee /sys/class/rtc/rtc0/wakealarm")
    os.system("sudo halt")
    # os.system("sudo poweroff")
    # danger_thread.join()
    # time.sleep(1)
    # os.system('sudo uhubctl -l 1-1 -a 0')
    # MOD = 0    # stop->normal->normal_job

