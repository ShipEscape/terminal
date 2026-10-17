# coding=utf-8
import socket
import json
import os
import time
import threading
from threading import Event
import smbus
import pymysql


# time.sleep(10)

# -------------------------------------------------- #

# lock = threading.Lock()

def dec_to_bcd(val):
    return ((val // 10) << 4) | (val % 10)


def bcd_to_dec(val):
    return (val >> 4) * 10 + (val & 0x0F)
rtc_i2c_bus = 1
rtc_addr = 0x32  # 0011 0010 1/0(R/W), i2cdetect -y 1

# 计时寄存器地址
year_add = 0x06
month_add = 0x05
date_add = 0x04
day_add = 0x03    # 周几
hour_add = 0x02
minutes_add = 0x01
seconds_add = 0x00

# 闹钟1寄存器地址
alarms1_date_add = 0x0B
alarms1_hour_add = 0x09
alarms1_minutes_add = 0x08
alarms1_seconds_add = 0x07
alarms1_on_add = 0x0E          # 0x17

# 控制寄存器地址              # 13
control_CTR1_add = 0x0F       # 0010 0000  0x20 
control_CTR2_add = 0x10       # 0001 0010  0x12

# 状态寄存器地址
status_add = 0x0F             # 0位为A1F

# 扩展控制寄存器地址
control_CTR4_add = 0x19       # 0

rtc_bus = smbus.SMBus(rtc_i2c_bus)


def write_pro():
  global control_CTR1_add
  global control_CTR2_add
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR2_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR2_add, status | 0x80)
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR1_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR1_add, status | 0x84)


def write_end():
  global control_CTR1_add
  global control_CTR2_add
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR1_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR1_add, status & 0x7B)
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR2_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR2_add, status & 0x7F)


def rtc_init():
  global control_CTR1_add
  global control_CTR2_add
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR2_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR2_add, status & 0xBF)  # 1011 1111 0xBF  IM=0
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR1_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR1_add, status & 0xDF)  # 1101 1111 0xDF  INTAF=0
  
  # status = rtc_bus.read_byte_data(rtc_addr, control_CTR3_add)
  # rtc_bus.write_byte_data(rtc_addr, control_CTR3_add, status | 0x80)  # 1000 0000 0x80  ARST=1


def set_time(add_min):
  global rtc_bus, rtc_addr
  global year_add, month_add, date_add, day_add, hour_add, minutes_add, seconds_add
  global alarms1_date_add, alarms1_hour_add, alarms1_minutes_add, alarms1_seconds_add
  global control_CTR1_add, control_CTR2_add
  
  status = rtc_bus.read_byte_data(rtc_addr, control_CTR2_add)
  rtc_bus.write_byte_data(rtc_addr, control_CTR2_add, status | 0x12)  # 0001 0010 0x12
  
  rtc_bus.write_byte_data(rtc_addr, year_add, dec_to_bcd(time.localtime()[0]%2000))
  rtc_bus.write_byte_data(rtc_addr, month_add, dec_to_bcd(time.localtime()[1]) & 0x1F)
  rtc_bus.write_byte_data(rtc_addr, date_add, dec_to_bcd(time.localtime()[2]) & 0x3F)
  rtc_bus.write_byte_data(rtc_addr, hour_add, dec_to_bcd(time.localtime()[3]) & 0x9F)
  rtc_bus.write_byte_data(rtc_addr, minutes_add, dec_to_bcd(time.localtime()[4]) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, seconds_add, dec_to_bcd(time.localtime()[5]) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, day_add, dec_to_bcd(time.localtime()[6]+1) & 0x07)
  
  year_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, year_add))
  month_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, month_add))
  date_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, date_add))
  hour_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, hour_add))
  minutes_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, minutes_add))
  seconds_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, seconds_add))
  day_data = bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, day_add))
  print("20%d-%d-%d %d:%d:%d wday:%d" % (year_data, month_data, date_data, hour_data, minutes_data, seconds_data, day_data))
  
  # init alarms data
  alarms1_month_data = month_data
  alarms1_date_data = date_data
  alarms1_hour_data = hour_data
  alarms1_minutes_data = minutes_data
  alarms1_seconds_data = seconds_data
  
  alarms1_minutes_data = alarms1_minutes_data + add_min
  if alarms1_minutes_data >= 60:
    alarms1_hour_data = alarms1_hour_data + int(alarms1_minutes_data / 60)
    alarms1_minutes_data = alarms1_minutes_data % 60
    
  if alarms1_hour_data >= 24:
    alarms1_date_data = alarms1_date_data + int(alarms1_hour_data / 24)
    alarms1_hour_data = alarms1_hour_data % 24
    
  if month_data == 2:
    if (year_data % 4 == 0) & (year_data % 100 != 0):
      if alarms1_date_data > 29:
        alarms1_month_data += 1
        alarms1_date_data = 1
    else:
      if alarms1_date_data > 28:
        alarms1_month_data += 1
        alarms1_date_data = 1
  elif (month_data == 4) | (month_data == 6) | (month_data == 9) | (month_data == 11):
    if alarms1_date_data > 30:
      alarms1_month_data += 1
      alarms1_date_data = 1
  else:
    if alarms1_date_data > 31:
      alarms1_month_data += 1
      alarms1_date_data = 1
  
  
  rtc_bus.write_byte_data(rtc_addr, alarms1_date_add, dec_to_bcd(alarms1_date_data) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, alarms1_hour_add, dec_to_bcd(alarms1_hour_data) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, alarms1_minutes_add, dec_to_bcd(alarms1_minutes_data) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, alarms1_seconds_add, dec_to_bcd(alarms1_seconds_data) & 0x7F)
  rtc_bus.write_byte_data(rtc_addr, alarms1_on_add, 0x17)
  
  print('alarms date:', bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, alarms1_date_add)))
  print('alarms1 hour:', bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, alarms1_hour_add)))
  print('alarms1 minutes:', bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, alarms1_minutes_add)))
  print('alarms1 seconds:', bcd_to_dec(rtc_bus.read_byte_data(rtc_addr, alarms1_seconds_add)))

write_pro()
rtc_init()
write_end()

reboot_num = 0

def is_online():
  return os.system("ping -c1 -W2 192.168.0.1 >/dev/null 2>&1") == 0

while not is_online():
  time.sleep(1)
  reboot_num += 1
  if reboot_num >= 60:
    os.system("sudo reboot")
print("wlan0")

# 创建套接字
s = socket.socket(socket.AF_INET, socket.SO_REUSEADDR, 0)
s.bind(("192.168.0.168", 8080))

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

# 获取mysql服务信息（测试连接，会输出MySQL版本号）
# print(conn.get_server_info())

from smp import smp_run
from smp import danger_stop_func
from smp import send_to_pc
from smp import alarm_func
from smp import mi48_sleep
while_num = 6
MOD = 2
ear_while = 1
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
  global while_num, ear_while
  global pc_ip
  global s
  global dev_num
  global MOD
  global add_minutes
  global danger_th_start
  global normal_start
  global normal_job_schedule
  global d_mod
  
  while ear_while:
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
    # 创建数据库连接
    conn = pymysql.connect(
        host="192.168.0.200",
        port=3306,
        user="root",
        password="123456",
        database="firealert"
    )
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
            if int(res[2]) != 0:
              add_minutes = int(res[2])
            else:
              add_minutes = 15
            add_min_file = open(add_min_file_path, 'w+')
            add_min_file.write(str(add_minutes))
            add_min_file.close()
          elif res[1] == 2:
            # danger
            #print("res[1]==2")
            if danger_th_start == False:
              MOD = 2
              d_mod = 1
              #os.system('sudo uhubctl -l 2 -a 1')
              #os.system('sudo uhubctl -l 4 -a 1')
          elif res[1] == 3:
            # dangerstop
            #print("res[1]==3")
            if danger_th_start == True:
              #print("res[1]==3")
              MOD = 3
              # danger_th_start = False
          sql = "UPDATE communication SET revise=%s WHERE device_code = %s"
          cursor.execute(sql, (0, dev_num))
          conn.commit()
    cursor.close()
    conn.close()
    # lock.release()
    send_to_pc(s, pc_ip)
    time.sleep(0.5)
    
    # alarm_func(MOD, d_mod, danger_th_start)


def danger_job():
  global lock
  global while_num
  global dev_num
  global pc_ip
  global s
  
  danger_num = 100
  #danger_num = while_num
  #while danger_while_num:
  while True:
    os.system('sudo uhubctl -l 2 -a 1')
    os.system('sudo uhubctl -l 4 -a 1')
    time.sleep(5)
    os.system('sudo uhubctl -l 2 -a 0')
    os.system('sudo uhubctl -l 4 -a 0')
    time.sleep(5)
    os.system('sudo uhubctl -l 2 -a 1')
    os.system('sudo uhubctl -l 4 -a 1')
    print("danger_th_start:", danger_th_start)
    #print("danger_job")
    if danger_th_start == True:
      smp_run(danger_num, dev_num, s, pc_ip)  # danger_stop_func()终止运行
    elif danger_th_start == False:
      break
    # danger_while_num -= danger_num


def normal_job():
  global while_num
  global dev_num
  global pc_ip
  global s
  global add_minutes

  # print('\n\n666\n\n')
  #os.system('sudo uhubctl -l 2 -a 1')
  #os.system('sudo uhubctl -l 4 -a 1')
  time.sleep(0.5)
  smp_run(while_num, dev_num, s, pc_ip)
  #print("qwertyuiop")
  #os.system('sudo uhubctl -l 2 -a 0')
  #os.system('sudo uhubctl -l 4 -a 0')
  time.sleep(0.5)

  # set_time(add_minutes)
  # time.sleep(0.5)
  # os.system("sudo poweroff")

while_num = 20
os.system('sudo uhubctl -l 2 -a 1')
os.system('sudo uhubctl -l 4 -a 1')
time.sleep(1)
smp_run(while_num, dev_num, s, pc_ip)
os.system('sudo uhubctl -l 2 -a 0')
os.system('sudo uhubctl -l 4 -a 0')
time.sleep(2)

ear_thread = threading.Thread(target=ear_job, name='T1')

time.sleep(1)

danger_thread = threading.Thread(target=danger_job, name='T1')

time.sleep(1)
ear_thread.start()
time.sleep(2)

while True:
  # print(danger_thread.is_alive())
  # print("MOD:", MOD)
  if MOD == 0:            # 非MOD123
    if d_mod == 1:
      MOD = 2
      d_mod = 0
    else:
      ear_while = 0
      time.sleep(1)
      mi48_sleep()
      time.sleep(1)
      write_pro()
      set_time(add_minutes)
      write_end()
      time.sleep(0.5)
      os.system("sudo poweroff")
      # MOD = 1
      # break
      # os.system("echo +60 | sudo tee /sys/class/rtc/rtc0/wakealarm")
      # os.system("sudo halt")
      # os.system("sudo poweroff")
  elif MOD == 1:          # 正常模式
    while_num = 20
    # time.sleep(18)
    normal_job()
    #os.system('sudo uhubctl -l 2 -a 0')
    #os.system('sudo uhubctl -l 4 -a 0')
    MOD = 0
  elif MOD == 2:          # 紧急模式
    while_num = 100
    time.sleep(1)
    if danger_th_start == False:
      time.sleep(1)
      print("danger_thread.start!@$\n")
      danger_thread.start()
      danger_th_start = True
    else:
      #print("danger_pass")
      pass
  elif MOD == 3:          # 停止紧急模式
    danger_stop_func()
    danger_th_start = False
    write_pro()
    set_time(add_minutes)
    write_end()
    time.sleep(0.5)
    os.system("sudo poweroff")
    # os.system("echo +60 | sudo tee /sys/class/rtc/rtc0/wakealarm")
    # os.system("sudo halt")
    # os.system("sudo poweroff")
    # danger_thread.join()
    # time.sleep(1)
    # os.system('sudo uhubctl -l 1-1 -a 0')
    # MOD = 0    # stop->normal->normal_job

