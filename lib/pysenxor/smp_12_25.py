# coding=utf-8
import socket
import serial
import time
import struct
import json
import math
import RPi.GPIO as GPIO
import pickle
import os
import sys
import math
import copy
import numpy as np

import multiprocessing as mp

import argparse
import smbus
from spidev import SpiDev
from gpiozero import Pin, DigitalInputDevice, DigitalOutputDevice

import struct
import smbus2
import time
from subprocess import call

R = 0.3
Vs = 0.5
weight = 1.0
epsilon = R**2 + weight * Vs**2
min_neighbors = 1

os.system('sudo uhubctl -l 2 -a 1')
os.system('sudo uhubctl -l 4 -a 1')

send_data_list = []
send_flag = False
def send_to_pc(s, pc_ip):
  global send_data_list
  global send_flag
  if send_flag == True:
    for strstr_data in send_data_list:
      s.sendto(strstr_data.encode(), (pc_ip, 9090))
    send_flag = False
    send_data_list.clear()
    print('已发送数据至:', pc_ip, '9090')

danger_stop_t = False
def danger_stop_func():
  global danger_stop_t
  danger_stop_t = True

def scale(x, srcRange, dstRange):
    return (x-srcRange[0])*(dstRange[1]-dstRange[0])/(srcRange[1]-srcRange[0])+dstRange[0]

# -------------------------------------------------- #
cu_list = []
output_list = []
means_list = []

now_point = []  # x y z velocity
now_data_point_list = []  # 存当前data的now_point
json_x_list = []
r = 0
json_y_list = []
json_z_list = []
radian = 0
a = 0
temp_list = []  # 临时列表
temp_human_list = []  # 存当前人的列表
human_list = []  # 存当前人的列表
temp_num = 0  # 临时变量
temp_judge = 0
point_num = 0
x_weight = 1  # x权重
z_weight = 0  # z权重
y_weight = -0.1  # y权重

Alarm_state = 1  # 1:normal
alarm_low = 0
alarm_danger = False

send_list = []  # 发送

bus_voltage = 0  # 输出电压V
current = 0      # 电流mA
dl_p = 0         # 剩余电量%
dl_low = 0       # 防止过放

def alarm_func(mod, dmod, d_t_s):
  global alarm_danger
  if alarm_danger == True:
    mod = 2
    dmod = 1
    d_t_s = False
    
def spherical_to_cartesian(azimuth, elevation, range_val):
    # 注意：先转换为浮点数并考虑单位（这里是弧度）
    az_rad = azimuth * (math.pi / 128)  # 假设int8_t映射到[-π,π]
    el_rad = elevation * (math.pi / 128)
    
    x = range_val * math.cos(el_rad) * math.cos(az_rad)
    y = range_val * math.cos(el_rad) * math.sin(az_rad)  
    z = range_val * math.sin(el_rad)
    
    return x, y, z    

# -------------------------------------------------- #

def readVoltage(bus):

     address = 0x36
     read = bus.read_word_data(address, 2)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     voltage = swapped * 1.25 /1000/16
     return voltage


def readCapacity(bus):

     address = 0x36
     read = bus.read_word_data(address, 4)
     # print("read:", read)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     # print("swapped:", swapped)
     capacity = swapped/256
     return capacity
     
dbus = smbus2.SMBus(1)

time.sleep(1)

# ------------------------热成像------------------------ #
from senxor.mi48 import MI48, DATA_READY, format_header, format_framestats
from senxor.utils import data_to_frame, cv_filter
from senxor.interfaces import SPI_Interface, I2C_Interface
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--record', default=False, dest='record',
                        action='store_true', help='Record data'),
    parser.add_argument('-fps', '--framerate', default=7,
                        type=float, help='Bobcat framerate', dest='fps')
    parser.add_argument('-c', '--colormap', default='rainbow2', type=str,
                        help='Colormap')
    args = parser.parse_args()
    return args

args = parse_args()		# Namespace(record=False, fps=7, colormap='rainbow2')

RPI_GPIO_I2C_CHANNEL = 1	# ls /dev/*i2c*
MI48_I2C_ADDRESS = 0x40		# i2cdetect -y 1
MI48_bus = smbus.SMBus(RPI_GPIO_I2C_CHANNEL)
i2c = I2C_Interface(MI48_bus, MI48_I2C_ADDRESS)
# print(i2c)

# =======================
# MI48 SPI Stuff:
# =======================
MI48_SPI_MODE = 0b00
MI48_SPI_BITS_PER_WORD = 8   # cannot be handled with /dev/spidev-x.y and python on RPi 3B+; must work with default 8
MI48_SPI_LSBFIRST = False    # this appears to be a read-only on RPi
MI48_SPI_CSHIGH = True
# MI48_SPI_MAX_SPEED_HZ = 7800000
# MI48_SPI_MAX_SPEED_HZ = 15600000
MI48_SPI_MAX_SPEED_HZ = 31200000
MI48_SPI_CS_DELAY = 0.0001   # delay between asserting/deasserting CS_N and initiating/stopping clock/data

RPI_GPIO_SPI_BUS = 0		# ls /dev/*spi*
RPI_GPIO_SPI_CE_MI48 = 0
SPI_XFER_SIZE_BYTES = 160 	# bytes
spi = SPI_Interface(SpiDev(RPI_GPIO_SPI_BUS, RPI_GPIO_SPI_CE_MI48),
                    xfer_size=SPI_XFER_SIZE_BYTES)
print(spi.device)
spi.device.mode = MI48_SPI_MODE
spi.device.max_speed_hz = MI48_SPI_MAX_SPEED_HZ
spi.device.bits_per_word = 8
spi.device.lsbfirst = False
# spi.device.no_cs = True
# print(spi)

# SPI CS
mi48_spi_cs_n = DigitalOutputDevice("BCM7", active_high=False,
                                    initial_value=False)
# ===============================
# Configure DATA_READY and RESET
# ===============================
use_data_ready_pin = True
if use_data_ready_pin:
    mi48_data_ready = DigitalInputDevice("BCM24", pull_up=False)
mi48_reset_n = DigitalOutputDevice("BCM23", active_high=False,
                                   initial_value=True)

# ==============================
# Create an MI48 interface object
# ==============================
class MI48_reset:
    def __init__(self, pin,
                 assert_seconds=0.000035,
                 deassert_seconds=0.050):
        self.pin = pin
        self.assert_time = assert_seconds
        self.deassert_time = deassert_seconds

    def __call__(self):
        print('Resetting the MI48...')
        self.pin.on()
        time.sleep(self.assert_time)
        self.pin.off()
        time.sleep(self.deassert_time)
        print('Done.')

try:
  mi48 = MI48([i2c, spi], data_ready=mi48_data_ready,
              reset_handler=MI48_reset(pin=mi48_reset_n))
except TimeoutError as e:
  os.system("sudo reboot")

# print out camera info
camera_info = mi48.get_camera_info()
mi48.set_fps(args.fps)

# see if filtering is available in MI48 and set it up
if int(mi48.fw_version[0]) >= 2:
    # Enable filtering with default strengths
    mi48.enable_filter(f1=True, f2=True, f3=False)

    # If needed, set a temperature offset across entire frame
    # e.g. if overall accuracy (at product level) seems to be 
    # 0.7 above the blackbody, then we need to subtract 0.7 
    # from the readout of the MI48:
    # mi48.set_offset_corr(-5.55)
    #
    # However, for most applications the factory level, per pixel
    # calibration is sufficient, so keep offset 0
    mi48.set_offset_corr(0.0)
    
# initiate continuous frame acquisition
mi48.disable_low_netd()
with_header = True

# enable saving to a file
if args.record:
    filename = get_filename(mi48.camera_id_hex)
    fd_data = open(os.path.join('.', filename+'.dat'), 'w')

mi48.start(stream=True, with_header=with_header)


def mi48_sleep():
  global MI48_bus, MI48_I2C_ADDRESS, mi48
  mi48.stop_capture()
  MI48_bus.write_byte_data(MI48_I2C_ADDRESS, 0xB5, 0x80)

# ------------------------热成像------------------------ #


# ------------------------毫米波雷达------------------------ #
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, GPIO.HIGH)

now_data_point_num = 0  # 存储进度
# 命令数组
command = ["sensorStop\n",
           "flushCfg\n",
           "dfeDataOutputMode 1\n",
           "channelCfg 15 7 0\n",
           "adcCfg 2 1\n",
           "adcbufCfg -1 0 1 1 1\n",
           "lowPower 0 0\n",
           "profileCfg 0 60.75 30 25 59.1 394758 0 54.71 1 96 2950 2 1 36\n",
           "chirpCfg 0 0 0 0 0 0 0 1\n",
           "chirpCfg 1 1 0 0 0 0 0 2\n",
           "chirpCfg 2 2 0 0 0 0 0 4\n",
           "frameCfg 0 2 96 0 55.00 1 0\n",

           "dynamicRACfarCfg -1 4 4 2 2 8 12 4 12 5.00 8.00 0.40 1 1\n",
           "staticRACfarCfg -1 6 2 2 2 8 8 6 4 8.00 15.00 0.30 0 0\n",
           "dynamicRangeAngleCfg -1 0.75 0.0010 1 0\n",
           "dynamic2DAngleCfg -1 3 0.0300 1 0 1 0.30 0.85 8.00\n",
           "staticRangeAngleCfg -1 0 8 8\n",

           "antGeometry0 -1 -1 0 0 -3 -3 -2 -2 -1 -1 0 0\n",
           "antGeometry1 -1 0 -1 0 -3 -2 -3 -2 -3 -2 -3 -2\n",
           "antPhaseRot 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1\n",
           "fovCfg -1 70.0 70.0\n",
           "compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0\n",

           "staticBoundaryBox -3 3 0.5 7.5 0 3\n",
           "boundaryBox -4 4 0 8 0 3\n",
           "sensorPosition 2 0 15\n",
           "gatingParam 3 2 2 3 4\n",
           "stateParam 3 3 12 500 5 6000\n",
           "allocationParam 20 100 0.1 20 0.5 20\n",
           "maxAcceleration 0.1 0.1 0.1\n",
           "trackingCfg 1 2 800 30 46 96 55\n",
           "presenceBoundaryBox -3 3 0.5 7.5 0 3\n",

           "sensorStart\n",
           "sensorStop\n"]

command_len = len(command)  # 命令列表的长度

stop_start_num = 0
person_number = 0
radar_state = 0  # 雷达状态：1为启动，0为停止
while_num = 0
control_com = "/dev/radar_uart_0"
data_com = "/dev/radar_uart_1"
data_header_len = 40

control_ser = serial.Serial("%s" % control_com, 115200, timeout=0.5)  # 定义串口对象
data_ser = serial.Serial("%s" % data_com, 921600, timeout=0.5)  # 定义串口对象

def usb_init():
  global contol_ser
  global data_ser
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

# data_ser.set_buffer_size(rx_size=115200, tx_size=115200)
# control_ser.set_buffer_size(rx_size=1024, tx_size=1024)'''


# 雷达默认启动
radar_state = 1
while_num = 0
while while_num < command_len - 1:  # 到start
    control_ser.write(command[while_num].encode())
    time.sleep(0.05)
    while_num += 1


def start_radar_def():  # 启动
    global radar_state
    if radar_state == 1:
        print("当前雷达已为启动状态")
    elif radar_state == 0:
        # while_num = 0
        # while while_num < command_len - 1:  # 到start
        #     control_ser.write(command[while_num].encode())
        #     time.sleep(0.05)
        #     while_num += 1
        control_ser.write("sensorStart 0\n".encode())
        print("雷达开始")
        radar_state = 1


def stop_radar_def():  # 停止
    global radar_state
    if radar_state == 0:
        print("当前雷达已为停止状态")
    elif radar_state == 1:
        control_ser.write("sensorStop\n".encode())
        print("雷达停止")
        radar_state = 0
        data_ser.flushInput()
        data_ser.flushOutput()



def _region_query(points, idx, epsilon, weight):
    """
    邻域查询:
    距离度量 = (dx^2 + dy^2) + weight * (dv^2)
    """
    diffs = points[:, :2] - points[idx, :2]          # (N,2)
    spatial_sq = np.sum(diffs * diffs, axis=1)       # 平面距离平方
    dv = points[:, 3] - points[idx, 3]               # doppler 差
    combined = spatial_sq + weight * (dv * dv)
    neighbors = np.nonzero(combined <= epsilon)[0]
    return neighbors


def dbscan_mmwave(points, epsilon, weight, min_neighbors):
    """
    DBSCAN for mmWave point cloud with [x,y,z,doppler] input,
    只用 x,y,doppler，输出每个簇的中心坐标 [[x,y,z], ...]，z=0。

    参数:
      points : (N,4) numpy array, 每个点 [x,y,z,doppler]
      epsilon : 阈值
      weight : doppler 权重
      min_neighbors : 至少邻居数 (不含自身)

    返回:
      cluster_centers: list of [x,y,z]
    """
    N = points.shape[0]
    labels = np.zeros(N, dtype=np.int32)
    visited = np.zeros(N, dtype=bool)
    cluster_id = 0

    for i in range(N):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = _region_query(points, i, epsilon, weight)
        if len(neighbors) - 1 < min_neighbors:
            labels[i] = 0
            continue
        cluster_id += 1
        labels[i] = cluster_id
        seeds = list(neighbors)
        si = 0
        while si < len(seeds):
            p = seeds[si]
            if not visited[p]:
                visited[p] = True
                neigh_p = _region_query(points, p, epsilon, weight)
                if len(neigh_p) - 1 >= min_neighbors:
                    for nb in neigh_p:
                        if nb not in seeds:
                            seeds.append(nb)
            if labels[p] == 0:
                labels[p] = cluster_id
            si += 1

    # 输出每个簇的中心点 [x, y, 0]
    cluster_centers = []
    for cid in range(1, cluster_id + 1):
        idxs = np.nonzero(labels == cid)[0]
        pts = points[idxs]
        if len(pts) == 0:
            continue
        x_center = float(pts[:, 0].mean())
        y_center = float(pts[:, 1].mean())
        cluster_centers.append([x_center, y_center, 0.0])

    return cluster_centers


def plt_run(dev_num, s, pc_ip):
    global now_data_point_list, cu_list, means_list
    global epsilon, weight, min_neighbors
    global Point_number, Diagonal_max
    global human_list
    global temp_list
    global temp_human_list
    global temp_num
    global temp_judge
    global point_num
    global x_weight
    global z_weight
    global person_number
    global r
    global a
    global radian
    global Alarm_state, alarm_low, alarm_danger
    global send_list
    global bus_voltage
    global current
    global dl_p, dl_low
    global json_x_list, json_y_list, json_z_list
    global dict_json_list
    global send_data_list, send_flag
    
    # print("now_data_point_list:", now_data_point_list)
    # print("len:", len(now_data_point_list))
    #for abc in now_data_point_list:
    #  json_x_list.append(scale(abc[0], (-10, 10), (-20, 20)))
    #  json_y_list.append(scale(abc[1], (0, 10), (0, 40)))
    #  json_z_list.append(scale(abc[2], (-2, 2), (0, 3)))
    
    # global lock
    if len(now_data_point_list) <= 10:
      min_neighbors = 1
    else:
      min_neighbors = 2
    
    centers = dbscan_mmwave(np.array(now_data_point_list), epsilon, weight, min_neighbors)
    #print("centers:", centers)
    #if len(now_data_point_list) != 0:
    #  for now_list in now_data_point_list:
    #    json_x_list.append(now_list[0])
    #    json_y_list.append(now_list[1])
    #    json_z_list.append(now_list[2])
    
    for means_l in centers:
      if means_l[0] < 10 and means_l[0] > -10 and means_l[1] < 10:
        json_x_list.append(means_l[0])
        json_y_list.append(means_l[1])
        json_z_list.append(means_l[2])
        #json_x_list.append(scale(means_l[0], (-10, 10), (-20, 20)))
        #json_y_list.append(scale(means_l[1], (0, 10), (0, 40)))
        #json_z_list.append(scale(means_l[2], (-2, 2), (0, 3)))

    person_number = len(json_x_list)

    Alarm_state = GPIO.input(17)
    if Alarm_state == 0:
      alarm_low += 1
      if alarm_low >= 30:
        alarm_danger = True
        print("烟雾报警，进入紧急模式")
    else:
      alarm_low = 0
    
    time.sleep(0.6)
    bus_voltage = readVoltage(dbus)
    dl_p = readCapacity(dbus)
    if dl_p > 100:
      dl_p=100
    if dl_p < 0:
      dl_p=0
    
    if bus_voltage < 3.20:
      dl_low += 1
      if dl_low >= 30:
        print("过放")
    else:
      dl_low = 0

    # thermal
    time.sleep(0.1)
    if hasattr(mi48, 'data_ready'):
        mi48.data_ready.wait_for_active()
    else:
        data_ready = False
        while not data_ready:
            # print("okt")
            time.sleep(0.01)
            data_ready = mi48.get_status() & DATA_READY
    # read the frame
    # assert the spi_cs, delay a bit then read
    mi48_spi_cs_n.on()
    # print("okq")
    time.sleep(MI48_SPI_CS_DELAY)
    # print("okw")
    thermal_data, header = mi48.read()
    # print("thermal_data:", type(thermal_data))
    # print("oke")
    if thermal_data is None:
        # print("oka")
        mi48.stop(stop_timeout=1.0)
        sys.exit(1)
    time.sleep(MI48_SPI_CS_DELAY)
    mi48_spi_cs_n.off()

    
    '''dict_json = {'thermal_data': thermal_data.tolist(), 'dev_id': dev_num, 'dl': str('%.2f' % dl_p), 'people_number': str(person_number), 'x': json_x_list, 'y': json_y_list, 'z': json_z_list, 'Smoke': str(Alarm_state)}'''
    
    # print(json_x_list, json_y_list, json_z_list)
    dict_json_0 = {'device_code': dev_num, 'title': 'TD1', 'dl': str('%.2f' % dl_p), 'Alarm': str(Alarm_state)}    # 'dl': str('%.2f' % dl_p)
    dict_json_1 = {'device_code': dev_num, 'title': 'TD2', 'people_number': str(person_number), 'x': json_x_list, 'y': json_y_list, 'z': json_z_list}
    dict_json_2 = {'device_code': dev_num, 'title': 'TD3', 'temperature': thermal_data.tolist()}

    # dict_json_0 = {'device_code': 'RP001001798f051b', 'title': 'TD1', 'dl': str('%.2f' % dl_p), 'Alarm': str(1)}

    # print('dict_json_0:', dict_json_0)
    print('dict_json_1:', dict_json_1)
    # print('dict_json_2:', dict_json_2)
    
    dict_json_list = []
    dict_json_list.append(dict_json_0)
    dict_json_list.append(dict_json_1)
    dict_json_list.append(dict_json_2)

    # lock.acquire()
    if send_flag == False:
      for dict_json in dict_json_list:
        str_data = json.dumps(dict_json)
        # print('str_data:', str_data)
        str_data = str(str_data)
        str_data = str_data.replace(chr(91), chr(34))
        str_data = str_data.replace(chr(93), chr(34))
        str_data = str_data.replace(' ', '')
        list_data = list(str_data)
        c_num = 0
        while True:
            c = list_data[c_num]
            if c == '{':
                list_data.insert(c_num + 1, '\n')
            elif c == ',' and list_data[c_num + 1] == '"':
                list_data.insert(c_num + 1, '\n')
            elif c == '}':
                list_data.insert(c_num, '\n')
                # list_data.insert(c_num, ',')
                break
            c_num += 1
  
        str_data = ''.join(list_data)
        # print(str_data)
  
        # send_list.append(data)
        # send_list.append(str_data)
  
        send_data_list.append(str_data)
      #lock.release()
      send_flag = True

    time.sleep(0.05)
    person_number = 0
    send_list.clear()
    temp_list.clear()
    temp_human_list.clear()
    json_x_list.clear()
    json_y_list.clear()
    json_z_list.clear()
    dict_json_list.clear()
    cu_list.clear()
    means_list.clear()
    # print("\n\n\n\n Debug2 \n\n\n\n")


def timer_run(dev_num, s, pc_ip):  # 主要逻辑函数
    global now_data_point_list
    global while_num
    global now_data_point_num
    global stop_start_num
    global radar_state
    global command_len, data_header_len
    global control_ser
    global data_ser

    if radar_state == 1:
        # start_time = time.time()
        # data = data_ser.readline()
        # data_ser.flushInput()
        data = data_ser.read(data_header_len)
        # 打印1
        if len(data) == 0 and radar_state == 1:
            control_ser.close()
            data_ser.close()
            os.system('sudo uhubctl -l 2 -a 0')
            os.system('sudo uhubctl -l 4 -a 0')
            time.sleep(1)
            os.system('sudo uhubctl -l 2 -a 1')
            os.system('sudo uhubctl -l 4 -a 1')
            time.sleep(1)
            os.system('ls -l /dev/radar_uart_0')
            os.system('ls -l /dev/radar_uart_1')
            # os.system('fuser /dev/radar_uart_0')
            # os.system('fuser /dev/radar_uart_1')
            # print("data length为0，请断电雷达并重启程序")
            usb_init()
            if control_ser.isOpen():
              pass
            else:
              control_ser.open()
            if data_ser.isOpen():
              pass
            else:
              data_ser.open()
            radar_state = 1
            time.sleep(1)
            while_num = 0
            while while_num < command_len - 1:  # 到start
                control_ser.write(command[while_num].encode())
                time.sleep(0.05)
                while_num += 1
            data = data_ser.read(data_header_len)
        elif len(data) == 0 and radar_state == 0:
            return

        # 打印2

        # 本次传输的数据总长度
        data_len = (data[12] + data[13] * (16 * 16) + data[14] * (16 * 16) * (16 * 16) +
                    data[15] * (16 * 16) * (16 * 16) * (16 * 16)) - data_header_len

        # print("\ndata_len:", data_len)

        # 读取数据
        data = data_ser.read(data_len)

        # 打印3

        while_num = 0
        # print("start")
        #flag_11 = 0
        #flag_22 = 0
        flag_while = 0
        while while_num < len(data):
            #if flag_11 ==0:
            #  print("while_1")
            #  flag_11 = 1
            if len(data) - while_num <= 8:
                # 余下为填充
                break
            # TLV 类型、长度
            TLV_type = data[while_num] + \
                       data[while_num + 1] * (16 * 16) + \
                       data[while_num + 2] * (16 * 16) * (16 * 16) + \
                       data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)

            TLV_len = data[while_num + 4] + \
                      data[while_num + 5] * (16 * 16) + \
                      data[while_num + 6] * (16 * 16) * (16 * 16) + \
                      data[while_num + 7] * (16 * 16) * (16 * 16) * (16 * 16)
            if TLV_len >= len(data):
                # 余下为填充
                break
            #print("TLV type:%d" % TLV_type, "   ", "TLV len:%d" % TLV_len)

            # TLV 值
            while_num += 8
            # print("***Debug:", while_num)
            end = while_num + TLV_len

            while while_num < end:
                flag_while = 1
                #if flag_22 ==0:
                #  print("while_2")
                #  flag_22 = 1
                # print("test:::", while_num, end)
                if TLV_type == 1010:
                    now_data_point_list.append([])
                    
                    track_id = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    while_num += 4

                    pos_x = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    pos_x = struct.unpack('!f', struct.pack('!I', pos_x))[0]
                    while_num += 4
                    pos_y = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    pos_y = struct.unpack('!f', struct.pack('!I', pos_y))[0]
                    while_num += 4
                    pos_z = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    pos_z = struct.unpack('!f', struct.pack('!I', pos_z))[0]
                    while_num += 4

                    vel_x = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    vel_x = struct.unpack('!f', struct.pack('!I', vel_x))[0]
                    while_num += 4
                    vel_y = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    vel_y = struct.unpack('!f', struct.pack('!I', vel_y))[0]
                    while_num += 4
                    vel_z = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    vel_z = struct.unpack('!f', struct.pack('!I', vel_z))[0]
                    while_num += 4

                    acc_x = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    acc_x = struct.unpack('!f', struct.pack('!I', acc_x))[0]
                    while_num += 4
                    acc_y = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    acc_y = struct.unpack('!f', struct.pack('!I', acc_y))[0]
                    while_num += 4
                    acc_z = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    acc_z = struct.unpack('!f', struct.pack('!I', acc_z))[0]
                    while_num += 4

                    ec_list = []
                    ec_num = 16
                    while ec_num > 0:
                        ec = data[while_num] + \
                                data[while_num + 1] * (16 * 16) + \
                                data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                        ec = struct.unpack('!f', struct.pack('!I', ec))[0]
                        while_num += 4
                        ec_list.append(ec)
                        ec_num -= 1

                    g = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    g = struct.unpack('!f', struct.pack('!I', g))[0]
                    while_num += 4

                    confidence_level = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    confidence_level = struct.unpack('!f', struct.pack('!I', confidence_level))[0]
                    while_num += 4

                    print("track_id:", track_id,
                          "pos_x:", pos_x, "pos_y:", pos_y, "pos_z:", pos_z,
                          "vel_x:", vel_x, "vel_y:", vel_y, "vel_z:", vel_z,
                          "acc_x:", acc_x, "acc_y:", acc_y, "acc_z:", acc_z,
                          "ec:", ec_list,
                          "confidence level:", confidence_level)


                    # 将坐标加入列表并清空临时列表
                    #print(now_data_point_list)
                    now_data_point_list[now_data_point_num].append(pos_x)
                    now_data_point_list[now_data_point_num].append(pos_y)
                    now_data_point_list[now_data_point_num].append(pos_z)
                    now_data_point_list[now_data_point_num].append(confidence_level)
                    now_data_point_num += 1

                elif TLV_type == 1011:
                    # print(data[while_num], end=" ")
                    while_num += 1

                elif TLV_type == 1012:
                    # print(data[while_num], end=" ")
                    while_num += 1

                elif TLV_type == 1020:
                    #elevation = data[while_num]
                    #elevation = struct.unpack('!I', struct.pack('!I', elevation))[0]
                    while_num += 1
                    #azimuth = data[while_num]
                    #azimuth = struct.unpack('!I', struct.pack('!I', azimuth))[0]
                    #while_num += 1
                    #doppler = data[while_num] + data[while_num + 1] * (16 * 16)
                    #doppler = struct.unpack('!I', struct.pack('!I', doppler))[0]
                    #while_num += 2
                    #range_a = data[while_num] + data[while_num + 1] * (16 * 16)
                    #range_a = struct.unpack('!I', struct.pack('!I', range_a))[0]
                    #while_num += 2
                    #snr = data[while_num] + data[while_num + 1] * (16 * 16)
                    #snr = struct.unpack('!I', struct.pack('!I', snr))[0]
                    #while_num += 2
                    #print("elevation:", elevation, "azimuth:", azimuth, "doppler:", doppler, 
                    #      "range_a:", range_a, "snr:", snr)

                    #point_cloud_x, point_cloud_y, point_cloud_z = spherical_to_cartesian(elevation, azimuth, range_a)
                    #print("point_cloud_x, y, z:", point_cloud_x, point_cloud_y, point_cloud_z)

                elif TLV_type == 1021:
                    # print(data[while_num], end=" ")
                    while_num += 1
                else:
                  break

        # 该段数据分析结束
        # print("end")

        # 使用
        # print("flag_while:", flag_while)
        if flag_while == 1:
          plt_run(dev_num, s, pc_ip)
        # if stop_start_num >= 14:
        #     plt_run()

        # print("now_data_point_list:", now_data_point_list, "   ", "len:", len(now_data_point_list))
        # time.sleep(0.05)
        # 重新初始化，然后下一轮循环
        while_num = 0
        now_data_point_num = 0
        # print(now_data_point_list)
        now_data_point_list.clear()
        # print(end_time - start_time)
        stop_start_num += 1
        if stop_start_num >= 15:
            stop_start_num = 0
            control_ser.write("sensorStop\n".encode())
            time.sleep(0.05)
            data_ser.flushInput()
            time.sleep(0.05)
            control_ser.write("sensorStart 0\n".encode())
        # end_time = time.time()
# ------------------------毫米波雷达------------------------ #


def smp_run(run_while, dev_num, s, pc_ip):
  global radar_state
  global command_len
  global danger_stop_t
  global mi48, MI48_bus, MI48_I2C_ADDRESS
  global with_header
  
  # time.sleep(0.5)
  usb_init()
  if control_ser.isOpen():
    pass
  else:
    print("open1")
    control_ser.open()
  if data_ser.isOpen():
    pass
  else:
    print("open2")
    data_ser.open()
  radar_state = 1
  time.sleep(1)
  while_num = 0
  while while_num < command_len - 1:  # 到start
      control_ser.write(command[while_num].encode())
      time.sleep(0.05)
      while_num += 1

  main_while = run_while
  # mi48_num = MI48_bus.read_byte_data(MI48_I2C_ADDRESS, 0xB2)
  # print("mi48:", mi48_num)
  # time.sleep(2)
  # MI48_bus.write_byte_data(MI48_I2C_ADDRESS, 0xB1, 0x02)
  # frame_mode = MI48_bus.read_byte_data(MI48_I2C_ADDRESS, 0xB1)
  # print("0xB1 =", hex(frame_mode))
  # mi48.start(stream=True, with_header=with_header)
  # time.sleep(1)
  while main_while:
      # print('smp.py:', danger_stop_t)
      if danger_stop_t == True:
        print('stopstop')
        danger_stop_t = False
        break
      timer_run(dev_num, s, pc_ip)
      main_while -= 1
      # print('main_while:', main_while)
  # mi48.stop_capture()
  # MI48_bus.write_byte_data(MI48_I2C_ADDRESS, 0xB5, 0x80)
  # time.sleep(1)
  stop_radar_def()
  control_ser.close()
  data_ser.close()
  
  
    
'''
dev_num_file = open('dev_num.txt', 'r')
dev_num = dev_num_file.read()
print(dev_num)

# 创建套接字
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
s.bind(("192.168.0.197", 8080))

pc_ip = "192.168.0.200"
# main_while = 20
if __name__ == '__main__':
    os.system('sudo uhubctl -l 1-1 -a 1')
    time.sleep(1)
    smp_run(20, s, pc_ip)
    os.system('sudo uhubctl -l 1-1 -a 0')
    time.sleep(1)
    os.system('sudo uhubctl -l 1-1 -a 1')
    time.sleep(1)
    smp_run(20, s, pc_ip)
    os.system('sudo uhubctl -l 1-1 -a 0')  '''
    

