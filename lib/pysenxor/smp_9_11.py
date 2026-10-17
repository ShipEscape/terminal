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

import multiprocessing as mp

import argparse
import smbus
from spidev import SpiDev
from gpiozero import Pin, DigitalInputDevice, DigitalOutputDevice

Diagonal_max = 0.85
Point_number = 2

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
# -------------------------------------------------- #

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
command = ["flushCfg\r",
                "dfeDataOutputMode 1\r",
                "channelCfg 15 7 0\r",  # first start
                "adcCfg 2 1\r",  # first start
                "adcbufCfg -1 0 1 1 1\r",
                "profileCfg 0 60 359 7 57.14 0 0 70 1 256 5209 0 0 158\r",
                "chirpCfg 0 0 0 0 0 0 0 1\r",
                "chirpCfg 1 1 0 0 0 0 0 2\r",
                "chirpCfg 2 2 0 0 0 0 0 4\r",
                "frameCfg 0 2 16 0 100 1 0\r",
                "lowPower 0 0\r",  # first start
                "guiMonitor -1 1 1 0 0 0 1\r",
                "cfarCfg -1 0 2 8 4 3 0 15 1\r",
                "cfarCfg -1 1 0 4 2 3 1 15 1\r",
                "multiObjBeamForming -1 1 0.5\r",
                "clutterRemoval -1 0\r",
                "calibDcRangeSig -1 0 -5 8 256\r",
                "extendedMaxVelocity -1 0\r",
                "lvdsStreamCfg -1 0 0 0\r",
                "compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0\r",
                "measureRangeBiasAndRxChanPhase 0 1.5 0.2\r",
                "CQRxSatMonitor 0 3 5 121 0\r",
                "CQSigImgMonitor 0 127 4\r",
                "analogMonitor 0 0\r",
                "aoaFovCfg -1 -90 90 -90 90\r",
                "cfarFovCfg -1 0 0 8.92\r",
                "cfarFovCfg -1 1 -1 1.00\r",
                "calibData 0 0 0\r",
                "sensorStart\r",
                "sensorStop\r"]
command_len = len(command)  # 命令列表的长度

stop_start_num = 0
person_number = 0
radar_state = 0  # 雷达状态：1为启动，0为停止
while_num = 0
control_com = "/dev/radar_uart_0"
data_com = "/dev/radar_uart_1"
data_header_len = 32

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
        control_ser.write("sensorStart 0\r".encode())
        print("雷达开始")
        radar_state = 1


def stop_radar_def():  # 停止
    global radar_state
    if radar_state == 0:
        print("当前雷达已为停止状态")
    elif radar_state == 1:
        control_ser.write("sensorStop\r".encode())
        print("雷达停止")
        radar_state = 0
        data_ser.flushInput()
        data_ser.flushOutput()


def dbscan1(my_list, diagonal_max, point_number):
    global cu_list

    flag_list = [0] * len(my_list)
    temp_flag_list = []
    temp_list = []
    temp_num = 0
    now_num = 0

    for now_point in my_list:
        temp_num = 0    # temp_list的位置信息
        # if flag_list[now_num] == 0:
        for all_point in my_list:
            diagonal = math.sqrt((all_point[0] - now_point[0]) **2 + (all_point[1] - now_point[1]) **2 +(all_point[2] - now_point[2])**2)
            # print(now_point, all_point, diagonal)
            if diagonal <= diagonal_max:
                temp_list.append(all_point)
                temp_flag_list.append(temp_num)
            temp_num += 1
        if len(temp_list) >= point_number:
            if flag_list[now_num] != 0:
                while_num = len(temp_flag_list)
                while while_num:
                    if flag_list[temp_flag_list[while_num-1]] == 0:     # 判断当前点的圈内的点是否归为其他簇
                        cu_list[flag_list[now_num]-1].append(temp_list[while_num-1])    # 当前算法没有合并两个簇的操作
                        flag_list[temp_flag_list[while_num-1]] = flag_list[now_num]
                    while_num -= 1
                while_num = len(temp_flag_list)
                while while_num:
                    if flag_list[temp_flag_list[while_num-1]] == 0:
                        flag_list[temp_flag_list[while_num-1]] = len(cu_list)

                    while_num -= 1
            else:
                cu_list.append(copy.deepcopy(temp_list))
                while_num = len(temp_flag_list)
                while while_num:
                    if flag_list[temp_flag_list[while_num-1]] == 0:
                        flag_list[temp_flag_list[while_num-1]] = len(cu_list)

                    while_num -= 1
            # print(flag_list)
        temp_list.clear()
        temp_flag_list.clear()
        now_num += 1

def plt_run(dev_num, s, pc_ip):
    global now_data_point_list, cu_list
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
    
    print("now_data_point_list:", now_data_point_list)
    ##for abc in now_data_point_list:
    ##  json_x_list.append(int(scale(abc[0], (-4, 4), (0, 8))))
    ##  json_y_list.append(int(scale(abc[1], (0, 5), (0, 4))))
    ##  json_z_list.append(int(scale(abc[2], (-2, 2), (0, 3))))
    
    # global lock
    if len(now_data_point_list) <= 8:
      Point_number = 2
    else:
      Point_number = 3
    dbscan1(now_data_point_list, Diagonal_max, Point_number)
        
    person_number = len(cu_list)
    for cu in cu_list:
      all_x = 0
      all_y = 0
      all_z = 0
      for cu_point in cu:
        all_x += cu_point[0]
        all_y += cu_point[1]
        all_z += cu_point[2]
      json_x_list.append(scale(all_x/len(cu)*10, (-4, 4), (0, 40)))
      json_y_list.append(scale(all_y/len(cu), (0, 5), (0, 40)))
      json_z_list.append(scale(all_z/len(cu), (-2, 2), (0, 3)))

    Alarm_state = GPIO.input(17)
    if Alarm_state == 0:
      alarm_low += 1
      if alarm_low >= 30:
        alarm_danger = True
        print("烟雾报警，进入紧急模式")
    else:
      alarm_low = 0
    
    time.sleep(0.6)
    '''bus_voltage = ina219.getBusVoltage_V()             # voltage on V- (load side)
    current = -ina219.getCurrent_mA()                   # current in mA
    dl_p = (bus_voltage - 3)/1.2*100
    if dl_p > 100:
      dl_p=100
    if dl_p < 0:
      dl_p=0
    
    if(bus_voltage < 3.15) and (current < 50):
      dl_low += 1
      if dl_low >= 30:
        print("过放")
    else:
      dl_low = 0'''

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
    dict_json_0 = {'device_code': dev_num, 'title': 'TD1', 'dl': str('%.2f' % 0), 'Alarm': str(Alarm_state)}    # 'dl': str('%.2f' % dl_p)
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
    # print("\n\n\n\n Debug2 \n\n\n\n")


def timer_run(dev_num, s, pc_ip):  # 主要逻辑函数
    global now_data_point_list
    global while_num
    global now_data_point_num
    global stop_start_num
    global radar_state
    global command_len
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
                    data[15] * (16 * 16) * (16 * 16) * (16 * 16)) - 32

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
            if TLV_type > 9 or TLV_len >= len(data):
                # 余下为填充
                break
            # print("TLV type:%d" % TLV_type, "   ", "TLV len:%d" % TLV_len)

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
                if TLV_type == 1:
                    # MMWDEMO输出消息检测点
                    now_data_point_list.append([])
                    now_point_x = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    float_now_point_x = struct.unpack('!f', struct.pack('!I', now_point_x))[0]
                    while_num += 4

                    now_point_y = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    float_now_point_y = struct.unpack('!f', struct.pack('!I', now_point_y))[0]
                    while_num += 4

                    now_point_z = data[while_num] + \
                                  data[while_num + 1] * (16 * 16) + \
                                  data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                  data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    float_now_point_z = struct.unpack('!f', struct.pack('!I', now_point_z))[0]
                    while_num += 4

                    now_point_velocity = data[while_num] + \
                                         data[while_num + 1] * (16 * 16) + \
                                         data[while_num + 2] * (16 * 16) * (16 * 16) + \
                                         data[while_num + 3] * (16 * 16) * (16 * 16) * (16 * 16)
                    float_now_point_velocity = struct.unpack('!f', struct.pack('!I', now_point_velocity))[0]
                    while_num += 4

                    # 将坐标加入列表并清空临时列表
                    now_data_point_list[now_data_point_num].append(float_now_point_x)
                    now_data_point_list[now_data_point_num].append(float_now_point_y)
                    now_data_point_list[now_data_point_num].append(float_now_point_z)
                    now_data_point_list[now_data_point_num].append(float_now_point_velocity)
                    now_data_point_num += 1

                elif TLV_type == 2:
                    # MMWDEMO输出范围轮廓
                    while_num += 1
                elif TLV_type == 3:
                    # MMWDEMO输出噪声分布
                    while_num += 1
                elif TLV_type == 4:
                    # MMWDEMO输出方位角静态热图
                    while_num += 1
                elif TLV_type == 5:
                    # MMWDEMO输出距离多普勒热图
                    while_num += 1
                elif TLV_type == 6:
                    # MMWDEMO输出消息统计
                    while_num += 1
                elif TLV_type == 7:
                    # MMWDEMO输出检测点侧信息
                    while_num += 1
                elif TLV_type == 8:
                    # MMWDEMO输出方位角/仰角静态热图
                    while_num += 1
                elif TLV_type == 9:
                    # MMWDEMO输出温度统计
                    while_num += 1

        # 该段数据分析结束
        # print("end")
        
        # 使用
        print("flag_while:", flag_while)
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
            control_ser.write("sensorStop\r".encode())
            time.sleep(0.05)
            data_ser.flushInput()
            time.sleep(0.05)
            control_ser.write("sensorStart 0\r".encode())
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
    

