# coding=utf-8
import smbus
import time
import os
import RPi.GPIO as GPIO

#GPIO.setmode(GPIO.BCM)
#GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def dec_to_bcd(val):
    return ((val // 10) << 4) | (val % 10)


def bcd_to_dec(val):
    return (val >> 4) * 10 + (val & 0x0F)

rtc_i2c_bus = 1
rtc_addr = 0x32  # 0011 0010 1/0(R/W), i2cdetect -y 1

# ¼ÆÊ±¼Ä´æÆ÷µØÖ·
year_add = 0x06
month_add = 0x05
date_add = 0x04
day_add = 0x03    # ÖÜ¼¸
hour_add = 0x02
minutes_add = 0x01
seconds_add = 0x00

# ÄÖÖÓ1¼Ä´æÆ÷µØÖ·
alarms1_date_add = 0x0B
alarms1_hour_add = 0x09
alarms1_minutes_add = 0x08
alarms1_seconds_add = 0x07
alarms1_on_add = 0x0E          # 0x17

# ¿ØÖÆ¼Ä´æÆ÷µØÖ·              # 13
control_CTR1_add = 0x0F       # 0010 0000  0x20 
control_CTR2_add = 0x10       # 0001 0010  0x12

# ×´Ì¬¼Ä´æÆ÷µØÖ·
status_add = 0x0F             # 0Î»ÎªA1F

# À©Õ¹¿ØÖÆ¼Ä´æÆ÷µØÖ·
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

write_pro()
#set_time(1)
write_end()

#os.system('sudo poweroff')

#while True:
#  RTC_state = GPIO.input(26)
#  print(RTC_state)
#  time.sleep(1)


