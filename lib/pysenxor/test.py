# coding=utf-8
import os
import time

add_min_file_path = "add_minutes.txt"

if os.path.isfile(add_min_file_path) and os.path.getsize(add_min_file_path) != 0:
  add_min_file = open(add_min_file_path, 'r')
else:
  add_min_file = open(add_min_file_path, 'w+')
  add_min_file.write(str(10))
  add_min_file.close()
  time.sleep(1)
  add_min_file = open(add_min_file_path, 'r')
add_minutes = int(add_min_file.read())
# add_minutes = 10    # 关机时间
print('False:', add_minutes, type(add_minutes))
