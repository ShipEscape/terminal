import pickle
import socket
import numpy as np
import PIL
import cv2

from senxor.utils import data_to_frame, cv_filter

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)     # 创建套接字
s.bind(("0.0.0.0", 9090))   # 监听所有可用IP地址
print('run')

shape = (80, 62)    # 80 * 62 图像
# image_list = []
# while_num = shape[1]
# while while_num:
#     image_list.append([])
#     while_num -= 1

while True:
    data, IP = s.recvfrom(100000)
    if IP[0] == '192.168.0.197':

        data = pickle.loads(data)           # data[0]:热成像，data[1]:雷达信息
        print(data)
        img_list = data_to_frame(data[0], shape)

        # 0 - 255
        img_8u = cv2.normalize(img_list.astype('uint8'), None, 255, 0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        img8u = cv_filter(img_8u, parameters={'blur_ks': 3}, use_median=False, use_bilat=True, use_nlm=False)
        cvcol = cv2.applyColorMap(img8u , cv2.COLORMAP_JET)
        cvresize = cv2.resize(cvcol, (320, 248), interpolation=cv2.INTER_CUBIC)
        cv2.imshow('img', cvresize)
        cv2.waitKey(1)
        # break
        # print("data:", data, "IP:", IP)




