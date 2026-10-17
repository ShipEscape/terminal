# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2019. All rights reserved.
#
import os
import time
import numpy as np
import logging
import serial

try:
    import matplotlib
    from matplotlib import pyplot as plt
except:
    print("Please install matplotlib to see the thermal image")

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame
from senxor.interfaces import get_serial, USB_Interface

# This will enable mi48 logging debug messages
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

# ==============================
# create an USB interface object
# ==============================
try:
    ser = get_serial()
except IndexError:
    # if on WSL (hack); apply similarly to other cases where
    # device may not be readily found by get_serial
    try:
        ser = serial.Serial('/dev/ttyS4')
    except OSError:
        ser = serial.Serial('/dev/ttyS3')
usb = USB_Interface(ser)
mi48 = MI48([usb, usb])

# print out camera info
camera_info = mi48.get_camera_info()
logger.info('Camera info:')
logger.info(camera_info)

# initiate single frame acquisition
mi48.disable_low_netd()
with_header = True
mi48.start(stream=False, with_header=with_header)

# Read the frame
data, header = mi48.read()
# Log the header and frame stats
if header is not None:
    logger.debug('  '.join([format_header(header),
                            format_framestats(data)]))
else:
    logger.debug(format_framestats(data))

# Visualise data after reshaping the array properly
img = data_to_frame(data, mi48.fpa_shape)
try:
    img = plt.imshow(img.astype(np.float32), cmap='coolwarm',
                     aspect='equal', interpolation=None)
    plt.axis('off')
    plt.show()
except NameError:
    # plt not found/not imported/missing
    pass

# stop capture and quit
mi48.stop()
