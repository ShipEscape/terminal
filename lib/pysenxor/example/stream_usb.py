# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2020. All rights reserved.
#
import sys
import os
import signal
import time
import logging
import serial

try:
    import cv2 as cv
except:
    print("Please install OpenCV (or link existing installation)"
          " to see the thermal image")
    exit(1)

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame
from senxor.interfaces import get_serial, USB_Interface

# This will enable mi48 logging debug messages
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

def cv_display(img, title='', resize=(320, 248),
               colormap=cv.COLORMAP_JET, interpolation=cv.INTER_CUBIC):
    """
    Display image using OpenCV-controled window.

    Data is a 2D numpy array of type uint8,

    Image is coloured and resized
    """
    cvcol = cv.applyColorMap(img, colormap)
    cvresize =  cv.resize(cvcol, resize, interpolation=interpolation)
    cv.imshow(title, cvresize)

# Make the a global variable and use it as an instance of the mi48.
# This allows it to be used directly in a signal_handler.
global mi48

# define a signal handler to ensure clean closure upon CTRL+C
# or kill from terminal
def signal_handler(sig, frame):
    """Ensure clean exit in case of SIGINT or SIGTERM"""
    logger.info("Exiting due to SIGINT or SIGTERM")
    mi48.stop()
    cv.destroyAllWindows()
    logger.info("Done.")
    sys.exit(0)

# Define the signals that should be handled to ensure clean exit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

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
logger.debug('Connected USB interface:')
logger.debug(usb)

# Make an instance of the MI48, attaching USB for 
# both control and data interface.
mi48 = MI48([usb, usb])

# print out camera info
camera_info = mi48.get_camera_info()
logger.info('Camera info:')
logger.info(camera_info)

# set desired FPS
if len(sys.argv) == 2:
    STREAM_FPS = int(sys.argv[1])
else:
    STREAM_FPS = 9
mi48.set_fps(STREAM_FPS)

# see if filtering is available in MI48 and set it up
if int(mi48.fw_version[0]) >= 2:
    # Enable filtering with default strengths
    mi48.enable_filter(f1=True, f2=True)

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
mi48.start(stream=True, with_header=with_header)

# change this to false if not interested in the image
GUI = True

while True:
    data, header = mi48.read()
    if data is None:
        logger.critical('NONE data received instead of GFRA')
        mi48.stop()
        sys.exit(1)

    img = data_to_frame(data, mi48.fpa_shape)
    #
    if header is not None:
        logger.debug('  '.join([format_header(header),
                                format_framestats(data)]))
    else:
        logger.debug(format_framestats(data))

    img8u = cv.normalize(img.astype('uint8'), None, 255, 0,
                         norm_type=cv.NORM_MINMAX,
                         dtype=cv.CV_8U)
    if GUI:
        cv_display(img8u)
        key = cv.waitKey(1)  # & 0xFF
        if key == ord("q"):
            break
#    time.sleep(1)

# stop capture and quit
mi48.stop()
cv.destroyAllWindows()
