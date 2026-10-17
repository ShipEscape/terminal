from senxor.mi48 import MI48, DATA_READY, format_header, format_framestats
from senxor.utils import data_to_frame, cv_filter
from senxor.interfaces import SPI_Interface, I2C_Interface
import argparse
import smbus
import time
from spidev import SpiDev
from gpiozero import Pin, DigitalInputDevice, DigitalOutputDevice

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
RPI_GPIO_SPI_CE_MI48 = 1
SPI_XFER_SIZE_BYTES = 160 	# bytes
spi = SPI_Interface(SpiDev(RPI_GPIO_SPI_BUS, RPI_GPIO_SPI_CE_MI48),
                    xfer_size=SPI_XFER_SIZE_BYTES)
spi.device.mode = MI48_SPI_MODE
spi.device.max_speed_hz = MI48_SPI_MAX_SPEED_HZ
spi.device.bits_per_word = 8
spi.device.lsbfirst = False
spi.device.no_cs = True
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
mi48 = MI48([i2c, spi], data_ready=mi48_data_ready,
            reset_handler=MI48_reset(pin=mi48_reset_n))
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

# mi48.start(stream=True, with_header=with_header)

while_num = 10
while while_num:
    mi48.start(stream=True, with_header=with_header)
    # thermal
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
    # print("oke")
    if thermal_data is None:
        # print("oka")
        mi48.stop(stop_timeout=1.0)
        sys.exit(1)
    time.sleep(MI48_SPI_CS_DELAY)
    mi48_spi_cs_n.off()
    
    print(thermal_data)
    mi48.stp()
    while_num -= 1

