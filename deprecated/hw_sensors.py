#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hw_sensors.py —— 树莓派传感器硬件访问层（真实采集）

把 smp.py 里散落在模块级、与业务逻辑混在一起的硬件访问代码，整理成可单独
调用的类。覆盖本项目实际使用的四类外设：

    热成像 MI48        smp.py:122-241（初始化）, smp.py:526-550（读帧）, smp.py:238-241（休眠）
    毫米波雷达         smp.py:246-376（配置/启停）, smp.py:618-808（读帧+解析+周期重启）
    电池电量计         smp.py:99-118（寄存器与换算）
    烟雾输入 GPIO      smp.py:247-250（初始化）, smp.py:502-509（去抖）

依赖（与原项目一致，均需在树莓派上安装）：
    smbus / smbus2 / RPi.GPIO / gpiozero / spidev / pyserial / senxor
    说明：pyproject.toml 只声明了 smbus 与 pymysql，但 smp.py 实际还 import
    smbus2 / RPi.GPIO / gpiozero / spidev / serial / senxor / numpy，
    这些在树莓派上已随系统或手工安装；本文件沿用同一批库。

⚠ 本文件在开发机上无法运行，也【未在真机上验证】：
   本机缺 senxor / RPi.GPIO / spidev / pyserial / smbus2，
   且没有 /dev/i2c-*、/dev/radar_uart_* 设备节点。
   所有寄存器地址、引脚号、波特率、时序、命令表均逐行取自 smp.py，
   未作任何猜测；凡是不确定的地方都在注释里写明"原代码未说明"。
   上机前建议先用 `python3 hw_sensors.py` 做一次单次快照自检。

与 smp.py 的差异（均为结构性改动，不改变硬件行为，逐条列出便于核对）：

  1. 无模块级副作用。smp.py 在 import 阶段就打开串口、初始化相机、向下发雷达
     配置、并调用 argparse.parse_args()（会吞掉宿主程序的 sys.argv）。
     本文件全部封装进 start()/close()，可安全 import。
  2. 不在库内终止宿主进程。smp.py 在读帧失败时 sys.exit(1)（smp.py:548）、
     相机初始化超时时 sudo reboot（smp.py:204-205）。本文件改为抛异常，
     是否重启由调用方决定。
  3. 串口按长度读满。smp.py 用单次 ser.read(n)；pyserial 在 timeout 内可能
     返回不足 n 字节，此时 smp.py 会因 data[12] 越界而 IndexError。
     本文件改为循环读满 _read_exact()，并设总超时上限。
  4. TLV type=1 增加边界判断 i+16 <= end。smp.py 的条件是 i < end，
     当 TLV_len 不是 16 的整数倍时会多读一个残缺点。
  5. 非 type=1 的 TLV 直接跳到 end。smp.py 是逐字节 +1 走到 end（等价但慢，
     例如 TLV 5 距离-多普勒热图会白跑数千次循环）。
  6. 帧浮点解析用小端 '<f'。见下方 _parse_tlv 内的字节序推导说明。
  7. 雷达命令表内置为默认值（与 smp.py:254-290 逐字一致，仅去掉行尾的 \r，
     发送时统一补 \r），可用 profile 参数或 profile_path 覆盖。
  8. argparse 的参数改为构造参数（fps / offset_corr / enable_filter ...）。
9. 新增 SensorHub，把四类传感器组合成统一接口，便于直接喂给 min.py 的
     --source hw（接口：battery() / smoke() / radar() / thermal() / close()）。
  10. SmokeInput 支持 gpiozero / RPi.GPIO 两种后端，默认自动选择。部分新板子上
      RPi.GPIO 会报 "Cannot determine SOC peripheral base address"，而 gpiozero
      仍可用，因此优先 gpiozero。
  11. 新增 --diag：只打印板型 / pin factory / senxor 实际位置等诊断信息，
      不接触任何外设，用于快速定位环境问题。
"""

from __future__ import annotations

import os
import struct
import subprocess
import time

# ============================================================ 常量
# 全部取自 smp.py，未作修改。括号内为原文件行号。

# ---- 热成像 MI48 ----
MI48_I2C_CHANNEL = 1                 # smp.py:139
MI48_I2C_ADDRESS = 0x40              # smp.py:140
MI48_SPI_MODE = 0b00                 # smp.py:148
MI48_SPI_BITS_PER_WORD = 8           # smp.py:149
MI48_SPI_LSBFIRST = False            # smp.py:150
MI48_SPI_MAX_SPEED_HZ = 31200000     # smp.py:154
MI48_SPI_CS_DELAY = 0.0001           # smp.py:155
MI48_SPI_BUS = 0                     # smp.py:157
MI48_SPI_CE = 0                      # smp.py:158
MI48_SPI_XFER_SIZE = 160             # smp.py:159
MI48_CS_PIN = "BCM7"                 # smp.py:171
MI48_DATA_READY_PIN = "BCM24"        # smp.py:178
MI48_RESET_PIN = "BCM23"             # smp.py:179
MI48_RESET_ASSERT_S = 0.000035       # smp.py:187
MI48_RESET_DEASSERT_S = 0.050        # smp.py:188
MI48_FRAME_PRE_SLEEP = 0.1           # smp.py:527
MI48_STATUS_POLL_S = 0.01            # smp.py:534
MI48_SLEEP_REG = 0xB5                # smp.py:241
MI48_SLEEP_VAL = 0x80                # smp.py:241
MI48_DEFAULT_FPS = 7                 # smp.py:130
MI48_FILTER_MIN_FW = 2               # smp.py:212

# ---- 电池电量计 ----
BATTERY_I2C_CHANNEL = 1              # smp.py:118
BATTERY_ADDRESS = 0x36               # smp.py:101 与 smp.py:110
BATTERY_VOLTAGE_REG = 2              # smp.py:102
BATTERY_CAPACITY_REG = 4             # smp.py:111
BATTERY_VOLTAGE_SCALE = 1.25 / 1000 / 16   # smp.py:104
BATTERY_CAPACITY_SCALE = 1.0 / 256         # smp.py:115

# ---- 烟雾输入 ----
SMOKE_INPUT_PIN = 17                 # smp.py:248
SMOKE_OUTPUT_PIN = 26                # smp.py:249
SMOKE_DEBOUNCE_FRAMES = 30           # smp.py:505
SMOKE_ALARM_LEVEL = 0                # smp.py:503（低电平报警）

# ---- 毫米波雷达 ----
RADAR_CONTROL_PORT = "/dev/radar_uart_0"   # smp.py:297
RADAR_DATA_PORT = "/dev/radar_uart_1"      # smp.py:298
RADAR_CONTROL_BAUD = 115200                # smp.py:301
RADAR_DATA_BAUD = 921600                   # smp.py:302
RADAR_SERIAL_TIMEOUT = 0.5                 # smp.py:301-302
RADAR_HEADER_LEN = 32                      # smp.py:299
RADAR_CMD_DELAY = 0.05                     # smp.py:348
RADAR_RESTART_EVERY = 15                   # smp.py:801
RADAR_RESTART_GAP_S = 0.05                 # smp.py:804, 806
RADAR_POWER_CYCLE_S = 1                    # smp.py:639, 642
RADAR_RETRY_S = 2                          # smp.py:321, 336
RADAR_MAX_TLV_TYPE = 9                     # smp.py:703
RADAR_READ_DEADLINE_S = 2.0                # 本文件新增（差异 3）

# ---- USB 端口供电（uhubctl） ----
USB_POWER_PORTS = ("2", "4")         # smp.py:34-35, 141-142, 179-184, 197-202, 225-226, 637-641

# ---- 雷达配置命令表（smp.py:254-290 逐字抄录，仅去掉行尾 \r） ----
RADAR_PROFILE_DEFAULT = (
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 7 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "profileCfg 0 60 359 7 57.14 0 0 70 1 256 5209 0 0 158",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 2",
    "chirpCfg 2 2 0 0 0 0 0 4",
    "frameCfg 0 2 16 0 100 1 0",
    "lowPower 0 0",
    "guiMonitor -1 1 1 0 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 15 1",
    "cfarCfg -1 1 0 4 2 3 1 15 1",
    "multiObjBeamForming -1 1 0.5",
    "clutterRemoval -1 1",
    "calibDcRangeSig -1 0 -5 8 256",
    "extendedMaxVelocity -1 0",
    "lvdsStreamCfg -1 0 0 0",
    "compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0",
    "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
    "CQRxSatMonitor 0 3 5 121 0",
    "CQSigImgMonitor 0 127 4",
    "analogMonitor 0 0",
    "aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0 8.92",
    "cfarFovCfg -1 1 -1 1.00",
    "calibData 0 0 0",
    "sensorStart",
    "sensorStop",
)


# ============================================================ 异常

class HardwareError(RuntimeError):
    """硬件访问失败的基类。"""


class ThermalReadError(HardwareError):
    """MI48 返回空帧。smp.py 在此处 sys.exit(1)（smp.py:545-548），本文件抛异常。"""


class RadarError(HardwareError):
    """雷达串口无法打开或数据流不可用。"""


# ============================================================ 通用工具

def _pin_factory_name():
    """返回 gpiozero 当前使用的 pin factory（诊断用）。

    这决定了 gpiozero 实际走 lgpio 还是 RPi.GPIO。在 RPi.GPIO 无法映射 SoC
    外设基址的板子上，如果 factory 是 RPiGPIOFactory，那 gpiozero 也会失败。

    注意：gpiozero 的 pin factory 是惰性解析的——进程里还没创建过任何 gpiozero
    设备时 Device.pin_factory 就是 None，直接读会得到 "None" 这种没意义的结果。
    所以先 ensure_pin_factory() 强制解析（该版本没有这个方法就跳过）。
    """
    try:
        from gpiozero import Device
    except Exception as exc:
        return "gpiozero 不可用 (%s)" % exc
    try:
        ensure = getattr(Device, "ensure_pin_factory", None)
        if callable(ensure):
            ensure()
    except Exception as exc:
        return "解析失败(%s)" % exc
    factory = getattr(Device, "pin_factory", None)
    if factory is None:
        return "未解析（本进程还没创建过 gpiozero 设备）"
    return repr(factory)


def _open_i2c_bus(channel):
    """打开 I2C 总线。优先 smbus（与 smp.py:141 一致），缺失时退回 smbus2。

    实测该 Pi 上 smbus 未安装而 smbus2 已安装。senxor 的 I2C_Interface 只用
    read_byte_data / write_byte_data / read_i2c_block_data 这类标准方法，
    smbus2 提供同名 API，可直接替换，省掉一个系统包依赖。
    """
    try:
        import smbus
        return smbus.SMBus(channel)
    except ImportError:
        import smbus2
        return smbus2.SMBus(channel)


def _run(argv):
    """执行外部命令。smp.py 用 os.system('sudo ...')，此处改为不经 shell 的等价调用。

    stdin=DEVNULL：sudo 未配置免密时立即失败，而不是交互式索要密码导致挂起。
    capture_output：uhubctl 会打印端口状态，若继承 stdout 会打乱终端画面。
    """
    try:
        return subprocess.run(argv, check=False, stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE).returncode
    except FileNotFoundError:
        return 127


def usb_port_power(on, ports=USB_POWER_PORTS, use_sudo=True):
    """uhubctl 控制 USB 端口供电，用于雷达/热像仪重新枚举。

    对应 smp.py 中反复出现的 'sudo uhubctl -l <port> -a 1|0'。
    on=True 上电，on=False 断电。
    """
    action = "1" if on else "0"
    results = []
    for port in ports:
        argv = ["uhubctl", "-l", str(port), "-a", action]
        if use_sudo:
            argv.insert(0, "sudo")
        results.append(_run(argv))
    return results


def _flush_input(ser):
    """清空输入缓冲。pyserial 3.x 用 reset_input_buffer，旧版是 flushInput。

    smp.py 用的是 flushInput（smp.py:375, 805），此处做兼容以适配两种版本。
    """
    for name in ("reset_input_buffer", "flushInput"):
        fn = getattr(ser, name, None)
        if fn is not None:
            fn()
            return


def _flush_output(ser):
    """清空输出缓冲。对应 smp.py:376 的 flushOutput()。"""
    for name in ("reset_output_buffer", "flushOutput"):
        fn = getattr(ser, name, None)
        if fn is not None:
            fn()
            return


# ============================================================ 热成像 MI48

class _Mi48Reset:
    """MI48 复位时序，对应 smp.py:185-199。

    senxor 初始化时会回调本对象。注意 mi48_reset_n 是 active_high=False 且
    initial_value=True（smp.py:179-180），所以 .on() 实际是把 RESET_N 拉低
    （断言复位），.off() 拉高（释放）。
    """

    def __init__(self, pin, assert_seconds=MI48_RESET_ASSERT_S,
                 deassert_seconds=MI48_RESET_DEASSERT_S):
        self.pin = pin
        self.assert_seconds = assert_seconds
        self.deassert_seconds = deassert_seconds

    def __call__(self):
        self.pin.on()
        time.sleep(self.assert_seconds)
        self.pin.off()
        time.sleep(self.deassert_seconds)


class Mi48Thermal:
    """MI48 热成像仪。

    初始化顺序严格对应 smp.py:139-235；读帧对应 smp.py:526-550。

    用法：
        cam = Mi48Thermal(fps=7).start()
        frame, header = cam.read_frame()      # frame 支持 .tolist()
        cam.sleep()                           # 低功耗（smp.py:238-241）
        cam.close()
    """

    def __init__(self, fps=MI48_DEFAULT_FPS, offset_corr=0.0,
                 enable_filter=True, use_data_ready_pin=True,
                 cs_delay=MI48_SPI_CS_DELAY, stop_timeout=1.0,
                 reboot_on_timeout=False, verbose=False):
        self.fps = fps
        self.offset_corr = offset_corr
        self.enable_filter = enable_filter
        self.use_data_ready_pin = use_data_ready_pin
        self.cs_delay = cs_delay
        self.stop_timeout = stop_timeout
        self.reboot_on_timeout = reboot_on_timeout
        self.verbose = verbose

        self.mi48 = None
        self.camera_info = None
        self.fw_version = None

        self._i2c_bus = None
        self._spi = None
        self._cs_n = None
        self._reset_n = None
        self._data_ready = None

    @staticmethod
    def _import_senxor():
        """导入 senxor 组件；失败时把该包的真实位置与内容一并报出来。

        导入路径与 smp.py:123-125 保持一致：
            from senxor.mi48 import MI48, DATA_READY, ...
            from senxor.interfaces import SPI_Interface, I2C_Interface
        本函数不改这些路径，只把失败原因说清楚——因为 senxor 在 PyPI 上查不到
        （/pypi/senxor/json 返回 404），它更可能是随 SDK 分发的本地目录，
        不同安装方式的模块布局可能不同，不能靠猜。
        """
        try:
            from senxor.interfaces import I2C_Interface, SPI_Interface
            from senxor.mi48 import MI48
            return MI48, I2C_Interface, SPI_Interface
        except ImportError as exc:
            detail = []
            try:
                import senxor
                base = getattr(senxor, "__file__", None)
                detail.append("senxor.__file__ = %s" % base)
                detail.append("senxor.__path__ = %s"
                              % list(getattr(senxor, "__path__", [])))
                if base:
                    detail.append("同目录文件 = %s"
                                  % sorted(os.listdir(os.path.dirname(base))))
            except Exception as exc2:
                detail.append("连 import senxor 都失败: %s" % exc2)
            raise HardwareError(
                "senxor 导入失败: %s\n  %s" % (exc, "\n  ".join(detail)))

    def start(self):
        from gpiozero import DigitalInputDevice, DigitalOutputDevice
        from spidev import SpiDev

        MI48, I2C_Interface, SPI_Interface = self._import_senxor()

        # I2C：smp.py:141-142 用的是 smbus.SMBus(1)；
        # smbus 缺失时 _open_i2c_bus() 会退回 smbus2（API 兼容）。
        self._i2c_bus = _open_i2c_bus(MI48_I2C_CHANNEL)
        i2c = I2C_Interface(self._i2c_bus, MI48_I2C_ADDRESS)

        # SPI：smp.py:160-166
        spi = SPI_Interface(SpiDev(MI48_SPI_BUS, MI48_SPI_CE),
                            xfer_size=MI48_SPI_XFER_SIZE)
        spi.device.mode = MI48_SPI_MODE
        spi.device.max_speed_hz = MI48_SPI_MAX_SPEED_HZ
        spi.device.bits_per_word = MI48_SPI_BITS_PER_WORD
        spi.device.lsbfirst = MI48_SPI_LSBFIRST
        self._spi = spi

        # 片选 / 数据就绪 / 复位：smp.py:171-180
        self._cs_n = DigitalOutputDevice(MI48_CS_PIN, active_high=False,
                                         initial_value=False)
        self._reset_n = DigitalOutputDevice(MI48_RESET_PIN, active_high=False,
                                            initial_value=True)
        if self.use_data_ready_pin:
            self._data_ready = DigitalInputDevice(MI48_DATA_READY_PIN,
                                                  pull_up=False)

        # 构造 MI48：smp.py:201-205
        try:
            self.mi48 = MI48([i2c, spi], data_ready=self._data_ready,
                             reset_handler=_Mi48Reset(self._reset_n))
        except TimeoutError:
            # smp.py:204-205 在这里直接 sudo reboot；库内不应擅自重启整机。
            if self.reboot_on_timeout:
                _run(["sudo", "reboot"])
            raise HardwareError(
                "MI48 初始化超时。原 smp.py 会在此处 sudo reboot；"
                "如需保留该行为请传 reboot_on_timeout=True")

        # 相机信息与帧率：smp.py:208-209
        self.camera_info = self.mi48.get_camera_info()
        self.mi48.set_fps(self.fps)

        # 固件 >= 2 才启用滤波：smp.py:212-224
        self.fw_version = self.mi48.fw_version
        if int(self.fw_version[0]) >= MI48_FILTER_MIN_FW:
            if self.enable_filter:
                self.mi48.enable_filter(f1=True, f2=True, f3=False)
            self.mi48.set_offset_corr(self.offset_corr)

        # 连续采集：smp.py:227-235
        self.mi48.disable_low_netd()
        self.mi48.start(stream=True, with_header=True)
        return self

    def read_frame(self):
        """读一帧。返回 (thermal_data, header)，thermal_data 支持 .tolist()。

        对应 smp.py:526-550。差异：无论成功与否都先释放片选再判断空帧，
        避免 smp.py:545-548 直接 sys.exit 时把 CS 留在断言状态。
        """
        from senxor.mi48 import DATA_READY

        time.sleep(MI48_FRAME_PRE_SLEEP)              # smp.py:527
        if hasattr(self.mi48, "data_ready") and self.mi48.data_ready is not None:
            self.mi48.data_ready.wait_for_active()    # smp.py:528-529
        else:
            while True:                               # smp.py:530-535
                time.sleep(MI48_STATUS_POLL_S)
                if self.mi48.get_status() & DATA_READY:
                    break

        self._cs_n.on()                               # smp.py:538（断言片选，拉低）
        try:
            time.sleep(self.cs_delay)                 # smp.py:540
            thermal_data, header = self.mi48.read()   # smp.py:542
        finally:
            time.sleep(self.cs_delay)                 # smp.py:549
            self._cs_n.off()                          # smp.py:550

        if thermal_data is None:                      # smp.py:545-548
            raise ThermalReadError("MI48 返回空帧（原 smp.py 在此处 sys.exit(1)）")
        return thermal_data, header

    def sleep(self):
        """停流并让 MI48 进入低功耗。对应 smp.py:238-241。"""
        self.mi48.stop_capture()
        self._i2c_bus.write_byte_data(MI48_I2C_ADDRESS,
                                      MI48_SLEEP_REG, MI48_SLEEP_VAL)

    def close(self):
        if self.mi48 is not None:
            try:
                self.mi48.stop_capture()
            except Exception:
                pass
        # gpiozero 设备需要显式释放，否则 BCM7/23/24 会一直被占用。
        for dev in (self._cs_n, self._reset_n, self._data_ready):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
        self._cs_n = self._reset_n = self._data_ready = None


# ============================================================ 毫米波雷达

class RadarMmwave:
    """TI MMWDEMO 毫米波雷达，经两路串口访问。

    控制口 /dev/radar_uart_0 @115200：下发配置、启停。
    数据口 /dev/radar_uart_1 @921600：TLV 帧流，type=1 为检测点 [x,y,z,doppler]。

    用法：
        radar = RadarMmwave().start()      # 打开串口并下发配置
        points = radar.read_points()       # [(x, y, z, doppler), ...]
        radar.close()
    """

    def __init__(self, control_port=RADAR_CONTROL_PORT,
                 data_port=RADAR_DATA_PORT,
                 control_baud=RADAR_CONTROL_BAUD,
                 data_baud=RADAR_DATA_BAUD,
                 serial_timeout=RADAR_SERIAL_TIMEOUT,
                 profile=None,
                 profile_path=None,
                 restart_every=RADAR_RESTART_EVERY,
                 read_deadline=RADAR_READ_DEADLINE_S,
                 chmod_ports=True,
                 power_cycle_on_retry=True,
                 retries=None,
                 verbose=False):
        self.control_port = control_port
        self.data_port = data_port
        self.control_baud = control_baud
        self.data_baud = data_baud
        self.serial_timeout = serial_timeout
        self.restart_every = restart_every
        self.read_deadline = read_deadline
        self.chmod_ports = chmod_ports
        self.power_cycle_on_retry = power_cycle_on_retry
        self.retries = retries            # None = 无限重试（与 smp.py:307/322 一致）
        self.verbose = verbose

        if profile_path:
            with open(profile_path, "r", encoding="utf-8") as fh:
                profile = [ln.rstrip("\r\n") for ln in fh if ln.strip()]
        self.profile = tuple(profile) if profile else RADAR_PROFILE_DEFAULT

        self.control_ser = None
        self.data_ser = None
        self.radar_state = 0
        self._frame_count = 0
        self._serial = None

    # ---- 串口 ----

    def _open_serial(self):
        """打开两路串口，必要时重试并做 USB 重新枚举。

        对应 smp.py:304-337 的 usb_init()。原实现在 while True 里无限重试，
        本文件默认保持无限（retries=None）；传 retries=N 可改为有限次。
        """
        import serial

        self._serial = serial
        attempt = 0
        while True:
            attempt += 1
            try:
                self.control_ser = serial.Serial(
                    self.control_port, self.control_baud, timeout=self.serial_timeout)
                if not self.control_ser.isOpen():
                    raise RadarError("%s 打开后 isOpen() 为假" % self.control_port)
                if self.chmod_ports:
                    # smp.py:313 用 sudo chmod 777；此处保留但可关闭。
                    _run(["sudo", "chmod", "777", self.control_port])
                break
            except Exception as exc:
                if self.retries is not None and attempt > self.retries:
                    raise RadarError("无法打开控制口 %s: %s" % (self.control_port, exc))
                if self.power_cycle_on_retry:
                    usb_port_power(True)
                time.sleep(RADAR_RETRY_S)

        attempt = 0
        while True:
            attempt += 1
            try:
                self.data_ser = serial.Serial(
                    self.data_port, self.data_baud, timeout=self.serial_timeout)
                if not self.data_ser.isOpen():
                    raise RadarError("%s 打开后 isOpen() 为假" % self.data_port)
                if self.chmod_ports:
                    _run(["sudo", "chmod", "777", self.data_port])
                break
            except Exception as exc:
                if self.retries is not None and attempt > self.retries:
                    raise RadarError("无法打开数据口 %s: %s" % (self.data_port, exc))
                if self.power_cycle_on_retry:
                    usb_port_power(True)
                time.sleep(RADAR_RETRY_S)

    def _close_serial(self):
        for ser in (self.control_ser, self.data_ser):
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        self.control_ser = self.data_ser = None

    # ---- 配置与启停 ----

    def _profile_commands(self):
        """smp.py:346 的循环条件是 while_num < command_len - 1，即最后一条
        （sensorStop）不下发。此处按语义显式剔除末尾的 sensorStop。"""
        cmds = list(self.profile)
        if cmds and cmds[-1].strip().lower() == "sensorstop":
            cmds = cmds[:-1]
        return cmds

    def _apply_profile(self):
        for cmd in self._profile_commands():
            self.control_ser.write((cmd + "\r").encode())
            time.sleep(RADAR_CMD_DELAY)

    def start_radar(self):
        """对应 smp.py:352-364 start_radar_def()。"""
        if self.radar_state == 1:
            return False
        self.control_ser.write(b"sensorStart 0\r")
        self.radar_state = 1
        return True

    def stop_radar(self):
        """对应 smp.py:367-376 stop_radar_def()。"""
        if self.radar_state == 0:
            return False
        self.control_ser.write(b"sensorStop\r")
        self.radar_state = 0
        _flush_input(self.data_ser)
        _flush_output(self.data_ser)
        return True

    def start(self):
        self._open_serial()
        self._apply_profile()
        self.radar_state = 1
        return self

    # ---- 读帧 ----

    def _read_exact(self, size):
        """读满 size 字节（差异 3）。

        smp.py 用单次 read()，可能短读并在后续 data[12] 越界崩溃；
        这里循环补齐，并以 read_deadline 作为总等待上限。
        """
        if size <= 0 or self.data_ser is None:
            return b""
        buf = bytearray()
        deadline = time.monotonic() + self.read_deadline
        while len(buf) < size:
            chunk = self.data_ser.read(size - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
            if time.monotonic() > deadline:
                break
        return bytes(buf)

    def _recover(self):
        """数据流中断后的恢复流程，对应 smp.py:634-664。"""
        self._close_serial()
        usb_port_power(False)
        time.sleep(RADAR_POWER_CYCLE_S)
        usb_port_power(True)
        time.sleep(RADAR_POWER_CYCLE_S)
        self._open_serial()
        self.radar_state = 1
        time.sleep(RADAR_POWER_CYCLE_S)
        self._apply_profile()

    def _maybe_restart(self):
        """每 restart_every 帧重启一次雷达，对应 smp.py:800-807。"""
        if not self.restart_every:
            return
        self._frame_count += 1
        if self._frame_count < self.restart_every:
            return
        self._frame_count = 0
        self.control_ser.write(b"sensorStop\r")
        time.sleep(RADAR_RESTART_GAP_S)
        _flush_input(self.data_ser)
        time.sleep(RADAR_RESTART_GAP_S)
        self.control_ser.write(b"sensorStart 0\r")

    def read_points(self):
        """读一帧并返回检测点 [(x, y, z, doppler), ...]。

        对应 smp.py:632-677（取帧）与 smp.py:686-755（TLV 解析）。
        """
        if self.radar_state != 1:
            return []
        header = self._read_exact(RADAR_HEADER_LEN)
        if len(header) < RADAR_HEADER_LEN:
            # smp.py:634 只在 len(data)==0 时恢复；短读在其代码里会直接崩溃。
            # 这里把短读也视为流不可用并触发恢复（差异 3/4 的连带处理）。
            self._recover()
            return []

        # 长度字段为小端 uint32，再减去 32 字节帧头（smp.py:671-672）
        data_len = struct.unpack_from("<I", header, 12)[0] - RADAR_HEADER_LEN
        if data_len <= 0:
            return []
        data = self._read_exact(data_len)
        points = self._parse_tlv(data)
        self._maybe_restart()
        return points

    @staticmethod
    def _parse_tlv(data):
        """解析 MMWDEMO TLV 块，只提取 type=1 的检测点。

        对应 smp.py:686-755。

        字节序推导（重要，勿凭直觉改）：
            smp.py:722-726 先把 4 个字节拼成【小端】整数：
                now_point_x = b0 + b1*256 + b2*65536 + b3*16777216
            再 struct.pack('!I', ...) 把它写成【大端】字节序列，即 [b3,b2,b1,b0]，
            最后 struct.unpack('!f', ...) 按大端读成 float。
            把 [b3,b2,b1,b0] 当大端读，等价于把 [b0,b1,b2,b3] 当小端读。
            验证：字节 00 00 80 3F → 小端整数 0x3F800000 → pack('!I') 得
            3F 80 00 00 → unpack('!f') = 1.0；而 '<f' 读 00 00 80 3F 同样 = 1.0。
            所以此处直接用 '<f'；若写成 '!f' 会得到约 4.6e-41 的错误数值。
        """
        points = []
        i = 0
        n = len(data)
        while i < n:
            if n - i <= 8:                      # smp.py:690-692：余下为填充
                break
            tlv_type = struct.unpack_from("<I", data, i)[0]
            tlv_len = struct.unpack_from("<I", data, i + 4)[0]
            if tlv_type > RADAR_MAX_TLV_TYPE or tlv_len >= n:
                break                           # smp.py:703-705
            i += 8                              # smp.py:709
            end = i + tlv_len                   # smp.py:711

            if tlv_type == 1:
                # 每个检测点 16 字节：x, y, z, doppler
                while i + 16 <= end:            # smp.py:713 的条件是 i < end（见差异 4）
                    x, y, z, doppler = struct.unpack_from("<ffff", data, i)
                    points.append((x, y, z, doppler))
                    i += 16
            # type 2-9（距离剖面/噪声/各类热图/统计）本项目未使用：
            # smp.py:757-780 是逐字节 +1 走到 end，此处直接跳转（差异 5）。
            i = end
        return points

    def close(self):
        if self.control_ser is not None and self.radar_state == 1:
            try:
                self.control_ser.write(b"sensorStop\r")
                self.radar_state = 0
            except Exception:
                pass
        self._close_serial()


# ============================================================ 电池

class BatteryGauge:
    """I2C 电量计（地址 0x36）。对应 smp.py:99-118。

    smp.py 用 smbus2.SMBus(1) 读电池；I2C 总线 1 与 MI48 相同但各自持有
    独立的总线对象，此处保持同样的做法。
    """

    def __init__(self, channel=BATTERY_I2C_CHANNEL, address=BATTERY_ADDRESS):
        self.channel = channel
        self.address = address
        self._bus = None

    def start(self):
        try:
            import smbus2 as busmod       # smp.py:24 用的是 smbus2
        except ImportError:
            import smbus as busmod        # 退化为 smbus（API 兼容）
        self._bus = busmod.SMBus(self.channel)
        return self

    def voltage(self):
        """输出电压 V。对应 smp.py:99-105。"""
        raw = self._bus.read_word_data(self.address, BATTERY_VOLTAGE_REG)
        swapped = struct.unpack("<H", struct.pack(">H", raw))[0]
        return swapped * BATTERY_VOLTAGE_SCALE

    def capacity(self):
        """剩余电量 %，已钳到 [0, 100]。对应 smp.py:108-116 与 smp.py:514-517。"""
        raw = self._bus.read_word_data(self.address, BATTERY_CAPACITY_REG)
        swapped = struct.unpack("<H", struct.pack(">H", raw))[0]
        return max(0.0, min(100.0, swapped * BATTERY_CAPACITY_SCALE))

    def close(self):
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass


# ============================================================ 烟雾输入

class SmokeInput:
    """烟雾报警输入 GPIO。

    对应 smp.py:247-250（初始化）、smp.py:502-509（读取与去抖）。
    GPIO17 上拉输入，低电平报警；GPIO26 输出恒定 HIGH —— 原代码未注释其用途，
    此处按原样保持 HIGH，不做任何假设。
    """

    def __init__(self, pin=SMOKE_INPUT_PIN, output_pin=SMOKE_OUTPUT_PIN,
                 debounce=SMOKE_DEBOUNCE_FRAMES, backend="auto",
                 cleanup_on_close=False):
        self.pin = pin
        self.output_pin = output_pin
        self.debounce = debounce
        self.backend = backend      # auto | gpiozero | rpi
        self.cleanup_on_close = cleanup_on_close
        self.alarm = False          # 去抖后的结论，对应 smp.py 的 alarm_danger
        self.backend_used = None
        self._low_frames = 0
        self._in = None
        self._out = None
        self._gpio = None

    def start(self):
        """优先 gpiozero，失败再退回 RPi.GPIO。

        smp.py:247-250 用的是 RPi.GPIO。但在部分板子上 RPi.GPIO 会直接报
        "Cannot determine SOC peripheral base address"（它无法映射 SoC 外设基址），
        此时 gpiozero（走 lgpio）通常仍可用——MI48 的 BCM7/23/24 本来就是
        用 gpiozero 驱动的。因此这里做后端自动选择，并把两侧的失败原因都报出来。
        """
        errors = []
        if self.backend in ("auto", "gpiozero"):
            try:
                self._start_gpiozero()
                self.backend_used = "gpiozero"
                return self
            except Exception as exc:
                errors.append("gpiozero: %s" % exc)
                self._in = self._out = None
        if self.backend in ("auto", "rpi"):
            try:
                self._start_rpi()
                self.backend_used = "RPi.GPIO"
                return self
            except Exception as exc:
                errors.append("RPi.GPIO: %s" % exc)
        raise HardwareError(
            "烟雾输入 GPIO 初始化失败 -> %s | pin factory = %s"
            % (" ; ".join(errors), _pin_factory_name()))

    def _start_gpiozero(self):
        from gpiozero import DigitalInputDevice, DigitalOutputDevice
        # 等价于 smp.py:248 的 GPIO.IN + PUD_UP。
        #
        # 必须显式给 active_state=True：gpiozero 的 .value 是"逻辑有效态"而不是
        # 原始电平。pull_up=True 时 gpiozero 默认 active_state=False，即"引脚为低
        # 才算 value=1"，这与 smp.py 里 GPIO.input() 的原始电平语义正好相反——
        # 无报警时反而读到 0，被去抖逻辑当成"连续低电平"而误报。
        # 实测对照：pinctrl get 17 -> hi，gpioget gpiochip0 17 -> 1，
        # 而改前 read() 返回 0，三者矛盾，即为该 bug。
        self._in = DigitalInputDevice(self.pin, pull_up=True, active_state=True)
        # 等价于 smp.py:249-250 的 GPIO.OUT + HIGH
        self._out = DigitalOutputDevice(self.output_pin, initial_value=True)

    def _start_rpi(self):
        import RPi.GPIO as GPIO
        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.output_pin, GPIO.OUT)
        GPIO.output(self.output_pin, GPIO.HIGH)

    def read(self):
        """返回当前原始电平（0=报警）。同时按 smp.py:503-509 累计去抖状态。"""
        if self._in is not None:
            level = int(self._in.value)
        elif self._gpio is not None:
            level = self._gpio.input(self.pin)
        else:
            raise HardwareError("烟雾输入尚未初始化")
        if level == SMOKE_ALARM_LEVEL:
            self._low_frames += 1
            if self._low_frames >= self.debounce:
                self.alarm = True
        else:
            self._low_frames = 0
        return level

    def close(self):
        for dev in (self._in, self._out):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
        self._in = self._out = None
        # smp.py 从不清理 GPIO；GPIO.cleanup() 会复位全部 BCM 引脚，
        # 可能干扰 gpiozero 管理的 MI48 引脚，故默认不做。
        if self.cleanup_on_close and self._gpio is not None:
            self._gpio.cleanup()
        self._gpio = None


# ============================================================ 组合

class SensorHub:
    """把四类传感器组合成统一接口，可直接作为 min.py 的数据源。

    接口与 min.py 的 open_source() 期望一致：
        battery() -> (voltage, capacity) | (None, None)
        smoke()   -> int | None          原始电平，0=报警
        radar()   -> [(x,y,z,doppler), ...] | None
        thermal() -> [[float, ...], ...] | None     已 .tolist()
        close()
    """

    def __init__(self, thermal=True, radar=True, battery=True, smoke=True,
                 usb_power_on_start=True, fps=MI48_DEFAULT_FPS,
                 radar_profile_path=None, radar_restart_every=RADAR_RESTART_EVERY,
                 radar_retries=None, verbose=False):
        self.enable_thermal = thermal
        self.enable_radar = radar
        self.enable_battery = battery
        self.enable_smoke = smoke
        self.usb_power_on_start = usb_power_on_start
        self.fps = fps
        self.radar_profile_path = radar_profile_path
        self.radar_restart_every = radar_restart_every
        self.radar_retries = radar_retries
        self.verbose = verbose

        self.thermal_cam = None
        self.radar_dev = None
        self.battery_dev = None
        self.smoke_dev = None
        self.notes = []          # 各子系统的失败原因，便于在终端上显示

    def start(self):
        if self.usb_power_on_start:
            # smp.py:34-35 / run.py:197-199：先给 USB 端口上电再访问外设
            usb_port_power(True)
            time.sleep(RADAR_POWER_CYCLE_S)

        if self.enable_thermal:
            try:
                self.thermal_cam = Mi48Thermal(fps=self.fps,
                                               verbose=self.verbose).start()
            except Exception as exc:
                self.thermal_cam = None
                self.notes.append("MI48 初始化失败: %s" % exc)

        if self.enable_battery:
            try:
                self.battery_dev = BatteryGauge().start()
            except Exception as exc:
                self.battery_dev = None
                self.notes.append("电量计不可用: %s" % exc)

        if self.enable_smoke:
            try:
                self.smoke_dev = SmokeInput().start()
            except Exception as exc:
                self.smoke_dev = None
                self.notes.append("烟雾 GPIO 不可用: %s" % exc)

        if self.enable_radar:
            try:
                self.radar_dev = RadarMmwave(
                    profile_path=self.radar_profile_path,
                    restart_every=self.radar_restart_every,
                    retries=self.radar_retries,
                    verbose=self.verbose).start()
            except Exception as exc:
                self.radar_dev = None
                self.notes.append("雷达不可用: %s" % exc)

        return self

    def battery(self):
        if self.battery_dev is None:
            return None, None
        try:
            return self.battery_dev.voltage(), self.battery_dev.capacity()
        except Exception as exc:
            self.notes.append("电量读取失败: %s" % exc)
            self.battery_dev = None
            return None, None

    def smoke(self):
        if self.smoke_dev is None:
            return None
        try:
            return self.smoke_dev.read()
        except Exception as exc:
            self.notes.append("烟雾读取失败: %s" % exc)
            self.smoke_dev = None
            return None

    @property
    def smoke_alarm(self):
        """去抖后的报警结论（30 帧连续低电平），对应 smp.py:503-509。"""
        return bool(self.smoke_dev is not None and self.smoke_dev.alarm)

    def radar(self):
        if self.radar_dev is None:
            return None
        try:
            return self.radar_dev.read_points()
        except Exception as exc:
            self.notes.append("雷达读取失败: %s" % exc)
            return []

    def thermal(self):
        """返回二维温度列表。smp.py:558 也是 .tolist()，此处保持一致，
        同时避免把 numpy 数组直接交给显示层（numpy 数组的真值判断会报错）。"""
        if self.thermal_cam is None:
            return None
        try:
            frame, _header = self.thermal_cam.read_frame()
        except ThermalReadError:
            raise
        except Exception as exc:
            self.notes.append("热成像读取失败: %s" % exc)
            return None
        return frame.tolist()

    def close(self):
        if self.radar_dev is not None:
            self.radar_dev.close()
        if self.thermal_cam is not None:
            try:
                self.thermal_cam.sleep()
            except Exception:
                pass
            self.thermal_cam.close()
        if self.smoke_dev is not None:
            self.smoke_dev.close()
        if self.battery_dev is not None:
            self.battery_dev.close()


# ============================================================ 环境诊断

def diagnose():
    """打印环境诊断，不接触任何外设。

    传感器初始化失败时先跑 `python3 hw_sensors.py --diag`：它会给出板型、
    gpiozero 的 pin factory、senxor 包的真实位置与内容、各设备节点，
    足以判断是环境问题还是代码问题——不需要靠猜。
    """
    import glob
    import platform

    lines = []
    lines.append("python      : %s" % platform.python_version())
    lines.append("platform    : %s %s" % (platform.system(), platform.release()))
    lines.append("machine     : %s" % platform.machine())

    model = None
    for path in ("/proc/device-tree/model",
                 "/sys/firmware/devicetree/base/model"):
        try:
            with open(path, "rb") as fh:
                model = fh.read().rstrip(b"\x00").decode("utf-8", "replace")
            break
        except OSError:
            continue
    lines.append("board model : %s" % (model or "读不到（可能不是树莓派）"))

    try:
        import gpiozero
        ver = getattr(gpiozero, "__version__", None)
        if ver is None:
            # 该版本不导出 __version__，退回用包元数据查
            try:
                from importlib.metadata import version as _pkg_version
                ver = _pkg_version("gpiozero")
            except Exception:
                ver = "?（未导出 __version__，元数据也查不到）"
        lines.append("gpiozero    : %s" % ver)
    except Exception as exc:
        lines.append("gpiozero    : 不可用 (%s)" % exc)
    lines.append("pin factory : %s" % _pin_factory_name())

    try:
        import RPi.GPIO as GPIO
        lines.append("RPi.GPIO    : 导入成功 VERSION=%s"
                     % getattr(GPIO, "VERSION", "?"))
        try:
            lines.append("RPi.GPIO 板型: %s" % (GPIO.RPI_INFO,))
        except Exception as exc:
            lines.append("RPi.GPIO 板型: 读取失败 (%s)"
                         "  <-- 通常就是 Cannot determine SOC ... 那个错误" % exc)
    except Exception as exc:
        lines.append("RPi.GPIO    : 导入失败 (%s)" % exc)

    try:
        import senxor
        base = getattr(senxor, "__file__", None)
        lines.append("senxor      : %s" % base)
        lines.append("senxor 路径 : %s" % list(getattr(senxor, "__path__", [])))
        if base:
            lines.append("senxor 内容 : %s"
                         % sorted(os.listdir(os.path.dirname(base))))
    except Exception as exc:
        lines.append("senxor      : 导入失败 (%s)" % exc)
    for mod in ("senxor.mi48", "senxor.utils", "senxor.interfaces"):
        try:
            __import__(mod)
            lines.append("  %-20s OK" % mod)
        except Exception as exc:
            lines.append("  %-20s 失败: %s" % (mod, exc))

    for mod in ("smbus", "smbus2", "spidev", "serial", "numpy"):
        try:
            __import__(mod)
            lines.append("%-11s : OK" % mod)
        except Exception as exc:
            lines.append("%-11s : 失败 (%s)" % (mod, exc))

    for pattern in ("/dev/i2c-*", "/dev/spidev*", "/dev/radar_uart_*"):
        lines.append("%-11s : %s" % (pattern, sorted(glob.glob(pattern)) or "无"))
    return lines


# ============================================================ 自检入口

def main(argv=None):
    """单次快照自检：在树莓派上运行 `python3 hw_sensors.py` 检查各传感器。

    只采集一轮，不做聚类、不画图，用于确认接线与库是否就绪。
    """
    import argparse
    import json

    parser = argparse.ArgumentParser(
        description="树莓派传感器单次快照自检（真实硬件）")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出")
    parser.add_argument("--fps", type=float, default=MI48_DEFAULT_FPS)
    parser.add_argument("--radar-profile", default=None,
                        help="雷达配置命令文件（每行一条），默认用内置命令表")
    parser.add_argument("--no-usb-power", action="store_true",
                        help="不执行 uhubctl 上电")
    parser.add_argument("--skip-thermal", action="store_true")
    parser.add_argument("--skip-radar", action="store_true")
    parser.add_argument("--skip-battery", action="store_true")
    parser.add_argument("--skip-smoke", action="store_true")
    parser.add_argument("--diag", action="store_true",
                        help="只打印环境诊断，不接触任何外设")
    args = parser.parse_args(argv)

    if args.diag:
        for line in diagnose():
            print(line)
        return 0

    hub = SensorHub(thermal=not args.skip_thermal,
                    radar=not args.skip_radar,
                    battery=not args.skip_battery,
                    smoke=not args.skip_smoke,
                    usb_power_on_start=not args.no_usb_power,
                    fps=args.fps,
                    radar_profile_path=args.radar_profile)
    hub.start()
    try:
        snapshot = {"notes": hub.notes}

        voltage, capacity = hub.battery()
        snapshot["battery"] = {"voltage_v": voltage, "capacity_pct": capacity}

        level = hub.smoke()
        snapshot["smoke"] = {"level": level,
                             "alarm": hub.smoke_alarm,
                             "note": "0=报警（低电平），30 帧连续低电平判定为报警"}

        points = hub.radar()
        snapshot["radar"] = {"point_count": len(points) if points is not None else None,
                             "points": points[:10] if points else None,
                             "note": "仅列前 10 个点，格式 [x, y, z, doppler]"}

        frame = hub.thermal()
        if frame is None:
            snapshot["thermal"] = None
        else:
            flat = [v for row in frame for v in row]
            snapshot["thermal"] = {
                "shape": [len(frame), len(frame[0]) if frame else 0],
                "min_c": min(flat) if flat else None,
                "max_c": max(flat) if flat else None,
                "mean_c": (sum(flat) / len(flat)) if flat else None,
                "note": "只给统计量，完整帧请用 min.py --source hw 查看",
            }

        if args.json:
            print(json.dumps(snapshot, ensure_ascii=False, indent=2))
        else:
            for key, value in snapshot.items():
                print("%-9s %s" % (key, value))
    finally:
        hub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
