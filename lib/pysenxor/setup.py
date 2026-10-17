#!/usr/bin/env python3
#
# Copyright (C) 2019-2021  Stanislav Markov, Meridian Innovation Ltd, Hong Kong

from setuptools import setup
import sys
import os

if sys.version_info < (3, 0, 0, 'final', 0):
    raise SystemExit('Python 3 is required!')

_install_requires = [
    # core
    'pyserial',
    'smbus; platform_system=="Linux"',
    'spidev; platform_system=="Linux"',
    "numpy",
    "crcmod",
    #
    # to see image from single capture
    "matplotlib",
    # to enable matplotlib colormaps in thermogram
    "cmapy",
    #
    # to see the stream
# better link this manualy to existing installation
#        "cv2"
]
try:
    # add gpiozero to raspberry pi system; but below may fail on other arm host
    if os.uname()[4].startswith("arm"):
        _install_requires.append('gpiozero')
except AttributeError:
    # windows does not have os.uname(); one can confirm we're on windows by
    # os.name == 'nt', but we do not really care
    pass

# Read version file
with open(os.path.join('.', 'senxor', 'VERSION')) as fd:
    version = fd.read().strip()

setup(
    name="pysenxor",
    version=version,
    description="Python SDK for Meridian Innovation's SenXor.",
    author="Stanislav Markov, Meridian Innovation Ltd.",
    platforms="platform independent",
    package_dir={"": "."},
    packages=[
        "senxor",
    ],
    scripts=[],
    classifiers=[
        "Programming Language :: Python",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering",
    ],
    long_description="""

""",
    install_requires = _install_requires,
)
