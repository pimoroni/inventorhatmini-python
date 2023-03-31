# Inventor HAT Mini

[![Build Status](https://img.shields.io/github/actions/workflow/status/pimoroni/inventorhatmini-python/test.yml?branch=main)](https://github.com/pimoroni/inventorhatmini-python/actions/workflows/test.yml)
[![Coverage Status](https://coveralls.io/repos/github/pimoroni/inventorhatmini-python/badge.svg?branch=master)](https://coveralls.io/github/pimoroni/inventorhatmini-python?branch=master)
[![PyPi Package](https://img.shields.io/pypi/v/inventorhatmini.svg)](https://pypi.python.org/pypi/inventorhatmini)
[![Python Versions](https://img.shields.io/pypi/pyversions/inventorhatmini.svg)](https://pypi.python.org/pypi/inventorhatmini)

# Pre-requisites

You must enable:

* i2c: `sudo raspi-config nonint do_i2c 0`

You can optionally run `sudo raspi-config` or the graphical Raspberry Pi Configuration UI to enable interfaces.

# Installing

Stable library from PyPi:

* Just run `python3 -m pip install inventorhatmini`

In some cases you may need to use `sudo` or install pip with: `sudo apt install python3-pip`

Latest/development library from GitHub:

* `git clone https://github.com/pimoroni/inventorhatmini-python`
* `cd inventorhatmini-python`
* `sudo ./install.sh`

# Examples and Usage

There are many examples to get you started with your Inventor HAT Mini. With the library installed on your Raspberry Pi, these can be found in the `~/Pimoroni/inventorhatmini/examples` directory. Details about what each one does can be found in the [examples readme](/examples/README.md).

To take Inventor HAT Mini further, the full API is described in the [library readme](/library/inventorhatmini/README.md)
