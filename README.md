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

**Stable library (only) from PyPi:**

* Just run `sudo python3 -m pip install inventorhatmini`

This will install the library under the root user, which is required to use Inventor HAT Mini's RGB LEDs, which rely on the [rpi_ws281x package](https://pypi.org/project/rpi-ws281x/)

In some cases you may need to install pip with: `sudo apt install python3-pip`

**Stable library, with latest examples from GitHub:**

* `git clone https://github.com/pimoroni/inventorhatmini-python`
* `cd inventorhatmini-python`
* `./install.sh`

**Latest/development library and examples from GitHub:**

* `git clone https://github.com/pimoroni/inventorhatmini-python`
* `cd inventorhatmini-python`
* `./install.sh --unstable`

# Examples and Usage

There are many examples to get you started with your Inventor HAT Mini. With the library installed on your Raspberry Pi, these can be found in the `~/Pimoroni/inventorhatmini/examples` directory. Details about what each one does can be found in the [examples readme](/examples/README.md).

To take Inventor HAT Mini further, the full API is described in the [library reference](/REFERENCE.md)

# Configuring Audio

To use the audio output of InventorHATMini you will need to modify the `/boot/config.txt` file of your Raspberry Pi. To do this run `sudo nano /boot/config.txt` to open a terminal text editor.

Within the editor navigate to the bottom of the file and include the lines `dtoverlay=hifiberry-dac` and `gpio=25=op,dh`. The first line switches the audio to use the GPIO header of the PI for audio output, and the second line will cause the Pi to enable the audio output on bootup, by setting pin BCM 25 to high.

Note that you may also need to disable other audio outputs of the Pi (for example audio over HDMI). To do this navigate up the file to find any existing mention to `dtparam=audio` and change it to `dtparam=audio=off`.

There may the line `dtoverlay=vc4-kms-v3d`. Modify this to be `dtoverlay=vc4-kms-v3d,noaudio`.

When you are finished modifying the file, press CTRL+X on your keyboard, then `y` when asked to `Save modified buffer?`.

# Configuring I2C

To get the most out of your Inventor HAT Mini it is recommended to run your Pi with a higher I2C baudrate than the default 100KHz. This can be done by modifying the `/boot/config.txt/` file of your Raspberry Pi. To do this run `sudo nano /boot/config.txt` to open a terminal text editor.

Within the editor navigate to the line `dtparam=i2c_arm=on`. If this says `off` or is commented out with a `#`, use this opportunity to change it. Now insert the following line below it, `dtparam=i2c_baudrate=400000` to increase the baudrate.

When you are finished modifying the file, press CTRL+X on your keyboard, then Y when asked to `Save modified buffer?`.

# Uninstalling

To uninstall the library:

* Just run `python3 -m pip uninstall inventorhatmini`

Or if you have grabbed the library from Github:

* `cd inventorhatmini-python`
* `./uninstall.sh`
