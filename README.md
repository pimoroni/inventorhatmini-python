# Inventor HAT Mini

[![Build Status](https://img.shields.io/github/actions/workflow/status/pimoroni/inventorhatmini-python/test.yml?branch=main)](https://github.com/pimoroni/inventorhatmini-python/actions/workflows/test.yml)
[![Coverage Status](https://coveralls.io/repos/github/pimoroni/inventorhatmini-python/badge.svg?branch=master)](https://coveralls.io/github/pimoroni/inventorhatmini-python?branch=master)
[![PyPi Package](https://img.shields.io/pypi/v/inventorhatmini.svg)](https://pypi.python.org/pypi/inventorhatmini)
[![Python Versions](https://img.shields.io/pypi/pyversions/inventorhatmini.svg)](https://pypi.python.org/pypi/inventorhatmini)

A versatile motor, servo and audio driver HAT for Raspberry Pi. Bring your mechanical inventions, creations and contraptions to life!

**Buy it from:** https://shop.pimoroni.com/products/inventor-hat-mini


# Getting the Library

**Stable library only (no examples) from PyPi:**

* Just run `sudo python3 -m pip install inventorhatmini`

This will install the library under the root user. This is required to use Inventor HAT Mini's RGB LEDs, which rely on the [rpi_ws281x package](https://pypi.org/project/rpi-ws281x/).

In some cases you may need to install pip with: `sudo apt install python3-pip`

**Stable library, with latest examples from GitHub:**

* `git clone https://github.com/pimoroni/inventorhatmini-python`
* `cd inventorhatmini-python`
* `./install.sh`

**Latest/development library and examples from GitHub:**

* `git clone https://github.com/pimoroni/inventorhatmini-python`
* `cd inventorhatmini-python`
* `./install.sh --unstable`

# Configuring your Raspberry Pi

## Enable I2C

In order to use the motor, servo, and IO features of Inventor HAT Mini, you need to enable the I2C interface of your Raspberry Pi. This can be done in the terminal by running:

* `sudo raspi-config nonint do_i2c 0`

Alternatively, you can enable the I2C interface by:
* running `sudo raspi-config` and enabling the option under **Interfacing Options**.
* opening the graphical **Raspberry Pi Configuration** application from the **Preferences** menu.

You may need to reboot after enabling I2C for the change to take effect.

## Increase I2C Baudrate

To get the most out of your Inventor HAT Mini it is recommended to run your Raspberry Pi with a higher I2C baudrate than the default 100KHz. This can be done by modifying your Pi's configuration file. To do this run `sudo nano /boot/config.txt` to open a terminal text editor.

Within the editor navigate to the line `dtparam=i2c_arm=on`. If this says `off` or is commented out with a `#`, use this opportunity to change it. Now insert the following line below it, `dtparam=i2c_baudrate=400000` to increase the baudrate.

When you are finished modifying the file, press **CTRL+X** on your keyboard, then when asked to `Save modified buffer?` press **Y**.

Reboot after this for the change to take effect.


### Enable Audio

To use the audio output of your Inventor HAT Mini you will need to modifying your Pi's configuration file. To do this run `sudo nano /boot/config.txt` to open a terminal text editor.

Within the editor navigate to the bottom of the file and include the lines `dtoverlay=hifiberry-dac` and `gpio=25=op,dh`. The first line switches the audio to use the GPIO header of your Pi for audio output, and the second line will cause your Pi to enable the audio output on bootup, by setting pin BCM 25 to high.

Depending on your setup, you may also need to disable other audio outputs of your Pi (for example audio over HDMI). Look through the file for any existing mention to `dtparam=audio=on` and change it to `dtparam=audio=off`. There may the line `dtoverlay=vc4-kms-v3d`. Modify this to be `dtoverlay=vc4-kms-v3d,noaudio`.

When you are finished modifying the file, press **CTRL+X** on your keyboard, then when asked to `Save modified buffer?` press **Y**.

Reboot after this for the change to take effect.

# Examples and Usage

There are many examples to get you started with your Inventor HAT Mini. With the library installed on your Raspberry Pi, these can be found in the `~/Pimoroni/inventorhatmini/examples` directory. Details about what each one does can be found in the [examples readme](/examples/README.md).

To take Inventor HAT Mini further, the full API is described in the [library reference](/REFERENCE.md).


# Removing the Library

To uninstall the library only (keeping all examples):

* Just run `sudo python3 -m pip uninstall inventorhatmini`

Or if you have grabbed the library from Github:

* `cd inventorhatmini-python`
* `./uninstall.sh`
