# Inventor HAT Mini - Firmware Flashing

## Issue

It was recently reported that a number of [Inventor HAT Mini](https://shop.pimoroni.com/products/inventor-hat-mini) boards were unable to read
the encoders of any connected motors, instead returning zero. After much investigation it was discovered that these boards had been flashed
at our factory with an older firmware version than what was intended, that lacked the encoder subsystem. All other features of the boards are unaffected.

If you encounter this issue with your board, you can either reflash the board yourself using the instructions below, or reach out to Pimoroni support for a replacement at `support@pimoroni.com` (or via the form at https://pimoroni.freshdesk.com/support/tickets/new)

## Instructions

To be able to flash your Inventor HAT Mini, it needs its I2C address to be `0x16`. By default Inventor HAT Mini ships with address `0x17`, so this will need to be changed before flashing, and then changed back afterwards.

### Identify Address

First, identify your board's I2C address. This can be done by running `i2cdetect -y 1` on your Pi. You should see a printout similar to this:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- 17 -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

This confirms the board has address `0x17`.

### Change Address

Next, change the board's address by running the `set_i2c_address.py` script that is in this directory.

```
python3 ./set_i2c_address 0x17 0x16
```

You should see an output similar to this:
```
Waiting for flash writing to start..
flash write ongoing
flash write ongoing
...
flash write ongoing
flash write ongoing
flash write finished
```

:information-source: If you see `flash write ongoing` printed out repeatedly for more than a few seconds, it is likely the address changed worked but the acknowledgement was missed. So cancel the program by pressing CTRL+C on your keyboard.

Run `i2cdetect -y 1` again to confirm the change:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- 16 -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

### Flash the Firmware

With the Inventor HAT Mini having an address of `0x16` it can now be flashed. To do this run the `firmware_update.py` script that is in this directory, giving it the firmware file:

:warning: You will need to `pip install intelhex` for this to work

```
python ./firmware_update.py  "super_io.fw"
```

You should see an output similar to this:

```
Firmware length 30720
confirming I2C:0x16, Chip ID:0x510e, Read ID:0x510e
Entering bootloader...
confirming I2C:0x6f, Chip ID:0xb004, Read ID:0b004
confirmed bootloader version 211
Changed default boot to bootloader
Page 0 written to APROM
Page 1 written to APROM
...
Page 238 written to APROM
Page 239 written to APROM
Code written, verifying...
Verifying page 0
Verifying page 1
...
Verifying page 238
Verifying page 239
Flash verified, setting the newly flashed program as the default boot option and running it...
Changed default boot to main program
Jumped to start main program execution
```

If you get an error message like `Flash verification failed, exiting..`, try re-running the firmware update command again. If it still fails, reach out to Pimoroni support.


### Restoring the Address

With the firmware successfully flashed, the Inventor HAT Mini's address can be restored back to `0x17` (or any other value you wish to use) by following the same steps as in the [Change Address](#change-address) step but with the values reversed:

```
python ./set_i2c_address 0x16 0x17
```

Now your Inventor HAT Mini has been flashed with the latest firmware, letting its encoders be used.
