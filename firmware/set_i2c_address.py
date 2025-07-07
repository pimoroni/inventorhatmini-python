# EXAMPLE:
# pi@raspberrypi:~ $ python set_i2c_address.py 0x0A 0x22
# Waiting for flash writing to start..
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write finished
# pi@raspberrypi:~ $ 
# pi@raspberrypi:~ $ 
# pi@raspberrypi:~ $ i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- 22 -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- --                         
# pi@raspberrypi:~ $ python set_i2c_address.py 0x22 0x23
# Waiting for flash writing to start..
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write ongoing
# flash write finished
# pi@raspberrypi:~ $ i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- 23 -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- --  

import sys
import time

import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg

GPIO.setmode(GPIO.BCM)

CHIP_ID          = 0xBA11
VERSION          = 1 


# Registers specific to the trackball
REG_LED_RED      = 0x00
REG_LED_GREEN    = 0x01
REG_LED_BLUE     = 0x02
REG_LED_WHITE    = 0x03
REG_LEFT         = 0x04
REG_RIGHT        = 0x05
REG_UP           = 0x06
REG_DOWN         = 0x07
REG_SWITCH       = 0x08
BIT_SWITCH_STATE = 7


#Registers that should be shared between breakouts
REG_USER_FLASH   = 0xD0 # 32 bytes, 0xD0 - 0xEF
REG_FLASH_PAGE   = 0xF0
REG_INT          = 0xF9
BIT_INT_TRIGD    = 0
BIT_INT_OUT_EN   = 1
REG_CHIP_ID_L    = 0xFA
REG_CHIP_ID_H    = 0xFB
REG_VERSION      = 0xFC
REG_I2C_ADDRESS  = 0xFD
REG_CTRL         = 0xFE
BIT_SLEEP        = 0
BIT_RESET        = 1
BIT_READ_FLASH   = 2
BIT_WRITE_FLASH  = 3
BIT_ALLOW_I2C_ADDR_CHANGE = 4


I2C_ADDRESS = int(sys.argv[1], 0)
NEW_I2C_ADDRESS = int(sys.argv[2], 0)

i2c_dev = SMBus(1)

def write(bytearr):
    # print("Writing", bytearr)
    msg = i2c_msg.write(I2C_ADDRESS, bytearr)
    i2c_dev.i2c_rdwr(msg)

write( [REG_INT, (1<<BIT_INT_OUT_EN) ] ) # Enable interrupt output on INT pin

write( [REG_CTRL, (1<<BIT_ALLOW_I2C_ADDR_CHANGE) ] )

write([REG_I2C_ADDRESS, NEW_I2C_ADDRESS])

GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

# Wait for flash write to start
print("Waiting for flash writing to start..")
time.sleep(0.001)

# Wait for flash write to finish
while not GPIO.input(4):

    time.sleep(0.001)
    print("flash write ongoing")

print("flash write finished")
