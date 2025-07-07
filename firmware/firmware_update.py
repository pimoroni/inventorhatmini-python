
# To make the Pi not drop I2C clockspeed to 250kHz from 400kHz, one can say:
# echo "performance" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Usage: python firmware_update.py filename.hex

import binascii
import sys
import time

from smbus2 import SMBus, i2c_msg

# Small nuvoton:
bootloader_id       = 0xB001
bootloader_version  = 172
bootloader_i2c_addr = 0x6F
flash_size          = 16384-2048

I2C_WRITE_MAX_SIZE  = 32

REG_APROM_FLASH    = 0x10
REG_FLASH_PAGE     = 0xF0
REG_CHIP_ID_L      = 0xfa
REG_CHIP_ID_H      = 0xfb
REG_VERSION        = 0xfc
REG_DEBUG          = 0xF8
REG_BOOTLOADER_CMD = 0xF3

REG_CTRL           = 0xfe
MASK_CTRL_FREAD    = 0x4
MASK_CTRL_FWRITE   = 0x8

PAGE_SIZE          = 128

i2c_dev = SMBus(1)

def i2c_write_bytes(address, start_reg, valuelist):
    assert( len(valuelist) <= I2C_WRITE_MAX_SIZE )
    msg_w = i2c_msg.write(address, [start_reg] + valuelist)
    i2c_dev.i2c_rdwr(msg_w)


def i2c_read_bytes(address, start_reg, length):
    assert( length <= I2C_WRITE_MAX_SIZE )
    msg_w = i2c_msg.write(address, [start_reg])
    i2c_dev.i2c_rdwr(msg_w)
    msg_r = i2c_msg.read(address, length)
    i2c_dev.i2c_rdwr(msg_r)
    return list(msg_r)


def i2c_read8(address, reg):
    """Read a single (8bit) register from the device."""
    msg_w = i2c_msg.write(address, [reg])
    i2c_dev.i2c_rdwr(msg_w)
    msg_r = i2c_msg.read(address, 1)
    i2c_dev.i2c_rdwr(msg_r)
    return list(msg_r)[0]

def i2c_write8(address, reg, value):
    """Write a single (8bit) register to the device."""
    msg_w = i2c_msg.write(address, [reg, value])
    i2c_dev.i2c_rdwr(msg_w)

def enter_bootloader(address):
    print("Entering bootloader...")
    i2c_write8(address, REG_CTRL, 0xA0)
    i2c_write8(address, REG_DEBUG, 0xA3)
    time.sleep(0.1)

def set_default_boot(target):
    i2c_write8(bootloader_i2c_addr, REG_BOOTLOADER_CMD, target)
    time.sleep(0.02)

def set_default_boot_bootloader():
    set_default_boot(2)
    print("Changed default boot to bootloader")

def set_default_boot_main_program():
    set_default_boot(4)
    print("Changed default boot to main program")


def get_chip_id(address):
    """Get the IOE chip ID."""
    return (i2c_read8(address, REG_CHIP_ID_H) << 8) | i2c_read8(address, REG_CHIP_ID_L)

def confirm_id(address, id):
    chip_id = get_chip_id(address)
    print(f"confirming I2C:{hex(address)}, Chip ID:{hex(id)}, Read ID:{hex(chip_id)}")
    assert( chip_id == id )

def confirm_bootloader_version(version):
    assert( i2c_read8(bootloader_i2c_addr, REG_VERSION) == version )
    print("confirmed bootloader version", version)

def set_flash_page(page):
    i2c_write8(bootloader_i2c_addr, REG_FLASH_PAGE, page)
    # print "Page set to", page

def write_page_to_aprom():
    i2c_write8(bootloader_i2c_addr, REG_CTRL, MASK_CTRL_FWRITE)
    while True:
        time.sleep(0.01)
        try:
            i2c_read8(bootloader_i2c_addr, 0x00)
            break
        except Exception:
            print("waiting for page write to finish...")

def read_page_from_aprom():
    i2c_write8(bootloader_i2c_addr, REG_CTRL, MASK_CTRL_FREAD)
    time.sleep(0.001)

def jump_to_main_program():
    i2c_write8(bootloader_i2c_addr, REG_BOOTLOADER_CMD, 1)
    print("Jumped to start main program execution")


def firmware_update(bin_data, i2c_address, chip_id):
    assert( len(bin_data) % PAGE_SIZE == 0)
    assert( PAGE_SIZE <= len(bin_data) <= flash_size)

    try:
        i2c_read8(bootloader_i2c_addr, 0x00)
    except:
        # We're not in bootloader yet, let's enter first
        confirm_id(i2c_address, chip_id)
        enter_bootloader(i2c_address)

    # Ok, we should be in bootloader now

    # Ensure bootloader is running with 24MHz clock
    assert( i2c_read8(bootloader_i2c_addr, 0xF7) & 0x10 )

    confirm_id(bootloader_i2c_addr, bootloader_id)
    confirm_bootloader_version(bootloader_version)

    set_default_boot_bootloader()

    # Flash the program!
    for page in range(len(bin_data)//PAGE_SIZE):
        i=0
        # for i in range(PAGE_SIZE):
        while i<PAGE_SIZE:
            start_offset = page*PAGE_SIZE + i
            next_offset   = page*PAGE_SIZE + i + I2C_WRITE_MAX_SIZE
            # print("Writing bytes from", start_offset, "to", next_offset-1)
            i2c_write_bytes( bootloader_i2c_addr, REG_APROM_FLASH+i, bin_data[ start_offset : next_offset ] )
            i += I2C_WRITE_MAX_SIZE
            # i2c_write8(bootloader_i2c_addr, REG_APROM_FLASH+i, bin_data[page*PAGE_SIZE + i])
        set_flash_page(page)
        write_page_to_aprom()
        print(f"Page {page} written to APROM")

    print("Code written, verifying...")
    for page in range(len(bin_data)//PAGE_SIZE):
        print("Verifying page", page)
        set_flash_page(page)
        read_page_from_aprom()
        # for i in range(PAGE_SIZE):
        i=0
        while i< PAGE_SIZE:
            read_length  = min(I2C_WRITE_MAX_SIZE, PAGE_SIZE-i)
            read_data = i2c_read_bytes(bootloader_i2c_addr, REG_APROM_FLASH+i, read_length)

            start_offset = page*PAGE_SIZE + i
            wr_data = bin_data[ start_offset : start_offset+read_length ]

            # print("read_data, wr_data", read_data)
            # print (wr_data)

            if (read_data != wr_data):
                print("Flash verification failed, exiting..")
                return
            i += read_length

    print("Flash verified, setting the newly flashed program as the default boot option and running it...")

    set_default_boot_main_program()

    jump_to_main_program()



if __name__ == "__main__":
    filename = sys.argv[1]
    # if bin_filename.endswith(".bin"):
    #     bin_data = open(bin_filename, "rb").read()
    #     bin_data = [ord(x) for x in bin_data]
    # elif bin_filename.endswith(".hex"):

    from intelhex import IntelHex
    ih = IntelHex()
    ih.fromfile(filename,format='hex')

    target_i2c    =  ih[200000]
    target_chipid =  ih[200001]
    target_chipid += ih[200002] << 8
    nuvoton_class =  ih[200003]
    saved_crc     =  ih[200004]
    saved_crc     += ih[200005] << 8

    assert( nuvoton_class in [16, 32] )

    if nuvoton_class == 32:
        bootloader_id       = 0xB004
        bootloader_version  = 211
        flash_size          = 32768-2048

    del ih[200004]
    del ih[200005]

    bin_data = list(ih.tobinarray())
    crc = binascii.crc_hqx(bytearray(bin_data), 0)

    # print("crc, saved_crc", hex(crc), hex(saved_crc))
    assert( saved_crc == crc )

    for addr in range(200000, 200004):
        del ih[addr]

    bin_data = list(ih.tobinarray())

    print("Firmware length", len(bin_data))
    # print(repr(bin_data[:30]))

    # chip_id      = int( bin_filename.split(".")[0].split("_")[-1] , 16)
    # i2c_address  = int( bin_filename.split(".")[0].split("_")[-2] , 16)
    # file_crc     = int( bin_filename.split(".")[0].split("_")[-3] , 16)
    # crc = binascii.crc_hqx(str(bin_data), 0)
    # print("Calculated CRC:", hex(crc))
    # assert( file_crc == crc )

    firmware_update(bin_data, target_i2c, target_chipid)
