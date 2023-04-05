import time
from inventorhatmini import InventorHATMini, GPIO_1, GPIO_2
# from ioexpander.common import REVERSED_DIR

"""
An example of how to read a rotary encoder connected to Inventor HAT Mini's GPIO pins.

Press "User" to exit the program.
"""

# Constants
CHANNEL = 3         # The encoder channel to use (1 and 2 are used by Inventor's motor connectors)

# Create a new InventorHATMini
board = InventorHATMini(init_leds=False)

# Create an Encoder object using two GPIO pins
enc = board.encoder_from_gpio_pins(CHANNEL, GPIO_1, GPIO_2)

# Uncomment the below line (and the top imports) to
# reverse the counting direction of the encoder
# enc.direction(REVERSED_DIR)

# Read the encoder until the user button is pressed
while not board.switch_pressed():

    # Print out the current step and turn of the rotary encoder
    print("Step = ", enc.step(), ", Turn = ", enc.turn(), sep="")

    time.sleep(0.1)
