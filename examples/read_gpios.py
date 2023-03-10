import time
from inventorhatmini import InventorHATMini, NUM_GPIOS
from ioexpander import IN  # or IN_PU of a pull-up is wanted

"""
Shows how to initialise and read the 4 GPIO headers of Inventor HAT Mini.

Press "User" to exit the program.
"""

BRIGHTNESS = 0.4      # The brightness of the LEDs
GPIO_NAMES = ("GP0", "GP1", "GP2", "GP3")
USE_LEDS = True       # Whether to use the LEDs to show GPIO state (requires code to run with sudo)

# Create a new InventorHATMini
board = InventorHATMini(init_leds=USE_LEDS)

# Setup each GPIO as an input
for i in range(NUM_GPIOS):
    board.gpio_mode(i, IN)  # or IN_PU of a pull-up is wanted

# Read the GPIOs until the user button is pressed
while not board.switch_pressed():

    # Read each GPIO in turn and print its value
    for i in range(NUM_GPIOS):
        value = board.gpio_value(i)
        print(GPIO_NAMES[i], " = ", value, sep="", end=", ")

        # Set the neighbouring LED to a colour based on
        # the input, with Green for high and Blue for low
        if value:
            board.leds.set_hsv(i, 0.333, 1.0, BRIGHTNESS)
        else:
            board.leds.set_hsv(i, 0.666, 1.0, BRIGHTNESS)

    board.leds.show()

    # Print a new line
    print()

    time.sleep(0.1)

# Turn off the LED bars
board.leds.clear()
