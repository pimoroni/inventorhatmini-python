import time
from inventorhatmini import InventorHATMini, NUM_LEDS

"""
Displays a rotating rainbow pattern on Inventor HAT Mini's onboard LED bars.

Press "User" to exit the program.
"""

# Constants
SPEED = 5           # The speed that the LEDs will cycle at
BRIGHTNESS = 0.4    # The brightness of the LEDs
UPDATES = 50        # How many times the LEDs will be updated per second

# Create a new InventorHATMini
board = InventorHATMini()

# Variables
offset = 0.0

# Make rainbows until the user button is pressed
while not board.switch_pressed():

    offset += SPEED / 1000.0

    # Update all the LEDs
    for i in range(NUM_LEDS):
        hue = float(i) / NUM_LEDS
        board.leds.set_hsv(i, hue + offset, 1.0, BRIGHTNESS)

    time.sleep(1.0 / UPDATES)

# Turn off the LED bars
board.leds.clear()
