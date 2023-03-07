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
UPDATE_RATE = 1 / UPDATES

# Create a new InventorHATMini
board = InventorHATMini()

# Variables
offset = 0.0


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Make rainbows until the user button is pressed
while not board.switch_pressed():

    # Record the start time of this loop
    start_time = time.monotonic()

    offset += SPEED / 1000.0

    # Update all the LEDs
    for i in range(NUM_LEDS):
        hue = float(i) / NUM_LEDS
        board.leds.set_hsv(i, hue + offset, 1.0, BRIGHTNESS)

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Turn off the LED bars
board.leds.clear()
