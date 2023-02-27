import time
import math
from inventorhatmini import InventorHATMini, NUM_LEDS, NUM_MOTORS

"""
An example of applying a wave pattern to Inventor HAT Mini's motors and LEDs.

Press "User" to exit the program.
"""

# Constants
SPEED = 5             # The speed that the LEDs and motors will cycle at
BRIGHTNESS = 0.4      # The brightness of the LEDs
UPDATES = 50          # How many times to update LEDs and motors per second
UPDATE_RATE = 1 / UPDATES
SPEED_EXTENT = 1.0    # How far from zero to drive the motors
HALF_LEDS = NUM_LEDS // 2
USE_LEDS = True       # Whether to show a pattern on the LEDs whilst driving (requires code to run with sudo)


# Create a new InventorHATMini
board = InventorHATMini(init_leds=USE_LEDS)

offset = 0.0


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Make waves until the user button is pressed
while not board.switch_pressed():

    # Record the start time of this loop
    start_time = time.monotonic()

    offset += SPEED / 1000.0

    # Update the LED bars
    for i in range(HALF_LEDS):
        hue = i / (NUM_LEDS * 4)
        board.leds.set_hsv(i, hue + offset, 1.0, BRIGHTNESS)
        board.leds.set_hsv(NUM_LEDS - i - 1, hue + offset, 1.0, BRIGHTNESS)

    # Update both motors
    for i in range(NUM_MOTORS):
        angle = (i + offset) * math.pi
        board.motors[i].speed(math.sin(angle) * SPEED_EXTENT)

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Stop both motors
for m in board.motors:
    m.disable()

# Turn off the LEDs
board.leds.clear()
