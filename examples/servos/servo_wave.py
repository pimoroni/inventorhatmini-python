import time
import math
from inventorhatmini import InventorHATMini, NUM_SERVOS, LED_SERVO_1

"""
An example of applying a wave pattern to a group of servos and the LEDs.

Press "User" to exit the program.
"""

# Constants
SPEED = 5             # The speed that the LEDs will cycle at
BRIGHTNESS = 0.4      # The brightness of the LEDs
UPDATES = 50          # How many times to update LEDs and Servos per second
UPDATE_RATE = 1 / UPDATES
SERVO_EXTENT = 70.0   # How far from zero to move the servos
USE_LEDS = True       # Whether to show a wave on the LEDs (requires code to run with sudo)

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

    # Update all the Servos
    for i in range(NUM_SERVOS):
        angle = ((i / NUM_SERVOS) + offset) * math.pi
        board.servos[i].value(math.sin(angle) * SERVO_EXTENT)

        # Read back the servo's angle and use that to set a hue on the neighbouring LED
        hue = ((board.servos[i].value() / SERVO_EXTENT) + 1) * 0.333
        board.leds.set_hsv(i + LED_SERVO_1, hue, 1.0, BRIGHTNESS)

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Stop all the servos
for s in board.servos:
    s.disable()

# Turn off the LEDs
board.leds.clear()
