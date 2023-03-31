import time
import math
import random
from inventorhatmini import InventorHATMini, SERVO_1

"""
An example of how to move a servo smoothly between random positions.

Press "User" to exit the program.
"""

# Constants
UPDATES = 50            # How many times to update Servos per second
UPDATE_RATE = 1 / UPDATES
TIME_FOR_EACH_MOVE = 2  # The time to travel between each random value
UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES

SERVO_EXTENT = 70       # How far from zero to move the servo
USE_COSINE = True       # Whether or not to use a cosine path between values

# Create a new InventorHATMini and get a servo from it
board = InventorHATMini(init_leds=False)
s = board.servos[SERVO_1]

# Get the initial value and create a random end value between the extents
start_value = s.mid_value()
end_value = random.uniform(-SERVO_EXTENT, SERVO_EXTENT)


update = 0


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Continually move the servo until the user button is pressed
while not board.switch_pressed():

    # Record the start time of this loop
    start_time = time.monotonic()

    # Calculate how far along this movement to be
    percent_along = update / UPDATES_PER_MOVE

    if USE_COSINE:
        # Move the servo between values using cosine
        s.to_percent(math.cos(percent_along * math.pi), 1.0, -1.0, start_value, end_value)
    else:
        # Move the servo linearly between values
        s.to_percent(percent_along, 0.0, 1.0, start_value, end_value)

    # Print out the value the servo is now at
    print("Value = ", round(s.value(), 3), sep="")

    # Move along in time
    update += 1

    # Have we reached the end of this movement?
    if update >= UPDATES_PER_MOVE:
        # Reset the counter
        update = 0

        # Set the start as the last end and create a new random end value
        start_value = end_value
        end_value = random.uniform(-SERVO_EXTENT, SERVO_EXTENT)

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Disable the servo
s.disable()
