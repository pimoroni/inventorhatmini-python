import time
import math
from inventorhatmini import InventorHATMini, NUM_MOTORS

"""
A demonstration of using Inventor HAT Mini's internal watchdog to stop motors
from spinning when a simulated code lock-up occurs, by causing a board reset,
and recovering from it.

Press "User" to simulate a lock-up.
Stop the program using the stop button in Thonny or Ctrl+C in the Terminal
"""

# Constants
SPEED = 5             # The speed that the motors will cycle at
UPDATES = 50          # How many times to update motors per second
UPDATE_RATE = 1 / UPDATES
SPEED_EXTENT = 1.0    # How far from zero to drive the motors

# Create a new InventorHATMini
board = InventorHATMini(init_leds=False)


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Run in an infinite loop to show watchdog reset and recovery
while True:
    offset = 0.0

    # Configure and activate the watchdog
    board.set_watchdog_control(128)  # Accepts a multiple of two, up to 128
    board.activate_watchdog()
    print("Watchdog active")

    # Make waves until a watchdog reset occurs
    while not board.watchdog_timeout_occurred():

        # Record the start time of this loop
        start_time = time.monotonic()

        # Keep the watchdog fed so it does not reset the board
        board.feed_watchdog()

        offset += SPEED / 1000.0

        # Update both motors
        for i in range(NUM_MOTORS):
            angle = (i + offset) * math.pi
            board.motors[i].speed(math.sin(angle) * SPEED_EXTENT)

        # Put the code into a loop here whilst the user switch is pressed
        switch_pressed = False
        while board.switch_pressed():
            if not switch_pressed:
                print("Switch pressed")
                switch_pressed = True

        # Sleep until the next update, accounting for how long the above operations took to perform
        sleep_until(start_time + UPDATE_RATE)

    # The watchdog has reset the board
    print("Reset occurred")

    # Reinitialise the parts of InventorHATMini that were reset
    board.reinit()
    print("Reinitialising...")
    print()
