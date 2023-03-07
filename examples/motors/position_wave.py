import time
import math
from inventorhatmini import InventorHATMini, NUM_MOTORS, MOTOR_A, MOTOR_B, LED_SERVO_1, LED_GPIO_4
from ioexpander.common import PID, REVERSED_DIR

"""
A demonstration of driving both of Inventor HAT Mini's motor outputs between
positions, with the help of their attached encoders and PID control.

Press "User" to exit the program.
"""

ENCODER_NAMES = ["A", "B"]

GEAR_RATIO = 50                         # The gear ratio of the motors

SPEED_SCALE = 5.4                       # The scaling to apply to each motor's speed to match its real-world speed

UPDATES = 100                           # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
TIME_FOR_EACH_MOVE = 2                  # The time to travel between each value
UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES
PRINT_DIVIDER = 4                       # How many of the updates should be printed (i.e. 2 would be every other update)

# LED constant
BRIGHTNESS = 0.4                        # The brightness of the RGB LED
USE_LEDS = True                         # Whether to show a wave on the LEDs (requires code to run with sudo)

# PID values
POS_KP = 0.14                           # Position proportional (P) gain
POS_KI = 0.0                            # Position integral (I) gain
POS_KD = 0.0022                         # Position derivative (D) gain


# Create a new InventorHATMini
board = InventorHATMini(motor_gear_ratio=GEAR_RATIO, init_leds=USE_LEDS)

# Set the speed scale of the motors
board.motors[MOTOR_A].speed_scale(SPEED_SCALE)
board.motors[MOTOR_B].speed_scale(SPEED_SCALE)

# Reverse the direction of the left motor and encoder
board.motors[MOTOR_A].direction(REVERSED_DIR)
board.encoders[MOTOR_A].direction(REVERSED_DIR)

# Create PID objects for position control
pos_pids = [PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE) for i in range(NUM_MOTORS)]

# Enable all motors
for m in board.motors:
    m.enable()


update = 0
print_count = 0

# Set the initial and end values
start_value = 0.0
end_value = 270.0

captures = [None] * NUM_MOTORS


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Continually move the motor until the user button is pressed
while not board.switch_pressed():

    # Record the start time of this loop
    start_time = time.monotonic()

    # Capture the state of all the encoders
    for i in range(NUM_MOTORS):
        captures[i] = board.encoders[i].capture(UPDATE_RATE)

    # Calculate how far along this movement to be
    percent_along = min(update / UPDATES_PER_MOVE, 1.0)

    for i in range(NUM_MOTORS):
        # Move the motor between values using cosine
        pos_pids[i].setpoint = (((-math.cos(percent_along * math.pi) + 1.0) / 2.0) * (end_value - start_value)) + start_value

        # Calculate the velocity to move the motor closer to the position setpoint
        vel = pos_pids[i].calculate(captures[i].degrees, captures[i].degrees_per_second)

        # Set the new motor driving speed
        board.motors[i].speed(vel)

    # Update the LEDs
    board.leds.set_hsv(LED_SERVO_1, percent_along, 1.0, BRIGHTNESS)
    board.leds.set_hsv(LED_GPIO_4, percent_along, 1.0, BRIGHTNESS)

    # Print out the current motor values and their setpoints, but only on every multiple
    if print_count == 0:
        for i in range(NUM_MOTORS):
            print(ENCODER_NAMES[i], "=", captures[i].degrees, end=", ")
        print()

    # Increment the print count, and wrap it
    print_count = (print_count + 1) % PRINT_DIVIDER

    update += 1     # Move along in time

    # Have we reached the end of this movement?
    if update >= UPDATES_PER_MOVE:
        update = 0  # Reset the counter

        # Swap the start and end values
        temp = start_value
        start_value = end_value
        end_value = temp

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Stop all the motors
for m in board.motors:
    m.disable()

# Turn off the LEDs
board.leds.clear()
