import time
from inventorhatmini import MotorHATMini, MOTOR_A, MOTOR_B, NUM_MOTORS, NUM_LEDS
from ioexpander.common import PID, REVERSED_DIR

"""
A demonstration of driving both of Inventor HAT Mini's motor outputs through a
sequence of velocities, with the help of their attached encoders and PID control.

Press "User" to exit the program.
"""

# Wheel friendly names
LF = MOTOR_A
LR = MOTOR_B
RF = 2
RR = 3
NAMES = ["LF", "LR", "RF", "RR"]

# Constants
GEAR_RATIO = 50                         # The gear ratio of the motors
SPEED_SCALE = 5.4                       # The scaling to apply to each motor's speed to match its real-world speed

UPDATES = 100                           # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
TIME_FOR_EACH_MOVE = 2                  # The time to travel between each value
UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES
PRINT_DIVIDER = 4                       # How many of the updates should be printed (i.e. 2 would be every other update)

DRIVING_SPEED = 1.0                     # The speed to drive the wheels at, from 0.0 to SPEED_SCALE

# PID values
VEL_KP = 30.0                           # Velocity proportional (P) gain
VEL_KI = 0.0                            # Velocity integral (I) gain
VEL_KD = 0.4                            # Velocity derivative (D) gain

# Create a new InventorHATMini
board = MotorHATMini(motor_gear_ratio=GEAR_RATIO, init_servos=False)

# Set the speed scale of the motors
board.motors[LF].speed_scale(SPEED_SCALE)
board.motors[LR].speed_scale(SPEED_SCALE)
board.motors[RF].speed_scale(SPEED_SCALE)
board.motors[RR].speed_scale(SPEED_SCALE)

# Reverse the direction of the left motor and encoder
#board.motors[LF].direction(REVERSED_DIR)
#board.encoders[LEFT].direction(REVERSED_DIR)

# Create PID objects for position control
vel_pids = [PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE) for i in range(4)]


# Helper functions for driving in common directions
def drive_forward(speed):
    vel_pids[LF].setpoint = speed
    vel_pids[LR].setpoint = speed
    vel_pids[RF].setpoint = speed
    vel_pids[RR].setpoint = speed


def turn_right(speed):
    vel_pids[LF].setpoint = speed
    vel_pids[LR].setpoint = -speed
    vel_pids[RF].setpoint = speed
    vel_pids[RR].setpoint = -speed


def stop():
    vel_pids[LF].setpoint = 0
    vel_pids[LR].setpoint = -0
    vel_pids[RF].setpoint = 0
    vel_pids[RR].setpoint = -0


# Enable the motor to get started
for m in board.motors:
    m.enable()


# Variables
update = 0
print_count = 0
sequence = 0
captures = [None] * 4


# Sleep until a specific time in the future. Use this instead of time.sleep() to correct for
# inconsistent timings when dealing with complex operations or external communication
def sleep_until(end_time):
    time_to_sleep = end_time - time.monotonic()
    if time_to_sleep > 0.0:
        time.sleep(time_to_sleep)


# Continually move the motor until the user switch is pressed
while True:

    # Record the start time of this loop
    start_time = time.monotonic()

    # Capture the state of all the encoders
    for i in range(4):
        captures[i] = board.encoders[i].capture()

    for i in range(4):
        # Calculate the acceleration to apply to the motor to move it closer to the velocity setpoint
        accel = vel_pids[i].calculate(captures[i].revolutions_per_second)

        # Accelerate or decelerate the motor
        board.motors[i].speed(board.motors[i].speed() + (accel * UPDATE_RATE))

    # Print out the current motor values, but only on every multiple
    if print_count == 0:
        for i in range(4):
            print(NAMES[i], "=", captures[i].revolutions_per_second, end=", ")
        print()

    # Increment the print count, and wrap it
    print_count = (print_count + 1) % PRINT_DIVIDER

    update += 1     # Move along in time

    # Have we reached the end of this movement?
    if update >= UPDATES_PER_MOVE:
        update = 0  # Reset the counter

        # Move on to the next part of the sequence
        sequence += 1

        # Loop the sequence back around
        if sequence >= 2:
            sequence = 0

    # Set the motor speeds, based on the sequence
    if sequence == 0:
        drive_forward(DRIVING_SPEED)
    elif sequence == 1:
        drive_forward(-DRIVING_SPEED)
    elif sequence == 2:
        turn_right(DRIVING_SPEED)
    elif sequence == 3:
        turn_right(-DRIVING_SPEED)
    elif sequence == 4:
        stop()

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Stop all the motors
for m in board.motors:
    m.disable()

