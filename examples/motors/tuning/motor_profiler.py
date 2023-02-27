import time
from inventorhatmini import InventorHATMini, MOTOR_A
from ioexpander.common import NORMAL_DIR  # , REVERSED_DIR

"""
A program that profiles the speed of a motor across its PWM
duty cycle range using the attached encoder for feedback.
"""

GEAR_RATIO = 50                         # The gear ratio of the motor

DIRECTION = NORMAL_DIR                  # The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
SPEED_SCALE = 5.4                       # The scaling to apply to the motor's speed. Set this to the maximum measured speed
ZERO_POINT = 0.0                        # The duty cycle that corresponds with zero speed when plotting the motor's speed as a straight line
DEAD_ZONE = 0.0                         # The duty cycle below which the motor's friction prevents it from moving

DUTY_STEPS = 100                        # How many duty cycle steps to sample the speed of
SETTLE_TIME = 0.1                       # How long to wait after changing motor duty cycle
CAPTURE_TIME = 0.2                      # How long to capture the motor's speed at each step
SLEEP_STEP = 0.02                       # The time between taking counts during counting_sleep()

# Create a new InventorHATMini and get a motor and encoder from it
board = InventorHATMini(motor_gear_ratio=GEAR_RATIO, init_leds=False)
m = board.motors[MOTOR_A]
enc = board.encoders[MOTOR_A]

# Set the motor's speed scale, zeropoint, and deadzone
m.speed_scale(SPEED_SCALE)
m.zeropoint(ZERO_POINT)
m.deadzone(DEAD_ZONE)

# Set the motor and encoder's direction
m.direction(DIRECTION)
enc.direction(DIRECTION)


# The encoder needs to be read semi-regularly to avoid the chip's internal counter
# from wrapping and giving us wrong speed measurements, so use this instead of time.sleep()
def counting_sleep(duration):
    start = time.monotonic()
    while time.monotonic() - start < duration:
        enc.count()  # We don't do anything with the returned count
        time.sleep(SLEEP_STEP)


# Function that performs a single profiling step
def profile_at_duty(duty):
    # Set the motor to a new duty cycle and wait for it to settle
    if DIRECTION == 1:
        m.duty(0.0 - duty)
    else:
        m.duty(duty)
    counting_sleep(SETTLE_TIME)

    # Perform a dummy capture to clear the encoder
    enc.capture(SETTLE_TIME)

    # Wait for the capture time to pass
    counting_sleep(CAPTURE_TIME)

    # Perform a capture and read the measured speed
    capture = enc.capture(CAPTURE_TIME)
    measured_speed = capture.revolutions_per_second

    # These are some alternate speed measurements from the encoder
    # measured_speed = capture.revolutions_per_minute
    # measured_speed = capture.degrees_per_second
    # measured_speed = capture.radians_per_second

    # Print out the expected and measured speeds, as well as their difference
    print("Duty = ", m.duty(), ", Expected = ", m.speed(), ", Measured = ", measured_speed, ", Diff = ", m.speed() - measured_speed, sep="")


# Enable the motor to get started
m.enable()

print("Profiler Starting...")

# Profile from 0% up to one step below 100%
for i in range(DUTY_STEPS):
    profile_at_duty(i / DUTY_STEPS)

# Profile from 100% down to one step above 0%
for i in range(DUTY_STEPS):
    profile_at_duty((DUTY_STEPS - i) / DUTY_STEPS)

# Profile from 0% down to one step above -100%
for i in range(DUTY_STEPS):
    profile_at_duty(-i / DUTY_STEPS)

# Profile from -100% up to one step below 0%
for i in range(DUTY_STEPS):
    profile_at_duty(-(DUTY_STEPS - i) / DUTY_STEPS)

# Profile 0% again
profile_at_duty(0)

print("Profiler Finished...")

# Disable the motor now the profiler has finished
m.disable()
