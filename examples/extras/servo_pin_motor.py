import time
from inventorhatmini import InventorHATMini, SERVO_2, SERVO_3
# from ioexpander.common import REVERSED_DIR

"""
Shows how to use Inventor HAT Mini's Servo headers to control an externally connected motor driver.

Note that servo pins on the same PWM module must be used. These are:
- SERVO_1 and SERVO_4
- SERVO_2 and SERVO_3

Press "User" to exit the program.
"""

# Constants
DIRECTION = NORMAL_DIR  # The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
SPEED = 5               # The speed that the motor will cycle at
UPDATES = 50            # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
SPEED_EXTENT = 1.0      # How far from zero to drive the motor

# Create a new InventorHATMini
board = InventorHATMini(init_servos=False, init_leds=False)

# Create an Encoder object using two GPIO pins
motor = board.motor_from_servo_pins(CHANNEL, SERVO_2, SERVO_3, direction=NORMAL_DIR)


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

    # Update the motor
    angle = (i + offset) * math.pi
    motor.speed(math.sin(angle) * SPEED_EXTENT)

    # Sleep until the next update, accounting for how long the above operations took to perform
    sleep_until(start_time + UPDATE_RATE)

# Stop the motor
motor.disable()
