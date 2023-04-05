import time
from inventorhatmini import InventorHATMini, NUM_SERVOS, LED_SERVO_1
from ioexpander import OUT

"""
Shows how to initialise and use Inventor HAT Mini's servo headers as 3.3V outputs and set them.

Press "User" to exit the program.
"""

# Constants
BRIGHTNESS = 0.4                        # The brightness of the LEDs
SERVO_NAMES = ("S0", "S1", "S2", "S3")  # Friendly names to give the servos
USE_LEDS = True                         # Whether to use the LEDs to show servo state (requires code to run with sudo)

# Create a new InventorHATMini
board = InventorHATMini(init_servos=False, init_leds=USE_LEDS)

# The servo to set to high
current_servo = 0

# Setup each servo as an output
for i in range(NUM_SERVOS):
    board.servo_pin_mode(i, OUT)

# Set the servos until the user button is pressed
while not board.switch_pressed():

    # Set each servos in turn and print its value
    for i in range(NUM_SERVOS):
        # Set the pin to high if this is the current servo pin, otherwise low
        value = (i == current_servo)
        board.servo_pin_value(i, value)
        print(SERVO_NAMES[i], " = ", value, sep="", end=", ")

        # Set the neighbouring LED to a colour based on
        # the output, with Green for high and Blue for low
        if value:
            board.leds.set_hsv(i + LED_SERVO_1, 0.333, 1.0, BRIGHTNESS)
        else:
            board.leds.set_hsv(i + LED_SERVO_1, 0.666, 1.0, BRIGHTNESS)

    # Print a new line
    print()

    # Advance the current servo, and wrap if necessary
    current_servo += 1
    if current_servo >= NUM_SERVOS:
        current_servo = 0

    time.sleep(0.5)

# Set all the servo pins back to low
for i in range(NUM_SERVOS):
    board.servo_pin_value(i, False)

# Turn off the LED bars
board.leds.clear()
