import time
from inventorhatmini import InventorHATMini, NUM_SERVOS
from ioexpander import IN  # or IN_PU of a pull-up is wanted

"""
Shows how to initialise and use Inventor HAT Mini's servo headers as 3.3V inputs and read them.

Press "User" to exit the program.
"""

# Constants
BRIGHTNESS = 0.4                        # The brightness of the LEDs
SERVO_NAMES = ("S0", "S1", "S2", "S3")  # Friendly names to give the GPIOs
USE_LEDS = True                         # Whether to use the LEDs to show GPIO state (requires code to run with sudo)

# Create a new InventorHATMini
board = InventorHATMini(init_servos=False, init_leds=USE_LEDS)

# Setup each servo as an input
for i in range(NUM_SERVOS):
    board.servo_pin_mode(i, IN)  # or IN_PU of a pull-up is wanted

# Read the servos until the user button is pressed
while not board.switch_pressed():

    # Read each servo in turn and print its value
    for i in range(NUM_SERVOS):
        value = board.servo_pin_value(i)
        print(SERVO_NAMES[i], " = ", value, sep="", end=", ")

        # Set the neighbouring LED to a colour based on
        # the input, with Green for high and Blue for low
        if value:
            board.leds.set_hsv(i, 0.333, 1.0, BRIGHTNESS)
        else:
            board.leds.set_hsv(i, 0.666, 1.0, BRIGHTNESS)

    # Print a new line
    print()

    time.sleep(0.1)

# Turn off the LED bars
board.leds.clear()
