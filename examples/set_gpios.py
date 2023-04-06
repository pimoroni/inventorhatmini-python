import time
from inventorhatmini import InventorHATMini, NUM_GPIOS, LED_GPIO_1
from ioexpander import OUT

"""
Shows how to initialise and 4 GPIO headers of Inventor HAT Mini as outputs and set them.

Press "User" to exit the program.
"""

# Constants
BRIGHTNESS = 0.4                            # The brightness of the LEDs
GPIO_NAMES = ("GP0", "GP1", "GP2", "GP3")   # Friendly names to give the GPIOs
USE_LEDS = True                             # Whether to use the LEDs to show GPIO state (requires code to run with sudo)

# Create a new InventorHATMini
board = InventorHATMini(init_leds=USE_LEDS)

# The GPIO to set to high
current_gpio = 0

# Setup each GPIO as an output
for i in range(NUM_GPIOS):
    board.gpio_pin_mode(i, OUT)

# Set the GPIOs until the user button is pressed
while not board.switch_pressed():

    # Set each GPIO in turn and print its value
    for i in range(NUM_GPIOS):
        # Set the pin to high if this is the current gpio pin, otherwise low
        value = (i == current_gpio)
        board.gpio_pin_value(i, value)
        print(GPIO_NAMES[i], " = ", value, sep="", end=", ")

        # Set the neighbouring LED to a colour based on
        # the output, with Green for high and Blue for low
        if value:
            board.leds.set_hsv(i + LED_GPIO_1, 0.333, 1.0, BRIGHTNESS)
        else:
            board.leds.set_hsv(i + LED_GPIO_1, 0.666, 1.0, BRIGHTNESS)

    # Print a new line
    print()

    # Advance the current gpio, and wrap if necessary
    current_gpio += 1
    if current_gpio >= NUM_GPIOS:
        current_gpio = 0

    time.sleep(0.5)

# Set all the gpio pins back to low
for i in range(NUM_GPIOS):
    board.gpio_pin_value(i, False)

# Turn off the LED bars
board.leds.clear()
