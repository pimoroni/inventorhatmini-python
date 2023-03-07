import time
from inventorhatmini import InventorHATMini, NUM_GPIOS, LED_GPIO_1
from ioexpander import ADC

"""
Shows how to initialise and read the 4 ADC headers of Inventor HAT Mini.

Press "User" to exit the program.
"""

BRIGHTNESS = 0.4      # The brightness of the LEDs
UPDATES = 10          # How many times to update LEDs per second
ADC_NAMES = ("A0", "A1", "A2", "A3")
USE_LEDS = True       # Whether to use the LEDs to show ADC state (requires code to run with sudo)

# Create a new InventorHATMini
board = InventorHATMini(init_leds=USE_LEDS)

# Setup each GPIO as an analog input
for i in range(NUM_GPIOS):
    board.gpio_mode(i, ADC)

# Read the ADCs until the user button is pressed
while not board.switch_pressed():

    # Read each ADC in turn and print its voltage
    for i in range(NUM_GPIOS):
        voltage = board.gpio_value(i)
        print(ADC_NAMES[i], " = ", round(voltage, 3), sep="", end=", ")

        # Set the neighbouring LED to a colour based on the
        # voltage, with Green for high and Blue for low
        hue = (2.0 - (voltage / 3.3)) * 0.333
        board.leds.set_hsv(i + LED_GPIO_1, hue, 1.0, BRIGHTNESS)

    # Print a new line
    print()

    time.sleep(1.0 / UPDATES)

# Turn off the LED bars
board.leds.clear()
