import time
from inventorhatmini import MotorHATMini

"""
A simple program that resets Inventor HAT Mini,
turning off its LEDs, Motors, Servos, and Audio.
"""

board = MotorHATMini(init_servos=False)

while True:
    board.status_led(True)
    time.sleep(1.0)
    board.status_led(False)
    time.sleep(0.5)