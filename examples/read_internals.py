import time
from inventorhatmini import InventorHATMini, NUM_MOTORS

"""
Shows how to read the internal sensors of Inventor HAT Mini.

Press "User" to exit the program.
"""

MOTOR_NAMES = ["A", "B"]

# Create a new InventorHATMini
board = InventorHATMini(init_leds=False)

for motor in board.motors:
    motor.duty(1.0)

# Read the internal sensors until the user button is pressed
while not board.switch_pressed():

    # Read the voltage sense and print the value
    voltage = board.read_voltage()
    print("V =", round(voltage, 4), end=", ")

    # Read the current sense of each motor and print the value
    for i in range(NUM_MOTORS):
        current = board.read_motor_current(i)
        print(f"Current {MOTOR_NAMES[i]} = ", round(current, 4), end=", ")

    # Print a new line
    print()

    time.sleep(0.5)

for motor in board.motors:
    motor.disable()
