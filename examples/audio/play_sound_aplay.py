import time
import subprocess
from inventorhatmini import InventorHATMini

"""
Play a WAV audio file from your Inventor HAT Mini, using Aplay!

Press "User" to play the sound.

Note: WAV files need to be signed 16-bit
"""

# Create a new InventorHATMini
board = InventorHATMini(init_leds=False)

print("Press the 'User' button on your Inventor HAT Mini to play the sound.")

# Loop forever
while True:
    if board.switch_pressed():
        # Play the sound
        print("Playing ... ", end="")

        # Use aplay to play the sound
        command = "aplay -q ./ahoy.wav &"
        subprocess.run(command, shell=True)

        time.sleep(0.25)
        print("Done")
