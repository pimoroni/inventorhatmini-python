import pygame
from inventorhatmini import InventorHATMini

"""
Play a WAV audio file from your Inventor HAT Mini!

Press "User" to play the sound.

Note: WAV files need to be signed 16-bit
"""

# Create a new InventorHATMini
board = InventorHATMini(init_leds=False)

# Initialise PyGame so we can play sounds with it
pygame.init()

# Load the sound from file
ahoy = pygame.mixer.Sound("./ahoy.wav")

# Loop forever
while True:
    if board.switch_pressed():
        # Play the sound
        channel = ahoy.play()
        print("Playing ... ", end="")

        # Wait for the sound to finish playing
        while channel.get_busy():
            pass

        print("Done")
