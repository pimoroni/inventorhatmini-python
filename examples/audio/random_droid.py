import time
import random
from inventorhatmini import InventorHATMini
from ttastromech import TTAstromech

"""
Make your Inventor HAT Mini sound like an Astromech Droid!

This example uses the ttastromech library, for more details see https://pypi.org/project/ttastromech/
To install enter the following on the command line: pip install ttastromech

Press "User" to play a random word out of the speaker.
"""

# Constants
MIN_WORD_LENGTH = 3     # The smallest word size
MAX_WORD_LENGTH = 10    # The largest word size
WAIT_TIME = 0.25        # The time (in seconds) to wait after playing a word, before accepting user input again

# Create a new InventorHATMini and TTAstromech
board = InventorHATMini(init_leds=False)
droid = TTAstromech()


# Generate the characters from `c1` to `c2`, inclusive.
def char_range(c1, c2):
    for c in range(ord(c1), ord(c2) + 1):
        yield chr(c)


# Generate a random word from the letters a to z.
def random_string(word_length=5, no_repeat=True):
    chars = list(char_range("a", "z"))
    word = ""
    char_count = 0
    while char_count != word_length:
        char = chars[random.randint(0, 25)]
        if char in word and no_repeat:
            pass
        else:
            word += char
            char_count += 1
    print("Saying:", word)
    return word


# Loop forever
while True:
    if board.switch_pressed():
        # Speak a random word when "User" is pressed
        word_length = random.randint(MIN_WORD_LENGTH, MAX_WORD_LENGTH)
        droid.speak(random_string(word_length, no_repeat=False))
        time.sleep(WAIT_TIME)
