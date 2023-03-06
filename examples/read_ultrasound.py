import time
import RPi.GPIO as GPIO
from inventorhatmini import InventorHATMini, NUM_LEDS

"""
Control a HC-SR04 style ultrasonic distance sensor using the UART header on InventorHATMini.

Press "User" to exit the program.
"""

UPDATES = 10                            # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES

LED_HUE = 0.333                         # The hue (between 0 and 1) to set the RGB LEDs to
MAX_LED_DISTANCE = 100.0                # The distance that will cause all the LEDs to be lit (in cm)
BRIGHTNESS = 0.4                        # The brightness of the RGB LED
USE_LEDS = True                         # Whether to show the distance on the LEDs (requires code to run with sudo)

# Speed of sound is 343m/s which we need in cm/ns for our distance measure
SPEED_OF_SOUND_CM_NS = 343 * 100 / 1E9  # 0.0000343 cm / ns

# Create a new InventorHATMini
board = InventorHATMini(init_leds=USE_LEDS)

# Setup the UART pins for ultrasonic sensor use
GPIO.setup(InventorHATMini.PI_UART_TX_TRIG_PIN, GPIO.OUT)
GPIO.setup(InventorHATMini.PI_UART_RX_ECHO_PIN, GPIO.IN)


# Function for reading a distance value from a connected ultrasound sensor
# Ported from github.com/pimoroni/trilobot-python
def read_distance(timeout=50, samples=3, offset=190000):
    """ Return a distance in cm from the ultrasound sensor.
    timeout: total time in ms to try to get distance reading
    samples: determines how many readings to average
    offset: Time in ns the measurement takes (prevents over estimates)
    The default offset here is about right for a Raspberry Pi 4.
    
    Returns the measured distance in centimetres as a float.
    
    To give more stable readings, this method will attempt to take several
    readings and return the average distance. You can set the maximum time
    you want it to take before returning a result so you have control over
    how long this method ties up your program. It takes as many readings
    up to the requested number of samples set as it can before the timeout
    total is reached. It then returns the average distance measured. Any
    readings where the single reading takes more than the timeout is
    ignored so these do not distort the average distance measured. If no
    valid readings are taken before the timeout then it returns zero.
    You can choose parameters to get faster but less accurate readings or
    take longer to get more samples to average before it returns. The
    timeout effectively limits the maximum distance the sensor can measure
    because if the sound pulse takes longer to return over the distance
    than the timeout set then this method returns zero rather than waiting.
    So to extend the distance that can be measured, use a larger timeout.
    """

    # Start timing
    start_time = time.perf_counter_ns()
    time_elapsed = 0
    count = 0  # Track now many samples taken
    total_pulse_durations = 0
    distance = -999

    # Loop until the timeout is exceeded or all samples have been taken
    while (count < samples) and (time_elapsed < timeout * 1000000):
        # Trigger
        GPIO.output(InventorHATMini.PI_UART_TX_TRIG_PIN, 1)
        time.sleep(.00001)  # 10 microseconds
        GPIO.output(InventorHATMini.PI_UART_TX_TRIG_PIN, 0)

        # Wait for the ECHO pin to go high
        # wait for the pulse rise
        GPIO.wait_for_edge(InventorHATMini.PI_UART_RX_ECHO_PIN, GPIO.RISING, timeout=timeout)
        pulse_start = time.perf_counter_ns()

        # And wait for it to fall
        GPIO.wait_for_edge(InventorHATMini.PI_UART_RX_ECHO_PIN, GPIO.FALLING, timeout=timeout)
        pulse_end = time.perf_counter_ns()

        # get the duration
        pulse_duration = pulse_end - pulse_start - offset
        if pulse_duration < 0:
            pulse_duration = 0  # Prevent negative readings when offset was too high

        # Only count reading if achieved in less than timeout total time
        if pulse_duration < timeout * 1000000:
            # Convert to distance and add to total
            total_pulse_durations += pulse_duration
            count += 1

        time_elapsed = time.perf_counter_ns() - start_time

    # Calculate average distance in cm if any successful reading were made
    if count > 0:
        # Calculate distance using speed of sound divided by number of samples and half
        # that as sound pulse travels from robot to obstacle and back (twice the distance)
        distance = total_pulse_durations * SPEED_OF_SOUND_CM_NS / (2 * count)

    return distance


# Continually read the distance until the user button is pressed
while not board.switch_pressed():

    distance = read_distance()
    print("Distance is {:.1f} cm".format(distance))

    for i in range(NUM_LEDS):
        if distance >= i * (MAX_LED_DISTANCE / NUM_LEDS):
            board.leds.set_hsv(i, LED_HUE, 1.0, BRIGHTNESS)

    time.sleep(UPDATE_RATE)

# Turn off the LEDs
board.leds.clear()
