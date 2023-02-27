#!/usr/bin/env python3

import time
import atexit
import RPi.GPIO as GPIO
from ioexpander import SuperIOE
from ioexpander.motor import Motor
from ioexpander.servo import Servo
from ioexpander.encoder import Encoder, MMME_CPR
from inventorhatmini.plasma import Plasma, DummyPlasma
from inventorhatmini.errors import NO_IOE_MSG, NO_I2C

__version__ = '0.0.1'


# Index Constants
MOTOR_A = 0
MOTOR_B = 1

SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2
SERVO_4 = 3

ADC_1 = 0
ADC_2 = 1
ADC_3 = 1
ADC_4 = 2

LED_SERVO_1 = 0
LED_SERVO_2 = 1
LED_SERVO_3 = 2
LED_SERVO_4 = 3
LED_ADC_1 = 4
LED_ADC_2 = 5
LED_ADC_3 = 6
LED_ADC_4 = 7


# Count Constants
NUM_MOTORS = 2
NUM_SERVOS = 4
NUM_ADCS = 4
NUM_LEDS = 8


class InventorHATMini():
    # I2C pins
    PI_I2C_SDA_PIN = 2
    PI_I2C_SCL_PIN = 3
    PI_I2C_INT_PIN = 4

    # WS2812 pin
    PI_LED_DATA_PIN = 12

    # I2S Audio pins
    PI_AMP_EN_PIN = 25

    # User switch pin
    PI_USER_SW_PIN = 26

    # UART / HC-SR04 Ultrasound pins
    PI_UART_TX_TRIG_PIN = 14
    PI_UART_RX_ECHO_PIN = 25

    IOE_ADDRESS = 0x16

    # Expander motor driver pins, via DRV8833PWP Dual H-Bridge
    # IOE_MOTOR_EN_PIN = ?
    # IOE_MOTOR_FAULT_PIN = ?
    IOE_MOTOR_A_PINS = (20, 19)
    IOE_MOTOR_B_PINS = (16, 15)

    # Expander motor encoder pins
    IOE_ENCODER_A_PINS = (3, 4)
    IOE_ENCODER_B_PINS = (26, 1)

    # Expander servo pins
    IOE_SERVO_PINS = (23, 24, 25, 22)

    # Expander ADC pins
    IOE_ADC_1_PIN = 14
    IOE_ADC_2_PIN = 13
    IOE_ADC_3_PIN = 9
    IOE_ADC_4_PIN = 10

    # Speed of sound is 343m/s which we need in cm/ns for our distance measure
    SPEED_OF_SOUND_CM_NS = 343 * 100 / 1E9  # 0.0000343 cm / ns

    MOTOR_A_NAME = 'A'
    MOTOR_B_NAME = 'B'

    def __init__(self, motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False):
        """ Initialise inventor hat mini's hardware functions
        """

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Setup user button
        GPIO.setup(self.PI_USER_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Setup amplifier enable. This mutes the audio by default
        GPIO.setup(self.PI_AMP_EN_PIN, GPIO.OUT, initial=GPIO.LOW if start_muted else GPIO.HIGH)

        try:
            self.__ioe = SuperIOE(i2c_addr=self.IOE_ADDRESS, perform_reset=True)
        except TimeoutError:
            raise TimeoutError(NO_IOE_MSG) from None
        except OSError:
            raise OSError(NO_IOE_MSG) from None
        except FileNotFoundError:
            raise RuntimeError(NO_I2C) from None

        self.motors = None
        self.encoders = None
        if init_motors:
            cpr = MMME_CPR * motor_gear_ratio
            self.motors = [Motor(self.__ioe, self.IOE_MOTOR_A_PINS), Motor(self.__ioe, self.IOE_MOTOR_B_PINS)]
            self.encoders = [Encoder(self.__ioe, 1, self.IOE_ENCODER_A_PINS, counts_per_rev=cpr, count_microsteps=True),
                             Encoder(self.__ioe, 2, self.IOE_ENCODER_B_PINS, counts_per_rev=cpr, count_microsteps=True)]

        self.servos = None
        if init_servos:
            self.servos = [Servo(self.__ioe, self.IOE_SERVO_PINS[i]) for i in range(NUM_SERVOS)]

        if init_leds:
            # Setup the PixelStrip object to use with Inventor's LEDs
            self.leds = Plasma(NUM_LEDS, self.PI_LED_DATA_PIN)
        else:
            self.leds = DummyPlasma()

        atexit.register(self.__cleanup)

    def __cleanup(self):
        for motor in self.motors:
            motor.coast()

        for servo in self.servos:
            servo.disable()

        self.leds.clear()

        GPIO.cleanup()

    ##########
    # Button #
    ##########
    def switch_pressed(self):
        return GPIO.input(self.PI_USER_SW_PIN) != 0

    ########
    # LEDs #
    ########
    """
    """

    ##########
    # Motors #
    ##########
    def disable_motors(self):
        """ Disables both motors, allowing them to spin freely.
        """
        # GPIO.output(self.MOTOR_EN_PIN, False)

    def mute_audio(self):
        GPIO.output(self.PI_AMP_EN_PIN, False)

    def unmute_audio(self):
        GPIO.output(self.PI_AMP_EN_PIN, True)


if __name__ == "__main__":
    board = InventorHATMini(init_leds=False, start_muted=True)

    print("Inventor HAT Mini Function Test")

    # time.sleep(2.0)

    last_state = False
    while True:
        state = board.switch_pressed()
        if last_state is not state:
            if state:
                print("User Switch pressed")
                board.unmute_audio()
                if board.leds is not None:
                    for i in range(board.leds.numPixels()):
                        board.leds.setPixelColor(i, 0x00FF00)
                    board.leds.show()
                board.motor_a.full_positive()
                board.servo_1.value(20)
            else:
                print("User Switch released")
                board.mute_audio()
                if board.leds is not None:
                    for i in range(board.leds.numPixels()):
                        board.leds.setPixelColor(i, 0xFF0000)
                    board.leds.show()
                board.motor_a.coast()
                board.servo_1.value(-20)

        last_state = state

        time.sleep(0.01)
