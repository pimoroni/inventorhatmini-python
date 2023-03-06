#!/usr/bin/env python3

import time
import atexit
import RPi.GPIO as GPIO
from ioexpander import SuperIOE, ADC
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
    PI_UART_RX_ECHO_PIN = 15

    IOE_ADDRESS = 0x17

    # Expander motor driver pins, via DRV8833PWP Dual H-Bridge
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

    # Internal sense pins
    IOE_VOLTAGE_SENSE = 11
    IOE_CURRENT_SENSES = [7, 8]

    SHUNT_RESISTOR = 0.47

    def __init__(self, motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False):
        """ Initialise inventor hat mini's hardware functions
        """

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Setup user button
        GPIO.setup(self.PI_USER_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Setup amplifier enable. This mutes the audio by default
        GPIO.setup(self.PI_AMP_EN_PIN, GPIO.OUT, initial=GPIO.LOW if start_muted else GPIO.HIGH)

        self.__cpr = MMME_CPR * motor_gear_ratio
        self.__init_motors = init_motors
        self.__init_servos = init_servos
        self.reinit()

        if init_leds:
            # Setup the PixelStrip object to use with Inventor's LEDs, wrapped in a Plasma class
            self.leds = Plasma(NUM_LEDS, self.PI_LED_DATA_PIN)
        else:
            # Setup a dummy Plasma class, so examples don't need to check LED presence
            self.leds = DummyPlasma()

        atexit.register(self.__cleanup)

    def reinit(self):
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
        if self.__init_motors:
            self.motors = [Motor(self.__ioe, self.IOE_MOTOR_A_PINS), Motor(self.__ioe, self.IOE_MOTOR_B_PINS)]
            self.encoders = [Encoder(self.__ioe, 1, self.IOE_ENCODER_A_PINS, counts_per_rev=self.__cpr, count_microsteps=True),
                             Encoder(self.__ioe, 2, self.IOE_ENCODER_B_PINS, counts_per_rev=self.__cpr, count_microsteps=True)]

        self.servos = None
        if self.__init_servos:
            self.servos = [Servo(self.__ioe, self.IOE_SERVO_PINS[i]) for i in range(NUM_SERVOS)]

        self.__ioe.set_mode(self.IOE_VOLTAGE_SENSE, ADC)
        self.__ioe.set_mode(self.IOE_CURRENT_SENSES[0], ADC)
        self.__ioe.set_mode(self.IOE_CURRENT_SENSES[1], ADC)

    def __cleanup(self):
        for motor in self.motors:
            motor.coast()

        for servo in self.servos:
            servo.disable()

        self.leds.clear()

        GPIO.cleanup()

    def switch_pressed(self):
        return GPIO.input(self.PI_USER_SW_PIN) != 0

    def enable_motors(self):
        """ Enables both motors.
        """
        if self.motors is not None:
            for motor in self.motors:
                motor.enable()

    def disable_motors(self):
        """ Disables both motors, allowing them to spin freely.
        """
        if self.motors is not None:
            for motor in self.motors:
                motor.disable()

    def read_voltage(self):
        return (self.__ioe.input(self.IOE_VOLTAGE_SENSE) * (10 + 3.9)) / 3.9

    def read_motor_current(self, motor):
        if motor < 0 or motor >= NUM_MOTORS:
            raise ValueError("motor out or range. Expected MOTOR_A (0) or MOTOR_B (1)")

        return self.__ioe.input(self.IOE_CURRENT_SENSES[motor]) / self.SHUNT_RESISTOR

    def mute_audio(self):
        GPIO.output(self.PI_AMP_EN_PIN, False)

    def unmute_audio(self):
        GPIO.output(self.PI_AMP_EN_PIN, True)

    def activate_watchdog(self):
        self.__ioe.activate_watchdog()

    def deactivate_watchdog(self):
        self.__ioe.deactivate_watchdog()

    def feed_watchdog(self):
        self.__ioe.reset_watchdog_counter()

    def watchdog_timeout_occurred(self):
        return self.__ioe.watchdog_timeout_occurred()

    def clear_watchdog_timeout(self):
        self.__ioe.clear_watchdog_timeout()

    def is_watchdog_active(self):
        return self.__ioe.is_watchdog_active()

    def set_watchdog_control(self, divider):
        self.__ioe.set_watchdog_control(divider)


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
