#!/usr/bin/env python3

import time
import warnings

import gpiod
import gpiodevice
from gpiod.line import Bias, Direction, Value
from gpiodevice import platform
from ioexpander import ADC, SuperIOE
from ioexpander.common import NORMAL_DIR
from ioexpander.encoder import MMME_CPR, ROTARY_CPR, Encoder
from ioexpander.motor import Motor, MotorState
from ioexpander.servo import Servo

from inventorhatmini.errors import NO_I2C, NO_IOE_MSG
from inventorhatmini.plasma import DummyPlasma, Plasma

__version__ = '1.0.0'


# Index Constants
MOTOR_A = 0
MOTOR_B = 1

SERVO_1 = 0
SERVO_2 = 1
SERVO_3 = 2
SERVO_4 = 3

GPIO_1 = 0
GPIO_2 = 1
GPIO_3 = 2
GPIO_4 = 3

LED_SERVO_1 = 0
LED_SERVO_2 = 1
LED_SERVO_3 = 2
LED_SERVO_4 = 3
LED_GPIO_1 = 4
LED_GPIO_2 = 5
LED_GPIO_3 = 6
LED_GPIO_4 = 7


# Count Constants
NUM_MOTORS = 2
NUM_SERVOS = 4
NUM_GPIOS = 4
NUM_LEDS = 8

INPD = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_DOWN)
OUTL = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
OUTH = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE)


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

    # Expander GPIO/ADC pins
    IOE_GPIO_PINS = (14, 13, 9, 18)

    # Internal sense pins
    IOE_VOLTAGE_SENSE = 11
    IOE_CURRENT_SENSES = (7, 8)

    SHUNT_RESISTOR = 0.47

    def __init__(self, address=IOE_ADDRESS, motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False):
        """ Initialise inventor hat mini's hardware functions
        """
        self.ioe = None

        gpiodevice.friendly_errors = True

        self.address = address

        if ( address == self.IOE_ADDRESS ): # only if using default address, for use with multiple HATs
            # Setup user button
            self._pin_user_sw = gpiodevice.get_pin(self.PI_USER_SW_PIN, "IHM-SW", INPD)

            # Setup amplifier enable. This mutes the audio by default
            self._pin_amp_en = gpiodevice.get_pin(self.PI_AMP_EN_PIN, "IHM-AMP-En", OUTL if start_muted else OUTH)

        self.__cpr = MMME_CPR * motor_gear_ratio
        self.__init_motors = init_motors
        self.__init_servos = init_servos
        self.reinit()

        is_pi5 = platform.get_name().startswith("Raspberry Pi 5")

        if init_leds and not is_pi5:
            # Setup the PixelStrip object to use with Inventor's LEDs, wrapped in a Plasma class
            self.leds = Plasma(NUM_LEDS, self.PI_LED_DATA_PIN)
        else:
            # Setup a dummy Plasma class, so examples don't need to check LED presence
            self.leds = DummyPlasma()

        if is_pi5:
            warnings.warn("LEDs are not yet supported on Pi 5.")

    def _write_pin(self, pin, state):
        lines, offset = pin
        lines.set_value(offset, Value.ACTIVE if state else Value.INACTIVE)

    def _read_pin(self, pin):
        lines, offset = pin
        return lines.get_value(offset) == Value.ACTIVE

    def reinit(self):
        try:
            self.ioe = SuperIOE(i2c_addr=self.address, perform_reset=True)
        except TimeoutError:
            raise TimeoutError(NO_IOE_MSG) from None
        except OSError:
            raise OSError(NO_IOE_MSG) from None
        except FileNotFoundError:
            raise RuntimeError(NO_I2C) from None

        self.motors = None
        self.encoders = None
        if self.__init_motors:
            self.motors = [Motor(self.ioe, self.IOE_MOTOR_A_PINS), Motor(self.ioe, self.IOE_MOTOR_B_PINS)]
            self.encoders = [Encoder(self.ioe, 1, self.IOE_ENCODER_A_PINS, counts_per_rev=self.__cpr, count_microsteps=True),
                             Encoder(self.ioe, 2, self.IOE_ENCODER_B_PINS, counts_per_rev=self.__cpr, count_microsteps=True)]

        self.servos = None
        if self.__init_servos:
            self.servos = [Servo(self.ioe, self.IOE_SERVO_PINS[i]) for i in range(NUM_SERVOS)]

        self.ioe.set_mode(self.IOE_VOLTAGE_SENSE, ADC)
        self.ioe.set_mode(self.IOE_CURRENT_SENSES[0], ADC)
        self.ioe.set_mode(self.IOE_CURRENT_SENSES[1], ADC)

    def __del__(self):
        if self.ioe:
            self.ioe.reset()

    def switch_pressed(self):
        return self._read_pin(self._pin_user_sw)

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
        return (self.ioe.input(self.IOE_VOLTAGE_SENSE) * (10 + 3.9)) / 3.9

    def read_motor_current(self, motor):
        if motor < 0 or motor >= NUM_MOTORS:
            raise ValueError("motor out of range. Expected MOTOR_A (0) or MOTOR_B (1)")

        return self.ioe.input(self.IOE_CURRENT_SENSES[motor]) / self.SHUNT_RESISTOR

    def mute_audio(self):
        self._write_pin(self._pin_amp_en, False)

    def unmute_audio(self):
        self._write_pin(self._pin_amp_en, True)

    def gpio_pin_mode(self, gpio, mode=None):
        if gpio < 0 or gpio >= NUM_GPIOS:
            raise ValueError("gpio out of range. Expected GPIO_1 (0), GPIO_2 (1), GPIO_3 (2) or GPIO_4 (3)")

        if mode is None:
            return self.ioe.get_mode(self.IOE_GPIO_PINS[gpio])
        else:
            self.ioe.set_mode(self.IOE_GPIO_PINS[gpio], mode)

    def gpio_pin_value(self, gpio, value=None):
        if gpio < 0 or gpio >= NUM_GPIOS:
            raise ValueError("gpio out of range. Expected GPIO_1 (0), GPIO_2 (1), GPIO_3 (2) or GPIO_4 (3)")

        if value is None:
            return self.ioe.input(self.IOE_GPIO_PINS[gpio])
        else:
            self.ioe.output(self.IOE_GPIO_PINS[gpio], value)

    def servo_pin_mode(self, servo, mode=None):
        if self.servos is not None:
            raise ValueError("servo pin is already in use by a Servo")

        if servo < 0 or servo >= NUM_SERVOS:
            raise ValueError("servo out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        if mode is None:
            return self.ioe.get_mode(self.IOE_SERVO_PINS[servo])
        else:
            self.ioe.set_mode(self.IOE_SERVO_PINS[servo], mode)

    def servo_pin_value(self, servo, value=None, load=True, wait_for_load=False):
        if self.servos is not None:
            raise ValueError("servo pin is already in use by a Servo")

        if servo < 0 or servo >= NUM_SERVOS:
            raise ValueError("servo out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        if value is None:
            return self.ioe.input(self.IOE_SERVO_PINS[servo])
        else:
            self.ioe.output(self.IOE_SERVO_PINS[servo], value, load=load, wait_for_load=wait_for_load)

    def servo_pin_load(self, servo, wait_for_load=True):
        if self.servos is not None:
            raise ValueError("servo pin is already in use by a Servo")

        if servo < 0 or servo >= NUM_SERVOS:
            raise ValueError("servo out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        module = self.ioe.get_pwm_module(self.IOE_SERVO_PINS[servo])
        self.ioe.pwm_load(module, wait_for_load)

    def servo_pin_frequency(self, servo, frequency, load=True, wait_for_load=True):
        if self.servos is not None:
            raise ValueError("servo pin is already in use by a Servo")

        if servo < 0 or servo >= NUM_SERVOS:
            raise ValueError("servo out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        module = self.ioe.get_pwm_module(self.IOE_SERVO_PINS[servo])
        self.ioe.set_pwm_frequency(frequency, module, load=load, wait_for_load=wait_for_load)

    def encoder_from_gpio_pins(self, channel, gpio_a, gpio_b, direction=NORMAL_DIR, counts_per_rev=ROTARY_CPR, count_microsteps=False):
        if self.encoders is not None:
            if channel == 1:
                raise ValueError("channel 1 is already in use by Motor A's encoder.")
            if channel == 2:
                raise ValueError("channel 1 is already in use by Motor B's encoder.")

        if channel < 1 or channel > 4:
            raise ValueError("channel out of range. Expected 1 to 4.")

        if gpio_a < 0 or gpio_a >= NUM_GPIOS:
            raise ValueError("gpio_a out of range. Expected GPIO_1 (0), GPIO_2 (1), GPIO_3 (2) or GPIO_4 (3)")

        if gpio_b < 0 or gpio_b >= NUM_GPIOS:
            raise ValueError("gpio_b out of range. Expected GPIO_1 (0), GPIO_2 (1), GPIO_3 (2) or GPIO_4 (3)")

        return Encoder(self.ioe, channel, (self.IOE_GPIO_PINS[gpio_a], self.IOE_GPIO_PINS[gpio_b]), direction=direction, counts_per_rev=counts_per_rev, count_microsteps=count_microsteps)

    def motor_from_servo_pins(self, servo_p, servo_n, direction=NORMAL_DIR, speed_scale=MotorState.DEFAULT_SPEED_SCALE, zeropoint=MotorState.DEFAULT_ZEROPOINT,
                              deadzone=MotorState.DEFAULT_DEADZONE, freq=MotorState.DEFAULT_FREQUENCY, mode=MotorState.DEFAULT_DECAY_MODE):
        if self.servos is not None:
            raise ValueError("servo pins are already in use by Servos")

        if servo_p < 0 or servo_p >= NUM_SERVOS:
            raise ValueError("servo_p out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        if servo_n < 0 or servo_n >= NUM_SERVOS:
            raise ValueError("servo_n out of range. Expected SERVO_1 (0), SERVO_2 (1), SERVO_3 (2) or SERVO_4 (3)")

        return Motor(self.ioe, (self.IOE_SERVO_PINS[servo_p], self.IOE_SERVO_PINS[servo_n]), direction=direction,
                     speed_scale=speed_scale, zeropoint=zeropoint, deadzone=deadzone, freq=freq, mode=mode)

    def activate_watchdog(self):
        self.ioe.activate_watchdog()

    def deactivate_watchdog(self):
        self.ioe.deactivate_watchdog()

    def feed_watchdog(self):
        self.ioe.reset_watchdog_counter()

    def watchdog_timeout_occurred(self):
        return self.ioe.watchdog_timeout_occurred()

    def clear_watchdog_timeout(self):
        self.ioe.clear_watchdog_timeout()

    def is_watchdog_active(self):
        return self.ioe.is_watchdog_active()

    def set_watchdog_control(self, divider):
        self.ioe.set_watchdog_control(divider)


if __name__ == "__main__":
    board = InventorHATMini(start_muted=True)

    print("Inventor HAT Mini Function Test")

    last_state = False
    while True:
        state = board.switch_pressed()
        if last_state is not state:
            if state:
                print("User Switch pressed")
                board.unmute_audio()
                for i in range(NUM_LEDS):
                    board.leds.set_rgb(i, 0, 255, 0, show=False)
                board.leds.show()
                board.motors[MOTOR_A].full_positive()
                board.servos[SERVO_1].value(20)
            else:
                print("User Switch released")
                board.mute_audio()
                for i in range(NUM_LEDS):
                    board.leds.set_rgb(i, 255, 0, 0, show=False)
                board.leds.show()
                board.motors[MOTOR_A].coast()
                board.servos[SERVO_1].value(-20)

        last_state = state

        time.sleep(0.01)
