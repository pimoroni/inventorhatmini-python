#!/usr/bin/env python3

import time
import RPi.GPIO as GPIO
from colorsys import hsv_to_rgb
from rpi_ws281x import PixelStrip, Color
import atexit
import ioexpander as io
from inventorhatmini.errors import *
import sys

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
LED_ADC1 = 4
LED_ADC2 = 5
LED_ADC3 = 6
LED_ADC4 = 7


# Count Constants
NUM_MOTORS = 2
NUM_SERVOS = 4
NUM_ADCS = 4
NUM_LEDS = 8


FAST_DECAY = 0  # aka 'Coasting'
SLOW_DECAY = 1  # aka 'Braking'

NORMAL_DIR = 0
REVERSED_DIR = 1

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

class MotorState():
    DEFAULT_SPEED_SCALE = 1.0  # The standard motor speed scale
    DEFAULT_ZEROPOINT = 0.0  # The standard motor zeropoint
    DEFAULT_DEADZONE = 0.05 # The standard motor deadzone

    DEFAULT_DECAY_MODE = SLOW_DECAY  # The standard motor decay behaviour
    DEFAULT_FREQUENCY = 25000.0  # The standard motor update rate
    MIN_FREQUENCY = 10.0
    MAX_FREQUENCY = 400000.0

    ZERO_PERCENT = 0.0
    ONEHUNDRED_PERCENT = 1.0


    def __init__(self, direction=NORMAL_DIR, speed_scale=DEFAULT_SPEED_SCALE, zeropoint=DEFAULT_ZEROPOINT, deadzone=DEFAULT_DEADZONE):
        self.motor_speed = 0.0
        self.last_enabled_duty = 0.0
        self.enabled = False

        self.motor_direction = direction
        self.motor_scale = max(speed_scale, sys.float_info.epsilon)
        self.motor_zeropoint = clamp(zeropoint, 0.0, 1.0 - sys.float_info.epsilon)
        self.motor_deadzone = clamp(deadzone, 0.0, 1.0)

    def enable_with_return(self):
        self.enabled = True
        return self.get_deadzoned_duty()

    def disable_with_return(self):
        self.enabled = False
        return None

    def is_enabled(self):
        return self.enabled

    def get_duty(self):
        if self.motor_direction == NORMAL_DIR:
            return self.last_enabled_duty
        else:
            return 0.0 - self.last_enabled_duty

    def get_deadzoned_duty(self):
        duty = 0.0
        if self.last_enabled_duty <= 0.0 - self.motor_deadzone or self.last_enabled_duty >= self.motor_deadzone:
            duty = self.last_enabled_duty

        if self.enabled:
            return duty
        else:
            return None

    def set_duty_with_return(self, duty):
        # Invert provided speed if the motor direction is reversed
        if self.motor_direction == REVERSED_DIR:
            duty = 0.0 - duty

        # Clamp the duty between the hard limits
        self.last_enabled_duty = clamp(duty, -1.0, 1.0)

        # Calculate the corresponding speed
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

        return self.enable_with_return()

    def get_speed(self):
        if self.motor_direction == NORMAL_DIR:
            return self.motor_speed
        else:
            return 0.0 - self.motor_speed

    def set_speed_with_return(self, speed):
        # Invert provided speed if the motor direction is reversed
        if self.motor_direction == REVERSED_DIR:
            speed = 0.0 - speed

        # Clamp the speed between the hard limits
        self.motor_speed = clamp(speed, 0.0 - self.motor_scale, self.motor_scale)

        # Calculate the corresponding duty cycle
        self.last_enabled_duty = MotorState.__speed_to_duty(self.motor_speed, self.motor_zeropoint, self.motor_scale)

        return self.enable_with_return()

    def stop_with_return(self):
        return self.set_duty_with_return(0.0)

    def full_negative_with_return(self):
        return self.set_duty_with_return(-1.0)

    def full_positive_with_return(self):
        return self.set_duty_with_return(1.0)

    def to_percent_with_return(self, input, in_min=ZERO_PERCENT, in_max=ONEHUNDRED_PERCENT, speed_min=None, speed_max=None):
        if speed_min is None:
            speed_min = 0.0 - self.motor_scale
        if speed_max is None:
            speed_max = self.motor_scale

        speed = MotorState.map_float(input, in_min, in_max, speed_min, speed_max)
        return set_speed_with_return(speed)

    def get_direction(self):
        return self.motor_direction

    def set_direction(self, direction):
        self.motor_direction = direction

    def get_speed_scale(self):
        return self.motor_scale

    def set_speed_scale(self, speed_scale):
        self.motor_scale = max(speed_scale, sys.float_info.epsilon)
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

    def get_zeropoint(self):
        return self.motor_zeropoint

    def set_zeropoint(self, zeropoint):
        self.motor_zeropoint = clamp(zeropoint, 0.0, 1.0 - sys.float_info.epsilon)
        self.motor_speed = MotorState.__duty_to_speed(self.last_enabled_duty, self.motor_zeropoint, self.motor_scale)

    def get_deadzone(self):
        return self.motor_deadzone

    def set_deadzone_with_return(self, deadzone):
        self.motor_deadzone = clamp(deadzone, 0.0, 1.0)
        return get_deadzoned_duty()

    def duty_to_level(duty, resolution):
        return int(duty * resolution)

    def map_float(input, in_min, in_max, out_min, out_max):
        return (((input - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min

    def __duty_to_speed(duty, zeropoint, scale):
        speed = 0.0
        if duty > zeropoint:
            speed = MotorState.map_float(duty, zeropoint, 1.0, 0.0, scale)
        elif duty < -zeropoint:
            speed = map_float(duty, -zeropoint, -1.0, 0.0, -scale)
        return speed

    def __speed_to_duty(speed, zeropoint, scale):
        duty = 0.0
        if speed > 0.0:
            duty = map_float(speed, 0.0, scale, zeropoint, 1.0)
        elif speed < 0.0:
            duty = map_float(speed, 0.0, -scale, -zeropoint, -1.0)
        return duty


class Motor():

    def __apply_duty(self, duty, mode):
        if duty is not None:
            signed_level = MotorState.duty_to_level(duty, self.pwm_period)

            if mode == SLOW_DECAY:  # aka 'Braking'
                if signed_level >= 0:
                    self.ioe.output(self.pin_p, self.pwm_period, load=False)
                    self.ioe.output(self.pin_n, self.pwm_period - signed_level, load=True)
                else:
                    self.ioe.output(self.pin_p, self.pwm_period + signed_level, load=False)
                    self.ioe.output(self.pin_n, self.pwm_period, load=True)

            elif mode == FAST_DECAY:  # aka 'Coasting'
                if signed_level >= 0:
                    self.ioe.output(self.pin_p, signed_level, load=False)
                    self.ioe.output(self.pin_n, 0, load=True)
                else:
                    self.ioe.output(self.pin_p, 0, load=False)
                    self.ioe.output(self.pin_n, 0 - signed_level, load=True)

        else:
            self.ioe.output(self.pin_p, 0)
            self.ioe.output(self.pin_n, 0)

    def __init__(self, ioe, pin_p, pin_n, direction=NORMAL_DIR, speed_scale=MotorState.DEFAULT_SPEED_SCALE, zeropoint=MotorState.DEFAULT_ZEROPOINT,
        deadzone=MotorState.DEFAULT_DEADZONE, freq=MotorState.DEFAULT_FREQUENCY, mode=MotorState.DEFAULT_DECAY_MODE):

        self.ioe = ioe

        self.pin_p = pin_p
        self.pin_n = pin_n

        self.state = MotorState(direction, speed_scale, zeropoint, deadzone)

        self.pwm_frequency = freq
        self.motor_mode = mode


        self.pin_p_mod = self.ioe.get_pwm_module(pin_p)
        self.pin_n_mod = self.ioe.get_pwm_module(pin_n)
        if self.pin_p_mod != self.pin_n_mod:
            raise ValueError("Both motor pins must be on the same PWM module!")

        self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_p_mod, load=False)

        ioe.set_mode(pin_p, io.PWM)
        ioe.set_mode(pin_n, io.PWM)
        ioe.output(pin_p, 0, load=False)
        ioe.output(pin_n, 0, load=True)

    def enable(self):
        self.__apply_duty(self.state.enable_with_return(), self.motor_mode)

    def disable(self):
        self.__apply_duty(self.state.disable_with_return(), self.motor_mode)

    def is_enabled(self):
        return self.state.is_enabled()

    def duty(self, duty=None):
        if duty is None:
            return self.state.get_duty()
        else:
            self.__apply_duty(self.set_duty_with_return(duty), self.motor_mode)

    def speed(self, speed=None):
        if speed is None:
            return self.state.get_speed()
        else:
            self.__apply_duty(self.set_speed_with_return(speed), self.motor_mode)

    def frequency(self, freq=None):
        if freq is None:
            return self.pwm_frequency
        else:
            if (freq >= MotorState.MIN_FREQUENCY) and (freq <= MotorState.MAX_FREQUENCY):
                self.__apply_duty(self.state.get_deadzoned_duty(), self.motor_mode)
                self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, pin_p_mod, load=True)
            else:
                raise ValueError(f"freq out of range. Expected {MotorState.MIN_FREQUENCY} to {MotorState.MAX_FREQUENCY}")

    def stop(self):
        self.__apply_duty(self.state.stop_with_return(), self.motor_mode)

    def coast(self):
        self.__apply_duty(self.state.stop_with_return(), FAST_DECAY)

    def brake(self):
        self.__apply_duty(self.state.stop_with_return(), SLOW_DECAY)

    def full_negative(self):
        self.__apply_duty(self.state.full_negative_with_return(), self.motor_mode)

    def full_positive(self):
        self.__apply_duty(self.state.full_positive_with_return(), self.motor_mode)

    def to_percent(self, input, in_min=MotorState.ZERO_PERCENT, in_max=MotorState.ONEHUNDRED_PERCENT, speed_min=None, speed_max=None):
        self.__apply_duty(self.state.to_percent_with_return(input, in_min, in_max, speed_min, speed_max), motor_mode)

    def direction(self, direction=None):
        if direction is None:
            return self.state.get_direction()
        else:
            self.set_direction(direction)

    def speed_scale(self, speed_scale=None):
        if speed_scale is None:
            return self.state.get_speed_scale()
        else:
            self.set_speed_scale(speed_scale)

    def zeropoint(self, zeropoint=None):
        if zeropoint is None:
            return self.state.get_zeropoint()
        else:
            self.set_zeropoint(zeropoint)

    def deadzone(self, deadzone=None):
        if deadzone is None:
            return self.state.get_deadzone()
        else:
            self.__apply_duty(self.state.set_deadzone_with_return(deadzone), self.motor_mode)

    def decay_mode(self, mode=None):
        if mode is None:
            return self.motor_mode
        else:
            self.motor_mode = mode
            self.__apply_duty(self.state.get_deadzoned_duty(), self.motor_mode)

class Servo():
    def __init__(self, ioexpander, pin, load=True):
        self.__ioe.set_mode(pin, io.PWM)
        self.__ioe.output(pin, 0, load=load)

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
    IOE_ENCODER_A_PINS = (4, 3)
    IOE_ENCODER_B_PINS = (1, 26)

    # Expander servo pins
    IOE_SERVO_1_PIN = 23
    IOE_SERVO_2_PIN = 24
    IOE_SERVO_3_PIN = 25
    IOE_SERVO_4_PIN = 22

    # Expander ADC pins
    IOE_ADC_1_PIN = 14
    IOE_ADC_2_PIN = 13
    IOE_ADC_3_PIN = 9
    IOE_ADC_4_PIN = 10

    LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
    LED_DMA = 10          # DMA channel to use for generating signal (try 10)
    LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
    LED_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
    LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
    LED_GAMMA = [
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
        2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
        6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11,
        11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
        19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
        29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
        40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
        55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
        71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
        90, 91, 93, 94, 95, 96, 98, 99,100,102,103,104,106,107,109,110,
        111,113,114,116,117,119,120,121,123,124,126,128,129,131,132,134,
        135,137,138,140,142,143,145,146,148,150,151,153,155,157,158,160,
        162,163,165,167,169,170,172,174,176,178,179,181,183,185,187,189,
        191,193,194,196,198,200,202,204,206,208,210,212,214,216,218,220,
        222,224,227,229,231,233,235,237,239,241,244,246,248,250,252,255]

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
            self.__ioe = io.SuperIOE(i2c_addr=self.IOE_ADDRESS, perform_reset=True)
        except TimeoutError:
            raise TimeoutError(NO_IOE_MSG) from None
        except OSError:
            raise OSError(NO_IOE_MSG) from None
        except FileNotFoundError:
            raise RuntimeError(NO_I2C) from None

        if init_motors:
            self.motor_a = Motor(self.__ioe, self.IOE_MOTOR_A_PINS[0], self.IOE_MOTOR_A_PINS[1])
            self.motor_b = Motor(self.__ioe, self.IOE_MOTOR_B_PINS[0], self.IOE_MOTOR_B_PINS[1])

        if init_leds:
            # Setup the PixelStrip object to use with Inventor's LEDs
            self.leds = PixelStrip(NUM_LEDS, self.PI_LED_DATA_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL, self.LED_GAMMA)
            try:
                # Attempt to initialise the library
                self.leds.begin()
                self.leds.show()
            except:
                self.leds = None
                raise RuntimeError(LED_INIT_FAILED) from None
        else:
            self.leds = None

        atexit.register(self.__cleanup)

    def __cleanup(self):
        if self.leds is not None:
            for i in range(self.leds.numPixels()):
                self.leds.setPixelColor(i, 0)
            self.leds.show()

        if self.motor_a is not None:
            self.motor_a.coast()
            self.motor_b.coast()

        GPIO.cleanup()

        """
        # Set up the motors and encoders, if the user wants them
        self.motors = None
        if init_motors:
            cpr = MMME_CPR * motor_gear_ratio
            self.motors = [Motor(self.MOTOR_A_PINS), Motor(self.MOTOR_B_PINS)]
            # Set the encoders to use PIO 0 and State Machines 0 and 1
            self.encoders = [Encoder(0, 0, self.ENCODER_A_PINS, counts_per_rev=cpr, count_microsteps=True),
                             Encoder(0, 1, self.ENCODER_B_PINS, counts_per_rev=cpr, count_microsteps=True)]

        # Set up the servos, if the user wants them
        self.servos = None
        if init_servos:
            self.servos = [Servo(i) for i in range(self.SERVO_1_PIN, self.SERVO_6_PIN + 1)]

        # Set up the i2c for Qw/st and Breakout Garden
        self.i2c = PimoroniI2C(self.I2C_SDA_PIN, self.I2C_SCL_PIN, 100000)

        # Set up the amp enable
        self.__amp_en = Pin(self.AMP_EN_PIN, Pin.OUT)
        self.__amp_en.off()

        self.audio_pwm = PWM(Pin(self.PWM_AUDIO_PIN))
        self.__volume = self.DEFAULT_VOLUME

        # Set up the user switch
        self.__switch = Pin(self.USER_SW_PIN, Pin.IN, Pin.PULL_DOWN)

        # Set up the WS2812 LEDs, using PIO 0 and State Machine 2
        self.leds = WS2812(NUM_LEDS, 0, 2, self.LED_DATA_PIN)
        self.leds.start()
        """

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
        #GPIO.output(self.MOTOR_EN_PIN, False)

    #########
    # Servo #
    #########

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
            else:
                print("User Switch released")
                board.mute_audio()
                if board.leds is not None:
                    for i in range(board.leds.numPixels()):
                        board.leds.setPixelColor(i, 0xFF0000)
                    board.leds.show()
                board.motor_a.coast()

        last_state = state

        time.sleep(0.01)
