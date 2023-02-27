#!/usr/bin/env python3

import math
import time
import RPi.GPIO as GPIO
from colorsys import hsv_to_rgb
from rpi_ws281x import PixelStrip, Color
import atexit
import ioexpander as io
from inventorhatmini.errors import NO_IOE_MSG, NO_I2C, LED_INIT_FAILED
import sys
from collections import namedtuple

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


FAST_DECAY = 0  # aka 'Coasting'
SLOW_DECAY = 1  # aka 'Braking'

NORMAL_DIR = 0
REVERSED_DIR = 1


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def map_float(input, in_min, in_max, out_min, out_max):
    return (((input - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min


# A simple class for handling Proportional, Integral & Derivative (PID) control calculations
class PID:
    def __init__(self, kp, ki, kd, sample_rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0
        self._error_sum = 0
        self._last_value = 0
        self._sample_rate = sample_rate

    def calculate(self, value, value_change=None):
        error = self.setpoint - value
        self._error_sum += error * self._sample_rate
        if value_change is None:
            rate_error = (value - self._last_value) / self._sample_rate
        else:
            rate_error = value_change
        self._last_value = value

        return (error * self.kp) + (self._error_sum * self.ki) - (rate_error * self.kd)


class MotorState():
    DEFAULT_SPEED_SCALE = 1.0  # The standard motor speed scale
    DEFAULT_ZEROPOINT = 0.0  # The standard motor zeropoint
    DEFAULT_DEADZONE = 0.05  # The standard motor deadzone

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

        speed = map_float(input, in_min, in_max, speed_min, speed_max)
        return self.set_speed_with_return(speed)

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
        return self.get_deadzoned_duty()

    def duty_to_level(duty, resolution):
        return int(duty * resolution)

    def __duty_to_speed(duty, zeropoint, scale):
        speed = 0.0
        if duty > zeropoint:
            speed = map_float(duty, zeropoint, 1.0, 0.0, scale)
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
            self.ioe.output(self.pin_p, 0, load=False)
            self.ioe.output(self.pin_n, 0, load=True)

    def __init__(self, ioe, pins, direction=NORMAL_DIR, speed_scale=MotorState.DEFAULT_SPEED_SCALE, zeropoint=MotorState.DEFAULT_ZEROPOINT,
                 deadzone=MotorState.DEFAULT_DEADZONE, freq=MotorState.DEFAULT_FREQUENCY, mode=MotorState.DEFAULT_DECAY_MODE):

        self.ioe = ioe

        if not isinstance(pins, list) and not isinstance(pins, tuple):
            raise TypeError("cannot convert object to a list or tuple of pins")

        if len(pins) != 2:
            raise TypeError("list or tuple must only contain two integers")

        self.pin_p = pins[0]
        self.pin_n = pins[1]

        self.state = MotorState(direction, speed_scale, zeropoint, deadzone)

        self.pwm_frequency = freq
        self.motor_mode = mode

        self.pin_p_mod = self.ioe.get_pwm_module(self.pin_p)
        self.pin_n_mod = self.ioe.get_pwm_module(self.pin_n)
        if self.pin_p_mod != self.pin_n_mod:
            raise ValueError("Both motor pins must be on the same PWM module!")

        self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_p_mod, load=False)

        ioe.set_mode(self.pin_p, io.PWM)
        ioe.set_mode(self.pin_n, io.PWM)
        ioe.output(self.pin_p, 0, load=False)
        ioe.output(self.pin_n, 0, load=True)

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
            self.__apply_duty(self.state.set_duty_with_return(duty), self.motor_mode)

    def speed(self, speed=None):
        if speed is None:
            return self.state.get_speed()
        else:
            self.__apply_duty(self.state.set_speed_with_return(speed), self.motor_mode)

    def frequency(self, freq=None):
        if freq is None:
            return self.pwm_frequency
        else:
            if (freq >= MotorState.MIN_FREQUENCY) and (freq <= MotorState.MAX_FREQUENCY):
                self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_p_mod, load=False)
                self.__apply_duty(self.state.get_deadzoned_duty(), self.motor_mode)
            else:
                raise ValueError(f"freq out of range. Expected {MotorState.MIN_FREQUENCY}Hz to {MotorState.MAX_FREQUENCY}Hz")

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
        self.__apply_duty(self.state.to_percent_with_return(input, in_min, in_max, speed_min, speed_max), self.motor_mode)

    def direction(self, direction=None):
        if direction is None:
            return self.state.get_direction()
        else:
            self.state.set_direction(direction)

    def speed_scale(self, speed_scale=None):
        if speed_scale is None:
            return self.state.get_speed_scale()
        else:
            self.state.set_speed_scale(speed_scale)

    def zeropoint(self, zeropoint=None):
        if zeropoint is None:
            return self.state.get_zeropoint()
        else:
            self.state.set_zeropoint(zeropoint)

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


Pair = namedtuple("Pair", ["pulse", "value"])

ANGULAR = 0
LINEAR = 1
CONTINUOUS = 2


class Calibration():
    DEFAULT_MIN_PULSE = 500.0   # in microseconds
    DEFAULT_MID_PULSE = 1500.0  # in microseconds
    DEFAULT_MAX_PULSE = 2500.0  # in microseconds

    LOWER_HARD_LIMIT = 400.0    # The minimum microsecond pulse to send
    UPPER_HARD_LIMIT = 2600.0   # The maximum microsecond pulse to send

    def __init__(self, default_type=None):
        self.calibration = None
        self.limit_lower = True
        self.limit_upper = True

        if default_type is not None:
            if not isinstance(default_type, int):
                raise TypeError("cannot convert object to an integer")

            if default_type < 0 or default_type >= 3:
                raise ValueError("type out of range. Expected ANGULAR (0), LINEAR (1) or CONTINUOUS (2)")

            self.apply_default_pairs(default_type)

    def __str__(self):
        size = self.size()
        text = f"Calibration(size = {size}, pairs = {{"
        for i in range(size):
            pair = self.calibration[i]
            text += f"{{{pair.pulse}, {pair.value}}}"
            if i < size - 1:
                text +=", "
        text += f"}}, limit_lower = {self.has_lower_limit()}, limit_upper = {self.has_upper_limit()})"
        return text

    def apply_blank_pairs(self, size):
        if size < 0:
            raise ValueError("size out of range. Expected 0 or greater")

        if self.calibration is not None:
            self.calibration = None

        if size > 0:
            self.calibration = [Pair(0, 0)] * size
        else:
            self.calibration = None

    def apply_two_pairs(self, min_pulse, max_pulse, min_value, max_value):
        self.apply_blank_pairs(2)
        self.calibration[0] = Pair(min_pulse, min_value)
        self.calibration[1] = Pair(max_pulse, max_value)

    def apply_three_pairs(self, min_pulse, mid_pulse, max_pulse, min_value, mid_value, max_value):
        self.apply_blank_pairs(3)
        self.calibration[0] = Pair(min_pulse, min_value)
        self.calibration[1] = Pair(mid_pulse, mid_value)
        self.calibration[2] = Pair(max_pulse, max_value)

    def apply_uniform_pairs(self, size, min_pulse, max_pulse, min_value, max_value):
        self.apply_blank_pairs(size)
        if size > 0:
            size_minus_one = size - 1
            for i in range(0, size):
                pulse = map_float(i, 0.0, size_minus_one, min_pulse, max_pulse)
                value = map_float(i, 0.0, size_minus_one, min_value, max_value)
                self.calibration[i] = Pair(pulse, value)

    def apply_default_pairs(self, default_type):
        if default_type == ANGULAR:
            self.apply_three_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MID_PULSE, self.DEFAULT_MAX_PULSE,
                                   -90.0, 0.0, +90.0)

        elif default_type == LINEAR:
            self.apply_two_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MAX_PULSE,
                                 0.0, 1.0)

        elif default_type == CONTINUOUS:
            self.apply_three_pairs(self.DEFAULT_MIN_PULSE, self.DEFAULT_MID_PULSE, self.DEFAULT_MAX_PULSE,
                                   -1.0, 0.0, +1.0)

    def size(self):
        return len(self.calibration)

    def pair(self, index, pair=None):  # Ensure the pairs are assigned in ascending value order
        if pair is None:
            return self.calibration[index]
        else:
            self.calibration[index] = Pair._make(pair)

    def pulse(self, index, pulse=None):
        if pulse is None:
            return self.calibration[index].pulse
        else:
            value = self.calibration[index].value
            self.calibration[index] = Pair(pulse, value)

    def value(self, index, value=None):
        if value is None:
            return self.calibration[index].value
        else:
            pulse = self.calibration[index].pulse
            self.calibration[index] = Pair(pulse, value)

    def first(self, pair=None):
        if pair is None:
            return self.calibration[0]
        else:
            self.calibration[0] = Pair._make(pair)

    def first_pulse(self, pulse=None):
        if pulse is None:
            return self.calibration[0].pulse
        else:
            value = self.calibration[0].value
            self.calibration[0] = Pair(pulse, value)

    def first_value(self, value=None):
        if value is None:
            return self.calibration[0].value
        else:
            pulse = self.calibration[0].pulse
            self.calibration[0] = Pair(pulse, value)

    def last(self, pair=None):
        if pair is None:
            return self.calibration[-1]
        else:
            self.calibration[-1] = Pair._make(pair)

    def last_pulse(self, pulse=None):
        if pulse is None:
            return self.calibration[-1].pulse
        else:
            value = self.calibration[-1].value
            self.calibration[-1] = Pair(pulse, value)

    def last_value(self, value=None):
        if value is None:
            return self.calibration[-1].value
        else:
            pulse = self.calibration[-1].pulse
            self.calibration[-1] = Pair(pulse, value)

    def has_lower_limit(self):
        return self.limit_lower

    def has_upper_limit(self):
        return self.limit_upper

    def limit_to_calibration(self, lower, upper):
        self.limit_lower = lower
        self.limit_upper = upper

    def value_to_pulse(self, value):
        if len(self.calibration) >= 2:
            last = len(self.calibration) - 1

            value_out = value

            # Is the value below the bottom most calibration pair?
            if value < self.calibration[0].value:
                # Should the value be limited to the calibration or projected below it?
                if self.limit_lower:
                    pulse_out = self.calibration[0].pulse
                    value_out = self.calibration[0].value
                else:
                    pulse_out = map_float(value, self.calibration[0].value, self.calibration[1].value,
                                                 self.calibration[0].pulse, self.calibration[1].pulse)
            # Is the value above the top most calibration pair?
            elif value > self.calibration[last].value:
                # Should the value be limited to the calibration or projected above it?
                if self.limit_upper:
                    pulse_out = self.calibration[last].pulse
                    value_out = self.calibration[last].value
                else:
                    pulse_out = map_float(value, self.calibration[last - 1].value, self.calibration[last].value,
                                                 self.calibration[last - 1].pulse, self.calibration[last].pulse)
            else:
                # The value must between two calibration pairs, so iterate through them to find which ones
                for i in range(last):
                    if value <= self.calibration[i + 1].value:
                        pulse_out = map_float(value, self.calibration[i].value, self.calibration[i + 1].value,
                                                     self.calibration[i].pulse, self.calibration[i + 1].pulse)
                        break  # No need to continue checking so break out of the loop

            # Clamp the pulse between the hard limits
            if pulse_out < self.LOWER_HARD_LIMIT or pulse_out > self.UPPER_HARD_LIMIT:
                pulse_out = clamp(pulse_out, self.LOWER_HARD_LIMIT, self.UPPER_HARD_LIMIT)

                # Is the pulse below the bottom most calibration pair?
                if pulse_out < self.calibration[0].pulse:
                    value_out = map_float(pulse_out, self.calibration[0].pulse, self.calibration[1].pulse,
                                                     self.calibration[0].value, self.calibration[1].value)

                # Is the pulse above the top most calibration pair?
                elif pulse_out > self.calibration[last].pulse:
                    value_out = map_float(pulse_out, self.calibration[last - 1].pulse, self.calibration[last].pulse,
                                                     self.calibration[last - 1].value, self.calibration[last].value)

                else:
                    # The pulse must between two calibration pairs, so iterate through them to find which ones
                    for i in range(last):
                        if pulse_out <= self.calibration[i + 1].pulse:
                            value_out = map_float(pulse_out, self.calibration[i].pulse, self.calibration[i + 1].pulse,
                                                             self.calibration[i].value, self.calibration[i + 1].value)
                            break  # No need to continue checking so break out of the loop

            return Pair(pulse_out, value_out)

        return None

    def pulse_to_value(self, pulse):
        if len(self.calibration) >= 2:
            last = len(self.calibration) - 1

            # Clamp the pulse between the hard limits
            pulse_out = clamp(pulse, self.LOWER_HARD_LIMIT, self.UPPER_HARD_LIMIT)

            # Is the pulse below the bottom most calibration pair?
            if pulse_out < self.calibration[0].pulse:
                # Should the pulse be limited to the calibration or projected below it?
                if self.limit_lower:
                    value_out = self.calibration[0].value
                    pulse_out = self.calibration[0].pulse
                else:
                    value_out = map_float(pulse, self.calibration[0].pulse, self.calibration[1].pulse,
                                                 self.calibration[0].value, self.calibration[1].value)

            # Is the pulse above the top most calibration pair?
            elif pulse > self.calibration[last].pulse:
                # Should the pulse be limited to the calibration or projected above it?
                if self.limit_upper:
                    value_out = self.calibration[last].value
                    pulse_out = self.calibration[last].pulse
                else:
                    value_out = map_float(pulse, self.calibration[last - 1].pulse, self.calibration[last].pulse,
                                                 self.calibration[last - 1].value, self.calibration[last].value)
            else:
                # The pulse must between two calibration pairs, so iterate through them to find which ones
                for i in range(last):
                    if pulse <= self.calibration[i + 1].pulse:
                        value_out = map_float(pulse, self.calibration[i].pulse, self.calibration[i + 1].pulse,
                                                     self.calibration[i].value, self.calibration[i + 1].value)
                        break  # No need to continue checking so break out of the loop

            return Pair(pulse_out, value_out)

        return None


class ServoState():
    DEFAULT_FREQUENCY = 50.0    # The standard servo update rate
    MIN_FREQUENCY = 10.0        # Lowest achievable with hardware PWM with good resolution
    MAX_FREQUENCY = 350.0       # Highest nice value that still allows the full uS pulse range
                                # Some servos are rated for 333Hz for instance
    ZERO_PERCENT = 0.0
    ONEHUNDRED_PERCENT = 1.0

    MIN_VALID_PULSE = 1.0

    def __init__(self, calibration=ANGULAR):
        self.servo_value = 0.0
        self.last_enabled_pulse = 0.0
        self.enabled = False

        if isinstance(calibration, Calibration):
            from copy import deepcopy
            self.calib = deepcopy(calibration)
        elif isinstance(calibration, int):
            self.calib = Calibration(calibration)
        else:
            raise TypeError("cannot convert object to an integer or a Calibration class instance")

    def enable_with_return(self):
        # Has the servo not had a pulse value set before being enabled?
        if self.last_enabled_pulse < self.MIN_VALID_PULSE:
            # Set the servo to its middle
            return self.to_mid_with_return()
        return self.__enable_with_return()

    def disable_with_return(self):
        self.enabled = False
        return 0.0  # A zero pulse

    def is_enabled(self):
        return self.enabled

    def __enable_with_return(self):  # Internal version of enable without convenient initialisation to the middle
        self.enabled = True
        return self.last_enabled_pulse

    def get_pulse(self):
        return self.last_enabled_pulse

    def set_pulse_with_return(self, pulse):
        if pulse >= self.MIN_VALID_PULSE:
            out = self.calib.pulse_to_value(pulse)
            if out is not None:
                self.servo_value = out.value
                self.last_enabled_pulse = out.pulse
                return self.__enable_with_return()

        return self.disable_with_return()

    def get_value(self):
        return self.servo_value

    def set_value_with_return(self, value):
        out = self.calib.value_to_pulse(value)
        if out is not None:
            self.last_enabled_pulse = out.pulse
            self.servo_value = out.value
            return self.__enable_with_return()

        return self.disable_with_return()

    def get_min_value(self):
        value = 0.0
        if self.calib.size() > 0:
            value = self.calib.first_value()
        return value

    def get_mid_value(self):
        value = 0.0
        if self.calib.size() > 0:
            first = self.calib.first_value()
            last = self.calib.last_value()
            value = (first + last) / 2.0
        return value

    def get_max_value(self):
        value = 0.0
        if self.calib.size() > 0:
            value = self.calib.last_value()
        return value

    def to_min_with_return(self):
        return self.set_value_with_return(self.get_min_value())

    def to_mid_with_return(self):
        return self.set_value_with_return(self.get_mid_value())

    def to_max_with_return(self):
        return self.set_value_with_return(self.get_max_value())

    def to_percent_with_return(self, input, in_min=ZERO_PERCENT, in_max=ONEHUNDRED_PERCENT, value_min=None, value_max=None):
        if value_min is None:
            value_min = self.get_min_value()

        if value_max is None:
            value_max = self.get_max_value()

        value = map_float(input, in_min, in_max, value_min, value_max)
        return self.set_value_with_return(value)

    def get_calibration(self):
        from copy import deepcopy
        return deepcopy(self.calib)

    def set_calibration(self, calibration):
        if not isinstance(calibration, Calibration):
            raise TypeError("cannot convert object to a Calibration class instance")

        from copy import deepcopy
        self.calib = deepcopy(calibration)

    def pulse_to_level(pulse, resolution, freq):
        level = 0
        if pulse >= ServoState.MIN_VALID_PULSE:
            level = int((pulse * resolution * freq) / 1000000)
        return level


class Servo():
    def __apply_pulse(self, pulse, load, wait_for_load):
        self.ioe.output(self.pin, ServoState.pulse_to_level(pulse, self.pwm_period, self.pwm_frequency), load, wait_for_load)

    def __init__(self, ioe, pin, calibration=ANGULAR, freq=ServoState.DEFAULT_FREQUENCY):
        self.ioe = ioe
        self.pin = pin
        self.state = ServoState(calibration)
        self.pwm_frequency = freq

        self.pin_mod = self.ioe.get_pwm_module(pin)
        self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_mod, load=False)

        ioe.set_mode(pin, io.PWM)
        ioe.output(pin, 0, load=True)

    def enable(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.enable_with_return(), load, wait_for_load)

    def disable(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.disable_with_return(), load, wait_for_load)

    def is_enabled(self):
        return self.state.is_enabled()

    def pulse(self, pulse=None, load=True, wait_for_load=False):
        if pulse is None:
            return self.state.get_pulse()
        else:
            self.__apply_pulse(self.state.set_pulse_with_return(pulse), load, wait_for_load)

    def value(self, value=None, load=True, wait_for_load=False):
        if value is None:
            return self.state.get_value()
        else:
            self.__apply_pulse(self.state.set_value_with_return(value), load, wait_for_load)

    def frequency(self, freq=None, load=True, wait_for_load=False):
        if freq is None:
            return self.pwm_frequency
        else:
            if (freq >= ServoState.MIN_FREQUENCY) and (freq <= ServoState.MAX_FREQUENCY):
                self.pwm_period = self.ioe.set_pwm_frequency(self.pwm_frequency, self.pin_p_mod, load=False)
                if state.is_enabled():
                    self.__apply_pulse(self.state.get_deadzoned_duty(), self.motor_mode, load, wait_for_load)
                elif load:
                    self.ioe.pwm_load(self.pin_p_mod)
            else:
                raise ValueError(f"freq out of range. Expected {ServoState.MIN_FREQUENCY}Hz to {ServoState.MAX_FREQUENCY}Hz")

    def min_value(self):
        return self.state.get_min_value()

    def mid_value(self):
        return self.state.get_mid_value()

    def max_value(self):
        return self.state.get_max_value()

    def to_min(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_min_with_return(), load, wait_for_load)

    def to_mid(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_mid_with_return(), load, wait_for_load)

    def to_max(self, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_max_with_return(), load, wait_for_load)

    def to_percent(self, input, in_min=ServoState.ZERO_PERCENT, in_max=ServoState.ONEHUNDRED_PERCENT, value_min=None, value_max=None, load=True, wait_for_load=False):
        self.__apply_pulse(self.state.to_percent_with_return(input, in_min, in_max, value_min, value_max), load, wait_for_load)

    def calibration(self, calibration=None):
        if calibration is None:
            return self.state.get_calibration()
        else:
            self.state.set_calibration(calibration)


MMME_CPR = 12
ROTARY_CPR = 24


Capture = namedtuple("Capture", ["count",
                                 "delta",
                                 "frequency",
                                 "revolutions",
                                 "degrees",
                                 "radians",
                                 "revolutions_delta",
                                 "degrees_delta",
                                 "radians_delta",
                                 "revolutions_per_second",
                                 "revolutions_per_minute",
                                 "degrees_per_second",
                                 "radians_per_second"])


class Encoder():
    def __init__(self, ioe, channel, pins, common_pin=None, direction=NORMAL_DIR, counts_per_rev=ROTARY_CPR, count_microsteps=False):
        self.ioe = ioe
        self.channel = channel
        self.enc_direction = direction
        self.enc_counts_per_rev = counts_per_rev

        if not isinstance(pins, list) and not isinstance(pins, tuple):
            raise TypeError("cannot convert object to a list or tuple of pins")

        if len(pins) != 2:
            raise TypeError("list or tuple must only contain two integers")

        self.local_count = 0
        self.step = 0
        self.turn = 0
        self.last_raw_count = 0
        self.last_delta_count = 0
        self.last_capture_count = 0

        self.ioe.setup_rotary_encoder(channel, pins[0], pins[1], pin_c=common_pin, count_microsteps=count_microsteps)

    def __take_reading(self):
        # Read the current count
        raw_count = self.ioe.read_rotary_encoder(self.channel)
        raw_change = raw_count - self.last_raw_count
        self.last_raw_count = raw_count

        # Invert the change
        if self.enc_direction == REVERSED_DIR:
            raw_change = 0 - raw_change

        self.local_count += raw_change

        if raw_change > 0:
            self.step += raw_change
            while self.step > self.enc_counts_per_rev:
                self.step -= self.enc_counts_per_rev
                self.turn += 1

        elif raw_change < 0:
            self.step -= raw_change
            while self.step < 0:
                self.step += self.enc_counts_per_rev
                self.turn -= 1

    def count(self):
        self.__take_reading()
        return self.local_count

    def delta(self):
        self.__take_reading()

        # Determine the change in counts since the last time this function was performed
        change = self.local_count - self.last_delta_count
        self.last_delta_count = self.local_count

        return change

    def zero(self):
        self.ioe.clear_rotary_encoder(self.channel)
        self.local_count = 0
        self.step = 0
        self.turn = 0
        self.last_raw_count = 0
        self.last_delta_count = 0
        self.last_capture_count = 0

    def step(self):
        self.__take_reading()
        return self.step

    def turn(self):
        self.__take_reading()
        return self.turn

    def revolutions(self):
        return self.count() / self.enc_counts_per_rev

    def degrees(self):
        return self.revolutions() * 360.0

    def radians(self):
        return self.revolutions() * math.pi * 2.0

    def direction(self, direction=None):
        if direction is None:
            return self.enc_direction
        else:
            if direction not in (NORMAL_DIR, REVERSED_DIR):
                raise ValueError("direction out of range. Expected NORMAL_DIR (0) or REVERSED_DIR (1)")
            self.enc_direction = direction

    def counts_per_rev(self, counts_per_rev=None):
        if counts_per_rev is None:
            return self.enc_counts_per_rev
        else:
            if counts_per_rev < sys.float_info.epsilon:
                raise ValueError("counts_per_rev out of range. Expected greater than 0.0")
            self.enc_counts_per_rev = counts_per_rev

    def capture(self, sample_rate):
        self.__take_reading()

        # Determine the change in counts since the last capture was taken
        change = self.local_count - self.last_capture_count
        self.last_capture_count = self.local_count

        if sample_rate < sys.float_info.epsilon:
            raise ValueError("sample_rate out of range. Expected greater than 0.0")

        frequency = change / sample_rate
        revolutions = self.local_count / self.enc_counts_per_rev
        revolutions_delta = change / self.enc_counts_per_rev
        revolutions_per_second = frequency / self.enc_counts_per_rev

        return Capture(count=self.local_count,
                       delta=change,
                       frequency=frequency,
                       revolutions=revolutions,
                       degrees=revolutions * 360.0,
                       radians=revolutions * math.pi * 2.0,
                       revolutions_delta=revolutions_delta,
                       degrees_delta=revolutions_delta * 360.0,
                       radians_delta=revolutions_delta * math.pi * 2.0,
                       revolutions_per_second=revolutions_per_second,
                       revolutions_per_minute=revolutions_per_second * 60.0,
                       degrees_per_second=revolutions_per_second * 360.0,
                       radians_per_second=revolutions_per_second * math.pi * 2.0)


class DummyPlasma():
    def __init__(self):
        pass

    def set_rgb(self, index, r, g, b, show=True):
        pass

    def set_hsv(self, index, hue, sat=1.0, val=1.0, show=True):
        pass

    def get(self, index):
        pass

    def clear(self, show=True):
        pass

    def show(self): 
        pass


class Plasma():
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
        90, 91, 93, 94, 95, 96, 98, 99, 100, 102, 103, 104, 106, 107, 109, 110,
        111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 128, 129, 131, 132, 134,
        135, 137, 138, 140, 142, 143, 145, 146, 148, 150, 151, 153, 155, 157, 158, 160,
        162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 179, 181, 183, 185, 187, 189,
        191, 193, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220,
        222, 224, 227, 229, 231, 233, 235, 237, 239, 241, 244, 246, 248, 250, 252, 255]
    
    def __init__(self, num_leds, pin):
        # Setup the PixelStrip object to use with Inventor's LEDs
        self.leds = PixelStrip(num_leds, pin, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL, self.LED_GAMMA)
        try:
            # Attempt to initialise the library
            self.leds.begin()
            self.leds.show()
        except:
            raise RuntimeError(LED_INIT_FAILED) from None
    
    def set_rgb(self, index, r, g, b, show=True):
        if index < 0 or index >= self.leds.numPixels():
            raise ValueError("index out of range. Expected 0 to NUM_LEDs - 1");

        self.leds.setPixelColor(index, Color(r, g, b))
        
        if show:
            self.leds.show()
    
    def __hsv_to_rgb(h, s, v):
        if s == 0.0:
            return v, v, v
        
        i = int(h * 6.0)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i = i % 6
        if i == 0:
            return v, t, p
        if i == 1:
            return q, v, p
        if i ==2:
            return p, v, t
        if i == 3:
            return p, q, v
        if i == 4:
            return t, p, v
        if i == 5:
            return v, p, q
    
    def set_hsv(self, index, hue, sat=1.0, val=1.0, show=True):
        if index < 0 or index >= self.leds.numPixels():
            raise ValueError("index out of range. Expected 0 to NUM_LEDs - 1");

        r, g, b = Plasma.__hsv_to_rgb(hue, sat, val)
        self.leds.setPixelColor(index, Color(int(r * 255), int(g * 255), int(b * 255)))
        
        if show:
            self.leds.show()

    def get(self, index):
        # return a tuple
        pass
    
    def clear(self, show=True):
        for i in range(self.leds.numPixels()):
            self.leds.setPixelColor(i, Color(0, 0, 0))
            
        if show:
            self.leds.show()
    
    def show(self):
        self.leds.show()
    
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
            self.__ioe = io.SuperIOE(i2c_addr=self.IOE_ADDRESS, perform_reset=True)
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
        if self.leds is not None:
            for i in range(self.leds.numPixels()):
                self.leds.setPixelColor(i, 0)
            self.leds.show()

        if self.motor_a is not None:
            self.motor_a.coast()

        if self.motor_b is not None:
            self.motor_b.coast()

        if self.servo_1 is not None:
            self.servo_1.disable()

        if self.servo_2 is not None:
            self.servo_2.disable()

        if self.servo_3 is not None:
            self.servo_3.disable()

        if self.servo_4 is not None:
            self.servo_4.disable()

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
