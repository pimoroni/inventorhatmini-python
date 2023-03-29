# Inventor HAT Mini Library

This is the library for controlling the [Pimoroni Inventor HAT Mini](https://pimoroni.com/inventorhatmini) for the Raspberry Pi SBC.


## Table of Content
- [Getting Started](#getting-started)
- [Reading the User Button](#reading-the-user-button)
- [Motors](#motors)
- [Function Reference](#function-reference)
- [Constants Reference](#constants-reference)
  - [Motor Constants](#motor-constants)
  - [Servo Constants](#servo-constants)
  - [ADC Constants](#adc-constants)
  - [LED Constants](#led-constants)
  - [Count Constants](#count-constants)


## Getting Started

To start coding your Inventor HAT Mini, you will need to add the following lines to the start of your code file.
```python
from inventorhatmini import InventorHATMini
board = InventorHATMini()
```
This will create a `InventorHATMini` class called `board` that will be used in the rest of the examples going forward.


## Reading the User Button

Inventor HAT Mini has a single button on top, labelled User. This can be read using the `switch_pressed()` function.

For example, to read the A button you would write:

```python
state = board.switch_pressed()
```


## Motors

Inventor HAT Mini two motor outputs with independent control, enabling [differential steering](https://en.wikipedia.org/wiki/Differential_steering), whereby the speed of one motor can be controlled separately of the other.

There are several ways these motors can be commanded from code: TODO


## Function Reference

Here is the complete list of functions available on the `InventorHATMini` class:

```python
InventorHATMini(motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False)
reinit()
switch_pressed()
enable_motors()
disable_motors()
read_voltage()
read_motor_current(motor)
mute_audio()
unmute_audio()
gpio_mode(gpio)
gpio_mode(gpio, mode)
gpio_value(gpio)
gpio_value(gpio, value)
servo_pin_mode(servo)
servo_pin_mode(servo, mode)
servo_pin_value(servo)
servo_pin_value(servo, mode, load=True, wait_for_load=False)
servo_pin_load(servo, wait_for_load=True)
servo_pin_frequency(servo, frequency, load=True, wait_for_load=True)
encoder_from_gpios(channel, gpio_a, gpio_b, direction=NORMAL_DIR, counts_per_rev=ROTARY_CPR, count_microsteps=False)
motor_from_servo_pins(servo_p, servo_n, direction=NORMAL_DIR, speed_scale=DEFAULT_SPEED_SCALE, zeropoint=DEFAULT_ZEROPOINT, deadzone=DEFAULT_DEADZONE, freq=DEFAULT_FREQUENCY, mode=DEFAULT_DECAY_MODE)
activate_watchdog()
deactivate_watchdog()
feed_watchdog()
watchdog_timeout_occurred()
clear_watchdog_timeout()
is_watchdog_active()
set_watchdog_control(divider)
```


## Constants Reference

Here is the complete list of constants on the `inventorhatmini` module:

#### Motor Constants

* `MOTOR_A` = `0`
* `MOTOR_B` = `1`


#### Servo Constants

* `SERVO_1` = `0`
* `SERVO_2` = `1`
* `SERVO_3` = `2`
* `SERVO_4` = `3`


#### GPIO Constants

* `GPIO_1` = `0`
* `GPIO_2` = `1`
* `GPIO_3` = `2`
* `GPIO_4` = `3`


#### LED Constants

* `LED_SERVO_1` = `0`
* `LED_SERVO_2` = `1`
* `LED_SERVO_3` = `2`
* `LED_SERVO_4` = `3`
* `LED_GPIO_1` = `4`
* `LED_GPIO_2` = `5`
* `LED_GPIO_3` = `6`
* `LED_GPIO_4` = `7`


#### Count Constants

* `NUM_MOTORS` = `2`
* `NUM_SERVOS` = `4`
* `NUM_GPIOS` = `4`
* `NUM_LEDS` = `8`
