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
InventorHATMini(motor_gear_ratio=5-, init_motors=True, init_servos=True, init_leds=True, start_muted=False)
reinit()
switch_pressed()
enable_motors()
disable_motors()
read_voltage()
read_motor_current(motor)
mute_audio()
unmute_audio()
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


#### ADC Constants

* `ADC_1` = `0`
* `ADC_2` = `1`
* `ADC_3` = `2`
* `ADC_4` = `3`


#### LED Constants

* `LED_SERVO_1` = `0`
* `LED_SERVO_2` = `1`
* `LED_SERVO_3` = `2`
* `LED_SERVO_4` = `3`
* `LED_ADC_1` = `4`
* `LED_ADC_2` = `5`
* `LED_ADC_3` = `6`
* `LED_ADC_4` = `7`


#### Count Constants

* `NUM_MOTORS` = `2`
* `NUM_SERVOS` = `4`
* `NUM_ADCS` = `4`
* `NUM_LEDS` = `8`
