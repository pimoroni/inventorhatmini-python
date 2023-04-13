# Inventor HAT Mini - Library Reference <!-- omit in toc -->

This is the library reference for the [Pimoroni Inventor HAT Mini](https://pimoroni.com/inventorhatmini), a versatile motor, servo and audio driver HAT for Raspberry Pi.


## Table of Content <!-- omit in toc -->
- [Getting Started](#getting-started)
- [Reading the User Button](#reading-the-user-button)
- [Reading Voltage](#reading-voltage)
- [GPIOs](#gpios)
  - [Setup](#setup)
  - [Mode](#mode)
  - [As Input or ADC](#as-input-or-adc)
  - [As Output](#as-output)
  - [As Encoder](#as-encoder)
- [Motors and Encoders](#motors-and-encoders)
  - [Enabling and Disabling](#enabling-and-disabling)
  - [Reading Motor Current](#reading-motor-current)
  - [Skipping Initialisation](#skipping-initialisation)
- [Servos](#servos)
  - [Skipping Initialisation](#skipping-initialisation-1)
  - [As GPIOs](#as-gpios)
  - [As PWM](#as-pwm)
    - [Delayed Loading](#delayed-loading)
    - [Limitations](#limitations)
  - [As Motor](#as-motor)
- [RGB LEDs](#rgb-leds)
- [Audio](#audio)
  - [Muting and Unmuting](#muting-and-unmuting)
- [Watchdog Timer](#watchdog-timer)
  - [Configuring](#configuring)
  - [Activating](#activating)
  - [Feeding](#feeding)
  - [Deactivating](#deactivating)
  - [Handling a Time-Out](#handling-a-time-out)
- [Function Reference](#function-reference)
- [Constants Reference](#constants-reference)
    - [Motor Constants](#motor-constants)
    - [Servo Constants](#servo-constants)
    - [GPIO Constants](#gpio-constants)
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

Inventor HAT Mini has a single button on top, labelled **User**. This can be read using the `switch_pressed()` function. To read this button you would write:

```python
state = board.switch_pressed()
```

## Reading Voltage

Inventor HAT Mini can measure the voltage its motors and servos are being powered by. On an unmodified board this is also the voltage of the Raspberry Pi's 5V output.

The voltage can be measured by calling `.read_voltage()`:

```python
voltage = board.read_voltage()
```

## GPIOs

There are four GPIO pins on Inventor HAT Mini. These can be used as digital outputs, digital inputs, and analog inputs. There is also a special mode where a pair of GPIO pins can be used as rotary encoder inputs.

### Setup

To start using a GPIO pin, first import one of the handy constants used to reference them (see [GPIO Constants](#gpio-constants)). For example, to use the first GPIO pin:

```python
from inventorhatmini import GPIO_1
```

Alternatively, if you wish to use all four GPIO pins, `NUM_GPIOS` can be imported for use within a loop, e.g. `for i in range(NUM_GPIOS)`.

Then you need to import the constants for the pin mode to use. These are on the `ioexpander` module that Inventor HAT Mini is based on.

```python
# For input
from ioexpander import IN  # or IN_PU of a pull-up is wanted

# For output
from ioexpander import OUT

# For ADC
from ioexpander import ADC
```

### Mode

With the intended constants imported, the mode of a GPIO pin can be set by calling `.gpio_pin_mode(gpio, mode)`:

```python
board.gpio_pin_mode(GPIO_1, <IN or IN_PU or OUT or ADC>)
```

It is also possible to read the current mode of a GPIO pin by calling `.gpio_pin_mode(gpio)`:

```python
mode = board.gpio_pin_mode(GPIO_1)
```

### As Input or ADC

The current value of an GPIO pin in input or ADC mode can be read by calling `.gpio_pin_value(gpio)`:

```python
value = board.gpio_pin_value(GPIO_1)
```

If the mode is digital, the value will either be `0` or `1`.
If the mode is analog, the value will be a voltage from `0.0` to `3.3`.


### As Output

The current value of a GPIO pin in output mode can be set by calling `.gpio_pin_value(gpio, value)`:

```python
board.gpio_pin_value(GPIO_1, value)
```

The expected value is either `0` or `1`, or `True` or `False`.


### As Encoder

In addition to the input and output functions of GPIO pins, they can also be paired together to read rotary encoders like those used on dials or for motor feedback. Up to two encoders are supported in this way. This is in addtion to the two encoders already in use by the motor connectors.

To start using GPIO pins for an encoder, first import their handy constants:

```python
from inventorhatmini import GPIO_1, GPIO_2
```

Then on the `InventorHATMini` class call the function `.encoder_from_gpio_pins(channel, gpio_a, gpio_b)`, where `channel` is the encoder channel to use from `3` to `4` (`1` and `2` are in use by the motor connectors), and `gpio_a` and `gpio_b` are the two pins to use. There are also optional parameters for `direction`, `counts_per_rev`, and `count_microsteps`.

```python
CHANNEL = 3
encoder = board.encoder_from_gpio_pins(CHANNEL, GPIO_1, GPIO_2)
```

This function returns a new `Encoder` object that uses the specified pins. To use this object, refer to the [`Encoder` object](https://github.com/pimoroni/ioe-python/blob/master/docs/encoder.md) reference.


## Motors and Encoders

Inventor HAT Mini two motor outputs with independent control and feedback, enabling [differential steering](https://en.wikipedia.org/wiki/Differential_steering), whereby the speed of one motor can be controlled separately of the other.

Motors, and their encoders, are setup by default when creating a new `InventorHATMini` object.

To start using a motor, first import one of the handy constants used to reference them (see [Motor Constants](#motor-constants)). For example, to use the first motor:

```python
from inventorhatmini import MOTOR_A
```

From there the motor and encoder can be accessed by the following `board` variables:

```python
# Access a motor and its encoder
motor = board.motors[MOTOR_A]
encoder = board.encoders[MOTOR_A]
```

These give you access to objects that handle the motor and encoder functionality. For details of what can be done with them, check out their respective documentation pages:
* `board.motors`: [`Motor` object](https://github.com/pimoroni/ioe-python/blob/master/docs/motor.md)
* `board.encoders`:  [`Encoder` object](https://github.com/pimoroni/ioe-python/blob/master/docs/encoder.md)

### Enabling and Disabling

The motor outputs of Inventor HAT Mini can be enabled and disabled at any time by calling `.enable_motors()` and `.disable_motors()`. These functions call both motor's respective `.enable()` and `.disable()` functions.


### Reading Motor Current

The current draw of each motor output can be measured by calling `.read_motor_current(motor)`, where motor is either `MOTOR_A` or `MOTOR_B`:

```python
current = board.read_motor_current(MOTOR_A)
```

The returned value is the current in amps (A).


### Skipping Initialisation

For some projects the automatic initialisation of motors and their encoders may not be wanted. For example if you instead wanted to use the motor outputs to drive another kind of inductive load.

To allow for this, the parameter `init_motors=False` can be added when creating the `InventorHATMini` object.

```python
board = InventorHATMini(init_motors=False)
```

This leaves `board.motors` and `board.encoders` set to `None`, letting you to use those board pins for any other purpose.

The io expander pins available after this are:

* `board.IOE_MOTOR_A_PINS` = `(20, 19)`
* `board.IOE_MOTOR_B_PINS` = `(16, 15)`
* `board.IOE_ENCODER_A_PINS` = `(3, 4)`
* `board.IOE_ENCODER_B_PINS` = `(26, 1)`


## Servos

Inventor HAT Mini has four servo outputs. These are setup by default when creating a new `InventorHATMini` object.

To start using a servo, first import one of the handy constants used to reference them (see [Servo Constants](#servo-constants)). For example, to use the first servo:

```python
from inventorhatmini import SERVO_1
```

From there the servo can be accessed by the following `board` variables:

```python
# Access a servo
servo = board.servos[SERVO_1]
```

This gives you access to an object that handle the servo functionality. For details of what can be done with it, check out its documentation page:
* `board.servos`: [`Servo` object](https://github.com/pimoroni/ioe-python/blob/master/docs/servo.md)


### Skipping Initialisation

For some projects the automatic initialisation of servos may not be wanted. For example if you instead wanted to use the servo outputs to drive another kind of PWM device.

To allow for this, the parameter `init_servos=False` can be added when creating the `InventorHATMini` object.

```python
board = InventorHATMini(init_servos=False)
```

This leaves `board.servos` set to `None`, letting you to use those board pins for any other purpose.

The io expander pins available after this are:

* `board.IOE_SERVO_PINS` = `(23, 24, 25, 22)`


### As GPIOs

With servos uninitialised, the servo pins can be interacted with like regular GPIO pins (see [GPIOs](#gpios)), using the functions `.servo_pin_mode()` and `.servo_pin_value()`. The one exception to this is that these pins are not ADC capable.

To use a servo pin as an input:

```python
# Initialise InventorHATMini without servos
board = InventorHATMini(init_servos=False)

# Setup the servo pin as an input
board.servo_pin_mode(SERVO_1, IN)  # or IN_PU of a pull-up is wanted

# Read the value of the servo pin
value = board.servo_pin_value(SERVO_1)
```

To use a servo pin as an output:

```python
# Initialise InventorHATMini without servos
board = InventorHATMini(init_servos=False)

# Setup the servo pin as an output
board.servo_pin_mode(SERVO_1, OUT)

# Read the servo pin to high
board.servo_pin_value(i, True)
```


### As PWM

To support their use with servos, the servo pins accept the `PWM` pin mode. This constant can be imported from the `ioexpander` module, and passed into the `.servo_pin_mode()` function.

The frequency of the PWM signal can then be configured by calling `.servo_pin_frequency()`, which accepts a servo index and the frequency (in Hz).

Finally, the duty cycle of the PWM signal can be set by calling `.servo_pin_value()` and providing it with a value between `0.0` and `1.0`.

Below is an example of setting a servo pin to output a 25KHz signal with a 50% duty cycle:

```python
from ioexpander import PWM
from inventorhatmini import InventorHATMini, SERVO_1

# Initialise InventorHATMini without servos
board = InventorHATMini(init_servos=False)

# Setup the servo pin as a PWM output
board.servo_pin_mode(SERVO_1, PWM)

# Set the servo pin's frequency to 25KHz
board.servo_pin_frequency(SERVO_1, 25000)

# Output a 50% duty cycle square wave
board.servo_pin_value(SERVO_1, 0.5)
```


#### Delayed Loading

By default, changes to a servo pin's frequency or value are applied immediately. However, sometimes this may not be wanted, and instead you want all pins to receive updated parameters at the same time, regardless of how long the code ran that calculated the update.

For this purpose, `.servo_pin_frequency()` and `.servo_pin_value()` include an optional parameter `load`, which by default is `True`. To avoid this "loading" include `load=False` in the relevant function calls. Then either the last call can include `load=True`, or a specific call to `.servo_pin_load()` can be made.

In addition, any function that performs a load, including the `.servo_pin_load()` function, can be made to wait until the new PWM value has been sent out of the pins. By default this is disabled, but can be enabled by including `wait_for_load=True` in the relevant function calls.


#### Limitations

Inventor HAT Mini has limitations on which PWM signals can be controlled independently. For the servo pins, this means that Servo 1 and 2 must share the same frequency, and similarly for Servo 3 and 4.

Keep this in mind if changing the frequency of one, as the other will not automatically know about the change, resulting in unexpected duty cycle outputs.


### As Motor

As an extension of the general PWM support of the servo pins, they can also be paired together to output signals for controlling DC motors (via a standalone driver IC) in the same way as Inventor HAT Mini's onboard motor connectors are driven. Up to two motors are supported.

To start using servo pins for a motor, first import their handy constants:

```python
from inventorhatmini import SERVO_1, SERVO_2
```

Then on the `InventorHATMini` class call the function `.motor_from_servo_pins(servo_p, servo_n)`, where `servo_p` and `servo_n` are the positive and negative output pins to use. There are also optional parameters for `direction`, `speed_scale`, `zeropoint`, `deadzone`, `freq`, and `mode`:

```python
motor = board.motor_from_servo_pins(SERVO_1, SERVO_2)
```

This function returns a new `Motor` object that uses the specified pins. To use this object, refer to the [`Motor` object](https://github.com/pimoroni/ioe-python/blob/master/docs/motor.md) reference.


## RGB LEDs

There are 8 independently addressable RGB LEDs on Inventor HAT Mini, positioned nearby the four servo and four GPIO pins. These LEDs are automatically initialised when creating a new `InventorHATMini` object and can be accessed using the following variable:

```python
# Access the RGB LEDs
leds = board.leds
```

For details of what can be done with this object, check out its documentation page:
* `board.leds`: [`Plasma` object](docs/plasma.md)


## Audio

Inventor HAT Mini features an onboard I2S audio amplifier, letting it play any sounds created by your Raspberry Pi, from any source, be it a web browser, music player, or from code. 

There are many ways sound can be created from code, some examples of which can be found in [examples/audio](/examples/audio/).


### Muting and Unmuting

The audio output of Inventor HAT Mini can be muted at any time by calling `.mute_audio()`. Similarly, the audio output can be unmuted by calling `.unmute_audio()`.

Additionally, when creating a new `InventorHATMini` object, the starting audio output state can be specified by providing either `start_muted=True` or `start_muted=False`. The default is `False`.


## Watchdog Timer

Inventor HAT Mini features an onboard [Watchdog timer](https://en.wikipedia.org/wiki/Watchdog_timer), that can be used to turn off motor and servo outputs in the event of an issue with your code that stops it communicating with the board.

This is especially useful if you are creating a driving robot, to stop it from driving off uncontrolled!

To see an example of the Watchdog timer in action, visit [examples/extras/watchdog_reset.py](examples/extras/watchdog_reset.py).


### Configuring

To start using the watchdog timer, first it's duration needs to be set. This is done using the `.set_watchdog_control()` function, which accepts one of the following multiple of 2 clock divider values:

| Clock Divider   | Time-Out |
|-----------------|----------|
| 1               | 6.40ms   |
| 2               | -        |
| 4               | 25.6ms   |
| 8               | 51.2ms   |
| 16              | 102.4ms  |
| 32              | 204.8ms  |
| 64              | 409.6ms  |
| 128             | 819.2ms  |
| 256             | 1.638s   |

For example, here is how to set the watchdog to the longest time-out:

```python
board.set_watchdog_control(256)
```


### Activating

Once configured, the watchdog can be activated by calling:

```python
board.activate_watchdog()
```

There is now limited time (1.638 seconds in this example) to communicate with the Inventor HAT Mini before the watchdog timer reaches zero and resets the board!

The active state of the watchdog timer can be checked by calling `.is_watchdog_active()`.


### Feeding

To prevent the watchdog time from reaching zero, your code needs to "feed" the watchdog regularly. This is done by calling:

```python
board.feed_watchdog()
```

This will return the watchdog timer back up to the original value it was at when activated.


### Deactivating

If your code reaches a point where you know it will take longer than the time set, or you otherwise no longer need the watchdog, it can be deactivated by calling:

```python
board.deactivate_watchdog()
```


### Handling a Time-Out

If the watchdog does reset your Inventor HAT Mini whilst your code is running, it is possible to check for this by calling `.watchdog_timeout_occurred()`, then handle the event. The `InventorHATMini` class has a `.reinit()` function intended for this exact case:

```python
if board.watchdog_timeout_occurred():
  board.reinit()
```

There is also `.clear_watchdog_timeout()` that should be called if you wanted to configure Inventor HAT Mini without calling `.reinit()`.


## Function Reference

Here is the complete list of functions available on the `InventorHATMini` class:

```python
InventorHATMini(address=0x17, motor_gear_ratio=50, init_motors=True, init_servos=True, init_leds=True, start_muted=False)
reinit()
switch_pressed()
enable_motors()
disable_motors()
read_voltage()
read_motor_current(motor)
mute_audio()
unmute_audio()
gpio_pin_mode(gpio)
gpio_pin_mode(gpio, mode)
gpio_pin_value(gpio)
gpio_pin_value(gpio, value)
servo_pin_mode(servo)
servo_pin_mode(servo, mode)
servo_pin_value(servo)
servo_pin_value(servo, mode, load=True, wait_for_load=False)
servo_pin_load(servo, wait_for_load=True)
servo_pin_frequency(servo, frequency, load=True, wait_for_load=True)
encoder_from_gpio_pins(channel, gpio_a, gpio_b, direction=NORMAL_DIR, counts_per_rev=ROTARY_CPR, count_microsteps=False)
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
