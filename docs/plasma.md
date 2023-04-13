# Inventor HAT Mini - Plasma <!-- omit in toc -->

The Plasma class provides a means for controlling the onboard RGB LEDs of
[Inventor HAT Mini](https://shop.pimoroni.com/products/inventor-hat-mini). It shares familiarity with Inventor 2040 W's [Plasma library](https://github.com/pimoroni/pimoroni-pico/blob/main/micropython/modules/plasma/README.md), making it easy to port LED code between the two platforms.

Under the hood Plasma uses the [rpi_ws281x package](https://pypi.org/project/rpi-ws281x/), which interacts with the necessary hardware via a `PixelStrip` class.

- [Getting Started](#getting-started)
- [Set a LED](#set-a-led)
  - [RGB](#rgb)
  - [HSV](#hsv)
- [Clear all LEDs](#clear-all-leds)
- [Delayed Showing](#delayed-showing)
- [Function Reference](#function-reference)
- [DummyPlasma](#dummyplasma)

## Getting Started

`InventorHATMini` automatically creates a `Plasma` instance, accessable by `board.leds`. Instructions for creating a standalone instance are provided below for completeness.

To construct a new `Plasma` instance, specify the number of LEDs, and the GPIO pin to use. See [GPIO Usage](https://github.com/rpi-ws281x/rpi-ws281x-python/tree/master/library#gpio-usage) for details on what pins are supported.

```python
from inventorhatmini.plasma import Plasma

LEDS = 30
LED_DAT = 12

led_strip = Plasma(LEDS, LED_DAT)
```

## Set a LED

You can set the colour of a LED in either the RGB colourspace, or HSV (Hue, Saturation, Value). HSV is useful for creating rainbow patterns.

### RGB

Set the first LED - `0` - to Purple `255, 0, 255`:

```python
led_strip.set_rgb(0, 255, 0, 255)
```

### HSV

Set the first LED - `0` - to Red `0.0`:

```python
led_strip.set_hsv(0, 0.0, 1.0, 1.0)
```

## Clear all LEDs

To turn off all the LEDs, the function `.clear()` can be called. This simply goes through each LED and sets its RGB colour to black, making them emit no light.

This function is useful to have at the end of your code to turn the lights off, otherwise they will continue to show the last colours they were given.

## Delayed Showing

The `Plasma` class automatically applies changes to the LEDs immediately. However, sometimes this may not be wanted, and instead you want all LEDs to receive updated colours at the same time, regardless of how long the code ran that calculated them.

For this purpose, the `.set_rgb()`, `.set_hsv()`, and `.clear()` functions include an optional parameter `show`, which by default is `True`. To avoid this "showing" include `show=False` in the relevant function calls. Then either the last call can include `show=True`, or a specific call to `.show()` can be made.

## Function Reference

Here is the complete list of functions available on the `Plasma` class:
```python
Plasma(num_leds, pin)
set_rgb(index, r, g, b, show=True)
set_hsv(index, h, s=1.0, v=1.0, show=True)
get(index)
clear(show=True)
show()
```

### DummyPlasma

In addition to the `Plasma`class there is a `DummyPlasma` class. This class has all the same functions as the regular class, but does not interact with the LEDs.

It was created to deal with the case where code may have been written that uses the LEDs but cannot be run with root user privilege (aka `sudo`). In this case, providing `init_leds=False` to `Inventor HAT Mini` causes it to create a `DummyPlasma` class for its `board.leds` variable, allowing such code to be run without further modification.
