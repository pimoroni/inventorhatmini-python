NO_IOE_MSG = "Could not communicate with the IO Expander. Check the HAT is properly attached and that the I2C pins of the Raspberry Pi are not being used for another function."
NO_I2C = "The I2C interface needed for communication with the IO Expander is disabled. To enable it either run `sudo raspi-config nonint do_i2c 0` in the terminal, or use the `Raspberry Pi Configuration` UI accessible under `Preferences`."
LED_INIT_FAILED = """Failed to initialise the RGB LEDs because higher privileges are needed to control the necessary hardware of the Raspberry Pi.
To overcome this, either:
  - [if using the terminal] run your code with `sudo` in front.
  - [if using Thonny] close it, then open it via the terminal using `sudo thonny`. Then run your code as normal.
  - disable LED initialisation by including the parameter `init_leds=False` when creating your InventorHATMini object. This will prevent the LEDs from being used later on in your code."""
