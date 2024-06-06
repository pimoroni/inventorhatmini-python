import sys

import mock
import pytest


@pytest.fixture(scope='function', autouse=False)
def gpiod():
    sys.modules['gpiod'] = mock.Mock()
    sys.modules['gpiod.line'] = mock.Mock()
    yield sys.modules['gpiod']
    del sys.modules['gpiod.line']
    del sys.modules['gpiod']


@pytest.fixture(scope='function', autouse=False)
def gpiodevice():
    gpiodevice = mock.Mock()
    gpiodevice.get_pins_for_platform.return_value = [(mock.Mock(), 0), (mock.Mock(), 0)]
    gpiodevice.get_pin.return_value = (mock.Mock(), 0)

    sys.modules['gpiodevice'] = gpiodevice
    yield gpiodevice
    del sys.modules['gpiodevice']


@pytest.fixture(scope='function', autouse=False)
def ioexpander():
    """Mock ioexpander module."""
    io_expander = mock.MagicMock()
    sys.modules['ioexpander'] = io_expander
    sys.modules['ioexpander.motor'] = io_expander.motor
    sys.modules['ioexpander.servo'] = io_expander.servo
    sys.modules['ioexpander.encoder'] = io_expander.encoder
    sys.modules['ioexpander.common'] = io_expander.common
    yield io_expander
    del sys.modules['ioexpander']


@pytest.fixture(scope='function', autouse=False)
def rpi_ws281x():
    """Mock rpi_ws281x module."""
    rpi_ws281x = mock.MagicMock()
    sys.modules['rpi_ws281x'] = rpi_ws281x
    yield rpi_ws281x
    del sys.modules['rpi_ws281x']
