import sys

import mock
import pytest


@pytest.fixture(scope='function', autouse=False)
def GPIO():
    """Mock RPi.GPIO module."""
    RPi = mock.MagicMock()
    sys.modules['RPi'] = RPi
    sys.modules['RPi.GPIO'] = RPi.GPIO
    yield RPi.GPIO
    del sys.modules['RPi']
    del sys.modules['RPi.GPIO']


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


@pytest.fixture(scope='function', autouse=False)
def atexit():
    """Mock atexit module."""
    atexit = mock.MagicMock()
    sys.modules['atexit'] = atexit
    yield atexit
    del sys.modules['atexit']