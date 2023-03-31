def test_setup(atexit, GPIO, rpi_ws281x, ioexpander):
    import inventorhatmini
    board = inventorhatmini.InventorHATMini()
    GPIO.setwarnings.assert_called_once_with(False)
    GPIO.setmode.assert_called_once_with(GPIO.BCM)
    atexit.register.assert_called_once_with(board._InventorHATMini__cleanup)