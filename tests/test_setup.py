def test_setup(GPIO, rpi_ws281x, ioexpander):
    import inventorhatmini
    inventorhatmini.InventorHATMini()
    GPIO.setwarnings.assert_called_once_with(False)
    GPIO.setmode.assert_called_once_with(GPIO.BCM)
