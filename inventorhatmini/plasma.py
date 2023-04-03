from rpi_ws281x import Color, PixelStrip

from inventorhatmini.errors import LED_INIT_FAILED


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
            raise ValueError("index out of range. Expected 0 to NUM_LEDs - 1")

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
        if i == 2:
            return p, v, t
        if i == 3:
            return p, q, v
        if i == 4:
            return t, p, v
        if i == 5:
            return v, p, q

    def set_hsv(self, index, hue, sat=1.0, val=1.0, show=True):
        if index < 0 or index >= self.leds.numPixels():
            raise ValueError("index out of range. Expected 0 to NUM_LEDs - 1")

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
