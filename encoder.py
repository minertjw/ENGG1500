# Code adapted from:
# https://github.com/peterhinch/micropython-samples/blob/master/encoders/encoder_portable.py
# Encoder Support: this version should be portable between MicroPython platforms
# Thanks to Evan Widloski for the adaptation to the machine module


class Encoder(object):
    def __init__(self, pin_left, pin_right):
        from machine import Pin
        self.forward = True
        self.pin_left = Pin(pin_left, Pin.IN)
        self.pin_right = Pin(pin_right, Pin.IN)
        self._count_left = 0
        self._count_right = 0
        self.left_interrupt = self.pin_left.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_callback)
        self.right_interrupt = self.pin_right.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_callback)

    # Callback functions are used to `handle' desired operations immediately,
    # whenever a rising/falling edge triggers an `interrupt'
    def left_callback(self, line):
        # Add or subtract 1 from the left encoder count based on direction
        self._count_left += 1 if self.forward else -1

    def right_callback(self, line):
        # Add or subtract 1 from the right encoder count based on direction
        self._count_right += 1 if self.forward else -1

    def get_left(self):
        return self._count_left

    def get_right(self):
        return self._count_right

    def clear_count(self):
        # Reset the counts of both encoders to zero
        self._count_left = 0
        self._count_right = 0