import machine
from machine import Pin
import time

class sonic:
    """
    A ``sonic`` object is used to control a HCSR04 ultrasonic sensor.
    """

    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=4000*2/(340.29*1e-3)):
        """
        - trigger_pin: Output pin to send pulses
        - echo_pin: Input pin listens for reflection to estimate time of flight.
        - echo_timeout_us: Timeout in microseconds to listen to echo pin.
        By default the timeout is set to the max range limit of the sensor, 4m
        """
        # Initialise constants
        self.echo_timeout_us = int(echo_timeout_us)
        self.speed_sound = 340.29  # m/s

        # Initialise the TRIG output pin
        self.trigger = Pin(trigger_pin, mode=Pin.OUT)
        self.trigger.value(0)

        # Initialise the ECHO input pin
        self.echo = Pin(echo_pin, mode=Pin.IN)

    def distance_mm(self):
        """
        Estimate distance to obstacle in front of the ultrasonic sensor in mm.
        Sends a 10us pulse to 'trigger' pin and listens on 'echo' pin.
        We use the method `machine.time_pulse_us()` to count the microseconds
        passed before the echo is received.
        The time of flight is used to calculate the estimated distance.
        """
        # Send a 10us HIGH pulse to trigger the ultrasonic burst
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        # Read length of time pulse
        duration = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
        # Calculate the distance in mm, based on the delay before we hear
        # an echo and the constant speed of sound.
        # Note: duration is halved as the audio wave must travel there and back.
        mm = 0.5 * duration * 1e-6 * self.speed_sound * 1e3
        return mm