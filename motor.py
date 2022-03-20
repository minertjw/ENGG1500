class Motor(object):
    """ A ``motor`` object is used to control a DC motor using the L298N dual H-bridge.
        The Motor class has methods to change the direction of the motor by setting
        the IN1/IN2 pins:
        - ``Motor.set_forwards()``
        - ``Motor.set_backwards()``
         and a method to change the duty cycle (0-100%) of the PWM output:
        - ``Motor.duty(pwm)``
        """

    def __init__(self, side, in1_pin, in2_pin, en_pin):


        """
                :rtype: object
                :type side: str
                :type in1_pin: str
                :type in2_pin: str
                :type en_pin: str
                The arguments are:
                - ``side``  should be the strings: "left" or "right". Used to specify motor.
                - ``in1_pin`` should be a valid Pin name string.
                - ``in2_pin`` should be a valid Pin name string
                - ``en_pin`` should be the strings: 6 or 7
                Usage Model::
                # Initialise motor instance
                left_motor = Motor("left", 8, 9, 7)
                # Set motor direction to forwards, and duty cycle to 70%
                left_motor.set_forwards()
                left_motor.duty(70)
                """

        from machine import Pin
        from machine import PWM
        # Declare a variable to keep track of which motor is left/right

        self.side = side
        print("Initialising", self.side, "motor...")
        # Declare GPIO pins for direction IN1 and IN2 pins
        self.IN1 = Pin(in1_pin, Pin.OUT)
        self.IN2 = Pin(in2_pin, Pin.OUT)
        # Declare GPIO pins for PWM EN pin
        self.EN = PWM(Pin(en_pin))

        print("Motor", self.side, "initialised!")

    def duty(self, pwm):
        pwm_16 = 655*pwm
        self.EN.duty_u16(pwm_16)
        """ The duty cycle must be declared as a uint 16 variable between 0 and 65,535.
            Therefore, to set the duty cycle to 100%, we set it to 65535.
            """

    def set_forwards(self):
        if self.side == "left":
            self.IN1.on()
            self.IN2.off()
        else:  # only other motor is the right motor
            self.IN1.off()
            self.IN2.on()

    def set_backwards(self):
        if self.side == "left":
            self.IN1.off()
            self.IN2.on()
        else:
            self.IN1.on()
            self.IN2.off()
#this comment totally breaks the entire code
