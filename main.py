import time
from ultrasonic import sonic
from motor import Motor
from machine import Pin, PWM
from machine import time_pulse_us
from encoder import Encoder

# Initialisation Statements:

# Line Sensor
line_sensor = Pin(26, Pin.IN)

# Motors
motor_left = Motor("left", "8", "9", 6)
motor_right = Motor("right", "10", "11", 7)

# Ultrasonic Sensor
ultrasonic_sensor = sonic(3, 2)

# Servo
pwm = PWM(Pin(15))
pwm.freq(50)

# Encoders
ENC_L = 18  # pin2y34ry8
ENC_R = 19
enc = Encoder(ENC_L, ENC_R)


def setServoAngle(angle=90):  # Changes the angle the front mounted servo is pointing
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    # ...range
    pwm.duty_u16(position)  # set duty cycle #


def forward(speed=50):  # Moves the robot forward at a selected speed
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(speed)
    motor_right.duty(speed)


def backward(speed=50):  # Moves the robot backward at a selected speed
    motor_left.set_backwards()
    motor_right.set_backwards()
    motor_left.duty(speed)
    motor_right.duty(speed)


def distance(dist=50):  # Used in tandem with forward and backward functions, specifies a distance to travel in cm
    while (enc.get_left() + enc.get_right())/2 < (dist / 20):
        {
            print("Oh yeah, how about this for statement has no effect, stupid python")
            # OLED screen can count down distance maybe that would be cool
        }
    enc.clear_count()


def stop():  # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)


def stopDist(dist=60):  # Stops the robot a specified distance in mm from an obstacle
    while True:
        measure = ultrasonic_sensor.distance_mm()
        if floorCheck() == 1:
            stop()
        if measure < dist:
            stop()
            break


def floorCheck():  # Checks the colour of the floor
    return line_sensor.value()


def spinClock(angle=120):  # Spins the robot Clockwise by a specified angle
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_right.duty(40)
    motor_left.duty(40)
    time.sleep(angle / 100)
    stop()


def spinCounter(angle=120):  # Spins the robot Counter-Clockwise by a specified angle
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_right.duty(40)
    motor_left.duty(40)
    time.sleep(angle / 100)
    stop()


# Main Operational Code:
while True:
    forward(50)
    stopDist(60)
    if floorCheck() == 1:
        break
    else:
        backward(50)
        time.sleep(1)
        spinClock(90)

'''# Sweep servo between 0 and 180 degrees, in increments of 1 degree
while True:
    for pos in range(0, 180, 1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle
    for pos in range(180, 0, -1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle'''
