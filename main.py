import time
from ultrasonic import sonic
from motor import Motor
from machine import Pin, PWM
from machine import time_pulse_us
from encoder import Encoder

# Initialisation Statements:

# Line Sensors
line_sensorL = Pin(28, Pin.IN)
line_sensorM = Pin(26, Pin.IN)
line_sensorR = Pin(27, Pin.IN)

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


def turn(speed=50, left=True, intensity=25):  # Imposes a ratio on the motors, determined by the direction and intensity out of 100
    forward(speed)
    if left:
        motor_left.duty(((100-intensity)/100)*speed)
    else:
        motor_right.duty(((100-intensity)/100)*speed)


def distance(dist=20):  # Used in tandem with forward and backward functions, specifies a distance to travel in cm
    while (enc.get_left() + enc.get_right())/2 < dist:
        {
            print("Oh yeah, how about this for statement has no effect, stupid python")
            # OLED screen can count down distance maybe that would be cool
        }
    enc.clear_count()


def stop():  # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)


def stopDist(dist=60, reason="nothing"):  # Stops the robot a specified distance in mm from an obstacle
    while True:
        measure = ultrasonic_sensor.distance_mm()
        if reason == "floor":
            if floorCheck() == 1:
                stop()
                break
        if measure < dist:
            stop()
            break


def distCheck():  # Checks the distance between the ultrasonic sensor and what it is pointing at
    return ultrasonic_sensor.distance_mm()


def floorCheck(sensor="middle"):  # Checks the colour of the floor. Line sensor being used can be specified using multiple identifiers
    if sensor == "left" or sensor == "l" or sensor == "L":
        return line_sensorL.value()
    elif sensor == "right" or sensor == "r" or sensor == "R":
        return line_sensorR.value()
    else:
        return line_sensorM.value()


def spin(angle=120, direction="clock", speed=40):  # Spins the robot, can select angle, direction and speed
    if direction == "clock":
        motor_left.set_forwards()
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()
    motor_right.duty(speed)
    motor_left.duty(speed)
    time.sleep(angle / 100)
    stop()

# Curved Line Follow Operational Code:
while True:
    forward(40)
    if floorCheck("R") == 1:
        while floorCheck() == 0:
            turn(40, False, 50)
        forward(40)
    if floorCheck("L") == 1:
        while floorCheck() == 0:
            turn(40, True, 50)
        forward(40)


