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


def floorCheck():  # Checks the colour of the floor
    return line_sensor.value()


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


# Hug Wall Operational Code:

# Moves forward until line is no longer detected
while True:
    if floorCheck() == 0:
        forward(50)
    else:
        stop()
        break

# Captures distance to wall on either side of the robot
setServoAngle(0)
distLeft = distCheck()
setServoAngle(180)
distRight = distCheck()

# Checks which distance was smaller, hugLeft variable is passed to the turn function to determine which direction to turn
if distRight > distLeft > 150:  # This number will need to be playtested
    distWall = distLeft
    hugLeft = True
else:
    distWall = distRight
    hugLeft = False

# Moves forward with slight turning adjustments every 0.2 seconds, should stop if wall distance suddenly doubles (ie, the wall has ended)
while True:
    forward(50)
    time.sleep(0.1)
    if distWall > distCheck():  # Checks the stored distance with the current distance to compare if it has grown or shrunk
        turn(50, not hugLeft, 25)  # Turn right at speed 50, with turning intensity of 25%
        time.sleep(0.1)
    else:
        turn(50, hugLeft, 25)
        time.sleep(0.1)
    if distCheck() > 2*distWall:  # Checks if the distance is suddenly more than double the stored value, which would mean that the wall has ended
        stop()
        break






'''# Competency Task Operational Code:
while True:
    forward(50)
    stopDist(60, "floor")
    if floorCheck() != 1:
        backward(50)
        time.sleep(1)
        spin(90, "clock")
    else:
        break'''

'''# Sweep servo between 0 and 180 degrees, in increments of 1 degree
while True:
    for pos in range(0, 180, 1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle
    for pos in range(180, 0, -1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle'''
