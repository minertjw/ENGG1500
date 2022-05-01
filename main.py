import time
from machine import Pin, PWM, I2C, time_pulse_us
from motor import Motor
from ultrasonic import sonic
from encoder import Encoder
from ssd1306 import SSD1306_I2C

# Initialisation Statements:

# Line Sensors
line_sensorM = Pin(26, Pin.IN)
line_sensorL = Pin(27, Pin.IN)
line_sensorR = Pin(28, Pin.IN)

# Motors
motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)

# Ultrasonic Distance Sensor
TRIG = 3
ECHO = 2
ultrasonic_sensor = sonic(TRIG, ECHO)

# Servo
pwm = PWM(Pin(15))
pwm.freq(50)

# Encoders
ENC_L = 18  # pin2y34ry8
ENC_R = 19
enc = Encoder(ENC_L, ENC_R)

# OLED Screen
i2c = I2C(0, sda=Pin(12), scl=Pin(13))
oled = SSD1306_I2C(128, 64, i2c)


def setServoAngle(
        angle=100):  # Changes the angle the front mounted servo is pointing. Angle is inverted, (Left is 167, right is 25)
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    pwm.duty_u16(position)  # set duty cycle #
def lookForward():
    setServoAngle(100)
def lookLeft():
    setServoAngle(167)
def lookRight():
    setServoAngle(25)


def forward(speed=50):  # Moves the robot forward at a selected speed
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(int(speed*0.9))
    motor_right.duty(int(speed))
def backward(speed=50):  # Moves the robot backward at a selected speed
    motor_left.set_backwards()
    motor_right.set_backwards()
    motor_left.duty(int(speed))
    motor_right.duty(int(speed))


def turnLeft(speed=50,
             intensity=25):  # Imposes a ratio on the motors, determined by the direction and intensity out of 100
    forward(speed)
    motor_left.duty(((100 - intensity) / 100) * speed)
def turnRight(speed=50, intensity=25):
    forward(speed)
    motor_right.duty(((100 - intensity) / 100) * speed)


def stop():  # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)
    oled.text("Stopped", 0, 32)
    oled.show()
def stopDist(dist=60):  # Stops the robot a specified distance in mm from an obstacle
    forward()
    while dist < ultrasonic_sensor.distance_mm():
        oled.text("Stopping at", 0, 0)
        oled.text(str(dist) + "cm", 0, 8)
        oled.show()
        oled.fill(0)
    clearScreen()
    stop()


def distCheck():  # Checks the distance between the ultrasonic sensor and what it is pointing at
    return ultrasonic_sensor.distance_mm()
def floorCheck(pick="M"):  # Checks the colour of the floor
    if pick == "R":
        return line_sensorR.value()
    elif pick == "L":
        return line_sensorL.value()
    else:
        return line_sensorM.value()


def spin(angle=120, direction="clock"):  # Spins the robot, can select angle, direction and speed
    if direction == "clock":
        motor_left.set_forwards()
        motor_right.set_backwards()
        motor_right.duty(70)
        motor_left.duty(40)
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()
        motor_right.duty(40)
        motor_left.duty(70)
    time.sleep(angle / 100)
    stop()


def tickRight():
    return enc.get_right()
def tickLeft():
    return enc.get_left()


def encOnScreen():
    oled.text(str(enc.get_left()), 10, 0)
    oled.text(str(enc.get_right()), 42, 0)
    oled.show()
    oled.fill(0)
def lineOnScreen():
    oled.text(str(floorCheck("R")), 10, 0)
    oled.text(str(floorCheck()), 42, 0)
    oled.text(str(floorCheck("L")), 74, 0)
    oled.show()
    oled.fill(0)
def clearScreen():
    oled.fill(0)
    oled.show()


# Line Follow Code:
time.sleep(2)
while True:
    forward(45)
    while floorCheck("L") == 1:
        spin(10, "counter")
    while floorCheck("R") == 1:
        spin(10, "clock")


# Rotate Servo with ratio of wheel rotation:
'''while True:
    encOnScreen()
    setServoAngle(100+enc.get_right()-enc.get_left())
    if False:
        break'''

# Roundabout code
'''while True:
    forward(40)                     # this is just curved line follow
    if floorCheck("R") == 1:        #
        while floorCheck() == 0:    #
            turnRight(40)           #
        forward(40)                 #
    if floorCheck("L") == 1:        #
        while floorCheck() == 0:    #
            turnLeft(40)            #
        forward(40)                 #
    i = 0                           # original code starts here
    while i <= 4:                   # i is so it only runs 4 time andd the moves on
        if floorCheck("L") == 1 and floorCheck("R") == 1:  # i want it so it only runs when both floor check L&R are =1
            while floorCheck() == 1:
                turnLeft(40)
            forward(40)
        i += 1
        if floorCheck("L") == 1:     #this is to exit roundabout
            while floorCheck() == 1:
                turnLeft(40)
            forward(40)
    if i > 4:                           # if it loops 4 time its gonna enter the roundabout to the right
        if floorCheck("R") == 1 and floorCheck("L") == 1:
            while floorCheck() == 1:
                turnRight(40)
            forward(40)
        i += 1
        if floorCheck("R") == 1:
            while floorCheck() == 1:
                turnRight(40)
            forward(40)'''

# Star Wars on screen
'''setServoAngle()
oled.text("A long time ago", 0, 16)
oled.text(" in a galaxy far ", 0, 24)
oled.text("far away...", 0, 32)
oled.show()
time.sleep(5)
oled.fill(0)
oled.show()
time.sleep(3)
for i in range(64,-10,-1):
    oled.text("STAR WARS", 25, i)
    oled.show()
    time.sleep(0.03)
    oled.fill(0)'''

