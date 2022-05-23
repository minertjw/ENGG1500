import time
from machine import Pin, PWM, I2C, time_pulse_us
from motor import Motor
from ultrasonic import sonic
from encoder import Encoder
from ssd1306 import SSD1306_I2C
from APDS9960LITE import APDS9960LITE

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

# Colour Sensor
i2c = I2C(0, scl=Pin(20), sda=Pin(16))
apds9960 = APDS9960LITE(i2c)
apds9960.als.enableSensor()
apds9960.als.elightGain=3

# Servo Controls
def setServoAngle(angle=100):  # Changes the angle the front mounted servo is pointing. Angle is inverted, (Left is 167, right is 25)
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    pwm.duty_u16(position)  # set duty cycle #
def lookForward():
    setServoAngle(100)
def lookLeft():
    setServoAngle(30)
def lookRight():
    setServoAngle(167)

# Main Movement Controls
def forward(speed=50):  # Moves the robot forward at a selected speed
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(int(speed*0.9))
    motor_right.duty(int(speed))
def backward(speed=50):  # Moves the robot backward at a selected speed
    motor_left.set_backwards()
    motor_right.set_backwards()
    motor_left.duty(int(speed*0.9))
    motor_right.duty(int(speed))

# Turning Controls (Kinda Useless Currently)
def turnLeft(speed=50, intensity=25):  # Imposes a ratio on the motors, determined by the direction and intensity out of 100
    forward(speed)
    if intensity > 100:
        motor_left.set_backwards()
        motor_left.duty(int(((intensity-100))) * speed)
    else:
        motor_left.duty(int(((100 - intensity) / 100)) * speed)
def turnRight(speed=50, intensity=25):
    forward(speed)
    motor_right.duty(int(((100 - intensity) / 100)) * speed)

# Stop Options
def stop():  # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)
def stopDist(dist=60):  # Stops the robot a specified distance in mm from an obstacle
    forward()
    while dist < ultrasonic_sensor.distance_mm():
        oled.text("Stopping at", 0, 0)
        oled.text(str(dist) + "cm", 0, 8)
        oled.show()
        oled.fill(0)
    clearScreen()
    stop()

# Sensor Data
def distCheck():  # Checks the distance between the ultrasonic sensor and what it is pointing at
    return ultrasonic_sensor.distance_mm()
def floorCheck(pick="M"):  # Checks the colour of the floor
    if pick == "R":
        return line_sensorR.value()
    elif pick == "L":
        return line_sensorL.value()
    else:
        return line_sensorM.value()
def allFloorCheck():
    if line_sensorR.value() + line_sensorL.value() + line_sensorM.value() == 3:
        return True
    else:
        return False

# Spin Controls
def spin(direction="clock"):  # Spins the robot, can select angle, direction and speed
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

# Easy Encoder Gets
def tickRight():
    return enc.get_right()
def tickLeft():
    return enc.get_left()

# Screen Controls
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
def screen(string, x=0, y=0):
    oled.text(str(string),x,y)
    oled.show()
    oled.fill(0)
def clearScreen():
    oled.fill(0)
    oled.show()

# Locked Operational Code
def lineFollow():
    while True:
        forward(45)
        while floorCheck("L") == 1:
            spin("counter")
        while floorCheck("R") == 1:
            spin("clock")
def stateMachineTest():
    while True:
        forward(45)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    stop()
                    screen("Middle IR = 1")
                    time.sleep(999)
                stop()
                screen("Middle IR = 0")
                time.sleep(999)
            spin("counter")

        while floorCheck("R") == 1:
            if floorCheck("L") == 1:
                if allFloorCheck():
                    stop()
                    screen("Middle IR = 1")
                    time.sleep(999)
                stop()
                screen("Middle IR = 0")
                time.sleep(999)
            spin("clock")


def steerServo():
    while True:
        encOnScreen()
        setServoAngle(100+enc.get_right()-enc.get_left())
def distract():
    while True:
        forward(45)
        if floorCheck("L") == 1:
            forward(45)
            tickRight()
            tickLeft()
            if floorCheck() == 0:
                backward(45)
                tickRight()
                tickLeft()
                spin("counter")
                time.sleep()
        elif floorCheck("R") == 1:
            forward(45)
            tickRight()
            tickLeft()
            if floorCheck() == 0:
                backward(45)
                tickRight()
                tickLeft()
                spin("clock")
def deadEnd():
    setServoAngle()
    while True:
        forward(45)
        while floorCheck("L") == 1:
            spin("counter")
        while floorCheck("R") == 1:
            spin("clock")
        if distCheck() < 70: # has to see the wall before it sees the line, tune distance
            stop()
            break
    while True:
        forward(45)
        if floorCheck("R") == 1:
            spin("clock")
        lineFollow()
def wallFollow():
    count = 0
    while True:
        time.sleep(0.3)
        stop()
        oled.fill(0)
        lookLeft()
        distLeft = distCheck()
        time.sleep(0.3)
        lookRight()
        distRight = distCheck()
        oled.text(str(distLeft), 0, 0)
        oled.text(str(distRight),0,16)
        oled.text(str(count),0,48)
        count+=1
        if distRight > distLeft:
            turnRight(60, ((distRight - distLeft)))
        elif distLeft > distRight:
            turnLeft(60, ((distLeft - distRight)))

# Miscellaneous
def starWars():
    setServoAngle()
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
        oled.fill(0)

# Operational Code:

# lineFollow is infinitely looped, roundabout code below will not run unless lineFollow() is removed:
# Below is modified line follow code that includes a check within both while loops for the presence of a roundabout

stateMachineTest()
'''stop()
screen("alllines")
time.sleep(500)
check = False
while True:
    screen("forward")
    forward(45)
    if check:
        break
    while floorCheck("L") == 1:
        screen("spin 10 counter")
        spin("counter")
        if floorCheck("R") == 1:
            while floorCheck("R") == 1:
                screen("L turn left 45, 100")
                turnLeft(45, 100)
            check = True
            break

    while floorCheck("R") == 1:
        screen("spin 10 clock")
        spin("clock")
        if floorCheck("L") == 1:
            while floorCheck("R") == 1:
                screen("R turn left 45 100")
                turnLeft(45, 140)
            check = True
            break

# Now for some classic line follow code, but where it only sees using the right sensor.
# If it sees the left edge, then it knows there is an exit, and attempts to take it.
screen("wall follow mode")
stop()
time.sleep(6788)
while True:
    forward(45)
    while floorCheck("R") == 1:
        spin(10, "clock")
    if floorCheck("L") == 1:
        while floorCheck("M") == 1:
            turnLeft(40,100)
        break

lineFollow()'''