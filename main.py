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
line_sensorO = Pin(5, Pin.IN)

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
#i2c = I2C(0, scl=Pin(0), sda=Pin(1))
#apds9960 = APDS9960LITE(i2c)
#apds9960.als.enableSensor()
#apds9960.als.elightGain(3)

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
    elif pick == "O":
        return line_sensorO.value()
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
def initialise():
    oled.text("booting up",0,0)
    oled.show()
    time.sleep(0.5)
    oled.fill(0)
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
'''def colourOnScreen():
    oled.text(str(apds9960.als.ambientLightLevel),0,0)
    oled.text(str(apds9960.als.redLightLevel),0,16)
    oled.text(str(apds9960.als.greenLightLevel),0,32)
    oled.text(str(apds9960.als.blueLightLevel),0,48)
    oled.show()
    oled.fill(0)'''

# Locked Operational Code
def lineFollow():
    while True:
        forward(45)
        while floorCheck("L") == 1:
            spin("counter")
        while floorCheck("R") == 1:
            spin("clock")
def module1():
    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    roundabout("left",1)
                    check = False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    if allFloorCheck():
                        roundabout("left",1)
                        check = False
                spin("clock")

def module2():
    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    check = False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    if allFloorCheck():
                        check = False
                spin("clock")

def module3():
    while floorCheck("L") == 1 or floorCheck("R") == 1:
        motor_right.duty(0)
        motor_left.set_forwards()
        motor_left.duty(45)
    module1()

def module4():
    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    check = False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    if allFloorCheck():
                        check = False
                spin("clock")
    while floorCheck("L") == 1 or floorCheck("R") == 1:
        motor_right.duty(0)
        motor_left.set_forwards()
        motor_left.duty(45)

def module5():
    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    roundabout("right",1)
                    check = False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    if allFloorCheck():
                        roundabout("right",1)
                        check = False
                spin("clock")

def module6():
    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                check == False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    check = False
                spin("clock")

    while floorCheck("L") == 1:
        motor_right.duty(0)
        motor_left.set_forwards()
        motor_left.duty(45)

def roundabout(direction="left", exit=1):
    screen("roundabout")

    if direction == "left":
        while floorCheck("L") == 1 or floorCheck("R") == 1:
            motor_left.duty(0)
            motor_right.set_forwards()
            motor_right.duty(45)

        if exit == 2:
            while True:
                forward(40)
                while floorCheck("R") == 1:
                    spin("clock")
                if floorCheck("L") == 1:
                    while floorCheck("L") == 1:
                        forward(40)
                        while floorCheck("R") == 1:
                            spin("clock")
                    break

        while floorCheck("L") == 0:
            forward(40)
            while floorCheck("R") == 1:
                spin("clock")
        while floorCheck("L") == 1 or floorCheck("R") == 1:
            motor_left.duty(0)
            motor_right.set_forwards()
            motor_right.duty(45)

    else:
        while floorCheck("L") == 1 or floorCheck("R") == 1:
            motor_right.duty(0)
            motor_left.set_forwards()
            motor_left.duty(45)

        if exit == 2:
            while True:
                forward(40)
                while floorCheck("L") == 1:
                    spin("counter")
                if floorCheck("R") == 1:
                    while floorCheck("R") == 1:
                        forward(40)
                        while floorCheck("L") == 1:
                            spin("counter")
                    break

        while floorCheck("R") == 0:
            forward(40)
            while floorCheck("L") == 1:
                spin("counter")
        while floorCheck("R") == 1 or floorCheck("L") == 1:
            motor_right.duty(0)
            motor_left.set_forwards()
            motor_left.duty(45)
def roundaboutPlus(direction="left", exit=1):
    screen("roundaboutPlus")

    if direction == "left":
        while floorCheck("L") == 1 or floorCheck("R") == 1:
            motor_left.duty(0)
            motor_right.set_forwards()
            motor_right.duty(45)

        if exit == 2:
            while True:
                forward(40)
                while floorCheck("R") == 1:
                    spin("clock")
                if floorCheck("L") == 1:
                    while floorCheck("L") == 1:
                        forward(40)
                        while floorCheck("R") == 1:
                            spin("clock")
                    break
        while floorCheck("O") == 0:
            print("")
        while floorCheck("O") == 1:
            print("")
        while floorCheck("O") == 0:
            forward(40)
            while floorCheck("R") == 1:
                spin("clock")
        while floorCheck("O") == 1 or floorCheck("R") == 1:
            motor_left.duty(0)
            motor_right.set_forwards()
            motor_right.duty(45)
def module7():
    while floorCheck("L") == 1 or floorCheck("R") == 1:
        motor_right.duty(0)
        motor_left.set_forwards()
        motor_left.duty(45)

    check = True
    while check == True:
        forward(40)
        while floorCheck("L") == 1:
            if floorCheck("R") == 1:
                if allFloorCheck():
                    roundaboutPlus("left", 2)
                    check = False
            spin("counter")
        if check == True:
            while floorCheck("R") == 1:
                if floorCheck("L") == 1:
                    if allFloorCheck():
                        roundaboutPlus("left", 2)
                        check = False
                spin("clock")
def steerServo():
    while True:
        encOnScreen()
        setServoAngle(100+enc.get_right()-enc.get_left())
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

initialise()
module1()
screen("module2")
module2() # Takes risky edge track and stops at first fork
screen("module23456789")
motor_right.duty(0)
motor_left.set_forwards()
motor_left.duty(45)
time.sleep(0.5)
module2()
screen("module3")
module3() # Exits out of second roundabout
module4() # Exits towards second sharp turn
module5() # Exits third roundabout
module6()
module2() # Takes fork before first roundabout again
module7() # Takes the first roundabout again on the second exit
lineFollow()







