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


def setServoAngle(angle=100):  # Changes the angle the front mounted servo is pointing. Angle is inverted, (Left is 167, right is 25)
    position = int(8000*(angle/180)+1000)  # Convert angle into [1000, 9000]
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
    motor_left.duty(int(speed))
    motor_right.duty(int(speed))
def backward(speed=50):  # Moves the robot backward at a selected speed
    motor_left.set_backwards()
    motor_right.set_backwards()
    motor_left.duty(speed)
    motor_right.duty(speed)
def turnLeft(speed=50, intensity=25):  # Imposes a ratio on the motors, determined by the direction and intensity out of 100
    forward(speed)
    motor_left.duty(((100-intensity)/100)*speed)
def turnRight(speed=50, intensity=25):
    forward(speed)
    motor_right.duty(((100 - intensity) / 100) * speed)


def stop():  # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)
    oled.text("Stopped",0,0)
    oled.show()
def stopDist(dist=60): # Stops the robot a specified distance in mm from an obstacle
    forward()
    while dist < ultrasonic_sensor.distance_mm():
        oled.text("Stopping at", 0, 0)
        oled.text(str(dist)+"cm", 0, 8)
        oled.show()
        oled.fill(0)
    clearScreen()
    stop()


def distCheck():  # Checks the distance between the ultrasonic sensor and what it is pointing at
    return ultrasonic_sensor.distance_mm()
def floorCheck():  # Checks the colour of the floor
    return line_sensorM.value()


def spin(angle=120, direction="clock"):  # Spins the robot, can select angle, direction and speed
    if direction == "clock":
        motor_left.set_forwards()
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()
    motor_right.duty(50)
    motor_left.duty(50)
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
def clearScreen():
    oled.fill(0)
    oled.show()

# Operational Code:

stopDist(1000)

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

'''forward()
while True:
    while tickRight() > tickLeft():
        motor_right.duty(int(50*(tickLeft()/tickRight())))
        encOnScreen()
    forward()
    while tickLeft() > tickRight():
        motor_left.duty(int(50*0.8))
        encOnScreen()
    forward()
    encOnScreen()'''

