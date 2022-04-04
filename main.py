import time
from ultrasonic import sonic
from motor import Motor
from machine import Pin, PWM
from machine import time_pulse_us

# Initialisation Statements:

# Line Sensor
line_sensor = Pin(26, Pin.IN)

# Motors
motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)

# Ultrasonic Sensor
ultrasonic_sensor = sonic(3, 2)

# Servo
pwm = PWM(Pin(15))
pwm.freq(50)

def setServoAngle(angle): # Changes the angle the front mounted servo is pointing
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    # ...range
    pwm.duty_u16(position)  # set duty cycle #

def forward(speed): # Moves the robot forward at a selected speed
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(speed)
    motor_right.duty(speed)

def backwards(speed): # Moves the robot backward at a selected speed
    motor_left.set_backwards()
    motor_right.set_backwards()
    motor_left.duty(speed)
    motor_right.duty(speed)

def stop(): # Stops the robot moving
    motor_left.duty(0)
    motor_right.duty(0)

def stopDist(distance): # Stops the robot a specified distance in mm from an obstacle
    while True:
        dist = ultrasonic_sensor.distance_mm()
        if floorCheck() == 1:
            stop()
        if dist < distance:
            stop()
            break

def floorCheck(): # Checks the colour of the floor
    return line_sensor.value()

def spinClock(angle): # Spins the robot Clockwise by a specified angle
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_right.duty(40)
    motor_left.duty(40)
    time.sleep(angle/100)
    stop()

def spinCounter(angle): # Spins the robot Counter-Clockwise by a specified angle
    motor_left.set_backwards()
    motor_right.set_forwards()
    motor_right.duty(40)
    motor_left.duty(40)
    time.sleep(angle/100)
    stop()


# Operational Code

# Theoretical solution to the competency task
while True:
    forward(50)
    stopDist(60)
    if floorCheck() == 1:
        break
    else:
        backwards(50)
        time.sleep(1)
        spinClock(90)


'''
# Move the robot forward until Line Sensor detects black
count = 0
while True:
    print(line_sensor.value())
    sensor_value = line_sensor.value()
    if sensor_value == 0:
        forward(40)
    else:
        stop()
        count += 1
    if count > 100:
        backwards(40)
        time.sleep(2)
        stop()
        break

spinClock(180)

# Move forward until the Ultrasonic Sensor is less than 60mm from a wall
while True:
    dist = ultrasonic_sensor.distance_mm()
    if dist < 60:
        stop()
        break
    else:
        forward(40)
        print("Distance = {:6.2f} [mm]".format(dist))
        time.sleep(0.1)

# Sweep servo between 0 and 180 degrees, in increments of 1 degree
while True:
    for pos in range(0, 180, 1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle
    for pos in range(180, 0, -1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 10 ms to reach angle
'''