import time
from ultrasonic import sonic
from motor import Motor
from machine import Pin, PWM
from machine import time_pulse_us


def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    # ...range
    pwm.duty_u16(position)  # set duty cycle

#initilise light sensor
line_sensor = Pin(26, Pin.IN)

# initialise motors
motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)

# assign friendly values to ultrasonic pins, initialise ultrasonic sensor
TRIG = 3
ECHO = 2
ultrasonic_sensor = sonic(TRIG, ECHO)

# initialise servo
pwm = PWM(Pin(15))
pwm.freq(50)

count = 0

#move forward into hits black object underneath
while True:
    print(line_sensor.value())

    motor_left.set_forwards()
    motor_right.set_forwards()
    sensor_value = line_sensor.value()
    if sensor_value == 0:
        motor_right.duty(40)
        motor_left.duty(40)
    else:
        motor_right.duty(0)
        motor_left.duty(0)
        count += 1
    if count > 100:
        motor_left.set_backwards()
        motor_right.set_backwards()
        motor_right.duty(40)
        motor_left.duty(40)
        time.sleep(2)
        motor_right.duty(0)
        motor_left.duty(0)
        break

#spin 180

motor_left.set_backwards()
motor_right.set_forwards()
motor_right.duty(35)
motor_left.duty(35)
time.sleep(5)
motor_right.duty(0)
motor_left.duty(0)


# These statements make the code more readable.
# Instead of a pin number 2 or 3 we can now write "TRIG" or "ECHO"

# Ultrasonic sensor move forward and stop at 60mm
while True:
    dist = ultrasonic_sensor.distance_mm()
    if dist < 60:
        motor_left.duty(0)
        motor_right.duty(0)
        break
    else:
        motor_left.set_forwards()
        motor_right.set_forwards()
        motor_left.duty(40)
        motor_right.duty(40)
        # The code within this if-statement only gets executed
        # if the distance measured is less than 200 mm
        print("Distance = {:6.2f} [mm]".format(dist))
        time.sleep(0.1)

# servo loop code
while True:
    # Sweep between 0 and 180 degrees
    for pos in range(0, 180, 1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 50 ms to reach angle
    for pos in range(180, 0, -1):
        setServoAngle(pos)  # Set servo to desired angle
        time.sleep(0.01)  # Wait 50 ms to reach angle