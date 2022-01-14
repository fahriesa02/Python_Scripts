#!/usr/bin/env


# paket yang harus di import
from _typeshed import Self
import RPi.GPIO as GPIO
import pid
import time

# variabel yang dipakai untuk sistem
rudder_pid = pid.PID(1.0, 1.0, 1.0)

# setting GPIO pin
servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

yaw = GPIO.PWM(servoPIN, 50)
yaw.start()

# kalkulasi PID sebelum masuk ke proses
def interrupt_function():
    error = rudder_pid.update(yaw)
    yaw.ChangeDutyCycle(error/15)

# looping sistem
readings = [] # input pixel image processing
initial_time = time.time
try:
    while True:
        rudder_pid.setPoint(readings)
        time.sleep(0.25)

# cleanup GPIO
except KeyboardInterrupt:
    yaw.stop()
    GPIO.cleanup()