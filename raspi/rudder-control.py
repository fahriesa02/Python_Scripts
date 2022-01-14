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


def interrupt_function():
    error = rudder_pid.update(yaw)
    yaw.ChangeDutyCycle(error/15)

readings = []
initial_time = time.time
try:
    while True:
        rudder_pid.setPoint(readings)
        time.sleep(10)
# cleanup GPIO
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()