# -*- coding: utf-8 -*-
#!/usr/bin/env python

# ========== import ==========
import RPi.GPIO as GPIO
import wiringpi2
import time
import sys
import threading
import numpy as np
from time import sleep

# ========== PIN assign ==========
# moter pins
IN1 = 3
IN2 = 4
IN3 = 17
IN4 = 27

# ========== setup ==========
def setup():
    GPIO.setmode(GPIO.BCM)
    wiringpi2.wiringPiSetupGpio()
    # Moters
    wiringpi2.pinMode(IN1, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN2, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN3, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN4, wiringpi2.GPIO.PWM_OUTPUT)
    # initialize stop
    wiringpi2.softPwmCreate(IN1, 100, 100)
    wiringpi2.softPwmCreate(IN2, 100, 100)
    wiringpi2.softPwmCreate(IN3, 100, 100)
    wiringpi2.softPwmCreate(IN4, 100, 100)

# ========== Func of Moter ==========
# Go forward
def goForward(pwm):
    wiringpi2.softPwmWrite(IN1, pwm)
    wiringpi2.softPwmWrite(IN2, 0)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

# Go back
def goBack(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, 0)
    wiringpi2.softPwmWrite(IN4, pwm)

# clockwise
def clockWise(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

# cclockwise
def cClockWise(pwm):
    wiringpi2.softPwmWrite(IN1, pwm)
    wiringpi2.softPwmWrite(IN2, 0)
    wiringpi2.softPwmWrite(IN3, 0)
    wiringpi2.softPwmWrite(IN4, pwm)

# stop
def stop():
    wiringpi2.softPwmWrite(IN1, 100)
    wiringpi2.softPwmWrite(IN2, 100)
    wiringpi2.softPwmWrite(IN3, 100)
    wiringpi2.softPwmWrite(IN4, 100)
    time.sleep(0.1)

# ========== main ==========
if __name__ == '__main__':
    setup()
    mode = 0
    pwm = 50
    runTime = 5
    try:
        while True:
            error = 0
            mode = input("F:0, B:1, CW, 2, CCW:3...:")
            pwm = input("PWM...:")
            runTime = input("run time...:")
            if mode == 0:
                goForward(pwm)
            elif mode == 1:
                goBack(pwm)
            elif mode == 2:
                clockWise(pwm)
            elif mode == 3:
                cClockWise(pwm)
            else:
                print ("mode error")
                error = 1

            if error == 0:
                time.sleep(runTime)
                stop()

    finally:
        stop()
        GPIO.cleanup()


# end of program
