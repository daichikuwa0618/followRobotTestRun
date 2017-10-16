#!/usr/bin/env/ python
# -*- coding: utf-8 -*-

# Created Daichi Hayashi 2017/09/24
# From Yonago Institute of Technology

# +++++ Discription +++++
#   escape from wall whenever the robot moveServo
#   and, follow to anyone who is moving

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
IN1          = 3
IN2          = 4
IN3          = 14
IN4          = 15
# AD-Converter
spi_clk      = 11
spi_mosi     = 10
spi_miso     = 9
spi_ss       = 8
# Sensor sensor
sensorSwitch = 2
# signals from Inoue Lab. S0:MSB, S10:LSB
S0           = 7  # Lower right 8 Signal Flag
S1           = 5  # Lower left 6 識別(High:Distance, Low:Angle)
S2           = 6  # Lower left 5
S3           = 13 # Lower left 4
S4           = 19 # Lower left 3
S5           = 26 # Lower left 2
S6           = 12 # Lower right 5
S7           = 16 # Lower right 3
S8           = 20 # Lower right 2
S9           = 21 # Lower right 1
S10          = 25 # if this is High -> Not moving
paraList = [S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10] # parallel list

# ========== variables ==========
# runTime:[sec]
runTime = 60
# waitTime
waitTime = 3
# info from Inoue Lab.
result = [0, 0]

error = 0
# variables of sensors
sensorList = [0, 0, 0]
# Threshold of sensor
nearValue = 2300

# ========== constants ==========
# constants relates time contrbuting to distance to observer
goForwardConst = 20
goBackConst = 20
turnConst = 80

# parallel communication
bitToRange = 360/256  # bit情報から角度に変換
rangeTrans = 2        # bit情報から距離に変換(2bit左シフト)

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
    # AD-converter
    GPIO.setup(spi_mosi, GPIO.OUT)
    GPIO.setup(spi_miso, GPIO.IN)
    GPIO.setup(spi_clk, GPIO.OUT)
    GPIO.setup(spi_ss, GPIO.OUT)
    GPIO.setup(sensorSwitch,GPIO.OUT)
    # parallel communication
    GPIO.setup(S0, GPIO.OUT)
    GPIO.setup(paraList[1:11], GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.output(S0, 0) # initial output

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


# ========== Func For test ==========
def inputTest():
    num1 = input("距離を入力:")
    num2 = input("角度を入力:")

    result = [int(num1), float(num2)]

    return result

# ========== Func parallel ==========
def signalInput():
    signal = ""
    for cnt in range(8):
        if GPIO.input(chan_list[cnt+2]):
            signal += "1"
        else:
            signal += "0"
    return signal

# ========== Func return results ==========
# return distance[cm], angle[deg]
def signalGet():
    distance = ""
    angle = ""
    print ("initial signal 1")
    GPIO.output(S0, GPIO.HIGH)
    time.sleep(0.05)

    go = 0
    print ("wait flag :HIGH ...")
    while go == 0:
        mae = GPIO.input(S1)
        time.sleep(0.1)
        if mae == 1 and mae == GPIO.input(S1):
            go = 1
    distance = signalInput()

    go = 0
    print ("wait flag :LOW ...")
    while go == 0:
        mae = GPIO.input(S1)
        time.sleep(0.1)
        if mae == 0 and mae == GPIO.input(S1):
            go = 1
    angle = signalInput()

    GPIO.output(S0, GPIO.LOW)
    time.sleep(0.05)

    if GPIO.input(S10):
        print ("No Motion. Try Again...")
    else:
        print (distance, angle) # bit列で表示
        result = [int(distance,2) * bitToRange, float(int(angle,2)) * rangeTrans]

        return result

# ========== readSensor ==========
# this func also judge whether avoid or not
def readSensor():
    resetNum = 1000 # 補正用数値
    global error, sensorList
    for ch in range(3):
        GPIO.output(spi_ss, 0)
        GPIO.output(spi_clk, 0)
        GPIO.output(spi_mosi, 0)
        GPIO.output(spi_clk, 1)
        GPIO.output(spi_clk, 0)

        cmd = (ch | 0x18) << 3
        for i in range(5):
            if (cmd & 0x80):
                GPIO.output(spi_mosi, 1)
            else:
                GPIO.output(spi_mosi, 0)
            cmd <<= 1
            GPIO.output(spi_clk, 1)
            GPIO.output(spi_clk, 0)
        GPIO.output(spi_clk, 1)
        GPIO.output(spi_clk, 0)
        GPIO.output(spi_clk, 1)
        GPIO.output(spi_clk, 0)
        # channel designation

        # variable containing values of each sensor
        value = 0
        for i in range(12):
            value <<= 1
            GPIO.output(spi_clk, 1)
            if (GPIO.input(spi_miso)):
                value |= 0x1
            GPIO.output(spi_clk, 0)
        GPIO.output(spi_ss, 1)

        sensorList[ch] = value
        if sensorList[ch] < resetNum:
            sensorList[ch] = 0

    # judge whether avoid or not
    if np.sqrt((sensorList[1] ** 2) + ((sensorList[0] + sensorList[2]) ** 2)) > nearValue:
        error = 1
    else:
        error = 0

# ========== Func avoid from wall ==========
def avoidWall(value0, value1, value2):
    # mapping of sensor
    # ch0:Right ch1:Front ch2:Left
    # 角度から秒数に変換する定数
    turnTime = 90 # deviding const translating from deg to sec
    stop()
    # Go back a little bit
    goBack(50)
    time.sleep(1)
    stop()

    # avoiding angle
    degree = 180
    # ZeroDevisionError
    if value1 == 0:
        value1 = 1
    # 180degからどれだけ回転するのか
    cwDeg = np.rad2deg(np.arctan2(value0, value1))
    ccwDeg = np.rad2deg(np.arctan2(value2, value1))
    # 角度の補正
    degree = degree + cwDeg - ccwDeg

    # cw回転
    if degree <= 180:
        clockWise(80)
        print ("cw:" + str(degree))

    # ccw回転
    else:
        degree = 360 - degree
        cClockWise(80)
        print ("ccw:" + str(degree))

    time.sleep(degree / turnTime)
    stop()

    print ("Avoiding complete. Here, go back to normal operation...")
    time.sleep(1)

# ========== Func multi thread (infinite read sensor) ==========
def sensorLoop():
    while True:
        readSensor()
        time.sleep(0.1)

# ========== main ==========
if __name__ == '__main__':
    setup()
    try:
        GPIO.output(sensorSwitch, 1) # enable sensors
        print ("wait for a few seconds...")
        # multi Threading
        sensorThread = threading.Thread(target=sensorLoop, args=())
        sensorThread.start()
        time.sleep(waitTime)
        distance = 0
        angle = 0.0
        usedDistance = 0
        usedAngle = 0
        #forwardFlg = 0
        print ("Automatic running...")
        startTime = time.time()
        # ========== move ==========
        while True:
            print ("[" + str(sensorList[0]) + "," + str(sensorList[1]) + "," + str(sensorList[2]) + "]")
            if error == 0:
                stop()
                # store values
                #distance, angle = signalget()
                distance, angle = inputTest()
                if angle > 180:
                    angle = angle - 360
                if (usedAngle == angle and usedDistance == distance):
                    # no updates
                    stop()
                else:
                    if -30 > angle:
                        if error == 0:
                            timeAngle = abs(angle / turnConst)
                            print ("角度補正: CCW")
                            cClockWise(50)
                            time.sleep(timeAngle)
                            stop()
                    elif 30 < angle:
                        if error == 0:
                            timeAngle = angle / turnConst
                            print ("角度補正: CW")
                            clockWise(50)
                            time.sleep(timeAngle)
                            stop()
                    if distance > 230:
                        if error == 0:
                            timeDistance = float(distance - 200) / goForwardConst
                            goForwardTime = time.time()
                            print ("前進")
                            go = 1
                            while go == 1 and error == 0:
                                goForward(50)
                                # timer
                                if time.time() - goForwardTime >= timeDistance:
                                    go = 0
                                else:
                                    go = 1
                            stop()
                    elif distance < 170:
                        if error == 0:
                            timeDistance = float(200 - distance) / goBackConst
                            goBackTime = time.time()
                            print ("後退")
                            go = 1
                            while go == 1 and error == 0:
                                goBack(50)
                                # timer
                                if time.time() - goBackTime >= timeDistance:
                                    go = 0
                                else:
                                    go = 1
                            stop()
                    else:
                        print("停止")
                        stop()
                print ("dst=" + str(distance) + ",ang=" + str(angle))
            # error
            else:
                stop()
                print("avoiding")
                avoidWall(sensorList[0], sensorList[1], sensorList[2])
            # Timer
            if runTime < (time.time() - startTime):
                print ("End of running")
                break
            # store old info
            usedAngle = angle
            usedDistance = distance
            time.sleep(0.1) # interval
    # operate here when this program ends
    finally:
        stop()
        GPIO.remove_event_detect(S1)
        GPIO.cleanup()
        sensorThread._Thread__stop()

# end of program
