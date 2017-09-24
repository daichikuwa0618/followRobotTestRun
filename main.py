#!/usr/bin/env/ python
# -*- coding: utf-8 -*-

# Created Daichi Hayashi 2017/09/24
# From Yonago Institute of Technology

# +++++ Discription +++++
#   escape from wall whenever the robot moveServo
#   and, follow to anyone who is moveing

# ========== import ==========
import RPi.GPIO as GPIO
import wiringpi2
import time
import sys
import threading
import DETECT3
import numpy as np
from time import sleep

# ========== PIN assign ==========
# moter pins
IN1 = 3
IN2 = 4
IN3 = 17
IN4 = 27
# AD-Converter
spi_clk = 11
spi_mosi = 10
spi_miso = 9
spi_ss = 8
# Sensor sensor
sensor_switch = 2
# signals from Inoue Lab. S0:MSB, S10:LSB
S0 = 7      # Lower left 8 合図
S1 = 5      # Lower left 6 識別(High:Distance, Low:Angle)
S2 = 6      # Lower left 5
S3 = 13     # Lower left 4
S4 = 19     # Lower left 3
S5 = 26     # Lower left 2
S6 = 12     # Lower right 5
S7 = 16     # Lower right 3
S8 = 20     # Lower right 2
S9 = 21     # Lower right 1
S10 = 25    # if this is High, isn't working
paraList = [S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10] # parallel list

# ========== variables ==========
# runTime:[sec]
runTime = 60
# waitTime
waitTime = 5
# angle
deg = 0
# Distance
length = 0
error = 0
# variables of sensors
sensorList = [0, 0, 0]
# Threshold of sensor
nearValue = 2300
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
    GPIO.setup(sensor_switch,GPIO.OUT)
    # parallel communication
    GPIO.setup(S0, GPIO.OUT)
    GPIO.setup(paraList, GPIO.IN)

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

# ========== Func parallel ==========
def signalInput(signal):
    for cnt in range(8):
        if GPIO.input(chan_list[cnt+2]):
            signal[cnt] = 1
        else:
            signal[cnt] = 0
        return signal

# ========== Func return results ==========
def signalGet():
    try:
        # return values when the device is working fine
        if not GPIO.input(chan_list[10]):
            distance = angle = np.zeros(8, dtype=np.int)
            GPIO.output(S0, 1)
            GPIO.wait_for_edge(S1, GPIO.RISING)
            distance = signalInput(distance)
            GPIO.wait_for_edge(S1, GPIO.FALLING)
            angle = signalInput(angle)
            GPIO.output(S0, 0)
            distance = "".join(map(str, distance))
            angle = "".join(map(str, angle))
            result = [int(distance,2)<<rangeTrans, int(angle,2)*bitToRange]
            return result

    except Exception as e:
        pass
    else:
        pass
    finally:
        #GPIO.remove_event_detect(S1)
        #GPIO.cleanup()

# ========== readSensor ==========
# this func also judge whether avoid or not
def readSensor():
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

    # judge whether avoid or not
    if np.sqrt((sensorList[1] ** 2) + ((sensorList[0] + sensorList[2]) ** 2)) > NEAR:
        error = 1
    else:
        error = 0

# ========== Func avoid from wall ==========
def avoidWall(value0, value1, value2):
    # mapping of sensor
    # ch0:Right ch1:Front ch2:Left
    # 角度から秒数に変換する定数
    turnTime = 70 # deviding const translating from deg to sec
    resetNum = 80 # variable of correction
    stop()
    # Go back a little bit
    #back(50)
    time.sleep(1)
    #stop()

    # avoiding angle
    degree = 180
    # ZeroDevisionError
    if value1 <= resetNum:
        value1 = resetNum + 1
    if value0 <= resetNum:
        value0 = resetNum
    if value2 <= resetNum:
        value2 = resetNum
    # 180degからどれだけ回転するのか
    cwDeg = np.rad2deg(np.arctan2(value2 - resetNum, value1 - resetNum))
    ccwDeg = np.rad2deg(np.arctan2(value0 - resetNum, value1 - resetNum))
    # 角度の補正
    degree = degree + cwDeg - ccwDeg

    #nowTime = time.time()

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
        GPIO.output(sensor_switch, 1) # enable sensors
        print ("wait...")
        # multi Threading
        signalThread = threading.Thread(target = signalGet)
        sensorThread = threading.Thread(target = sensorLoop)
        signalThread.start()
        sensorThread.start()
        time.sleep(wait_time)
        print ("Automatic running...")
        startTime = time.time()
        while True:
            print ("[" + str(sensorList[0]) + "," + str(sensorList[1]) + "," + str(sensorList[2]) + "]")
            if error == 0:
                goForward(80)
            # error true
            else:
                print("avoiding")
                avoidWall(sensorList[0], sensorList[1], sensorList[2])
            # Timer
            if runTime < (time.time() - starTime):
                print ("End of running")
                break
            time.sleep(0.1)
    # ctrl + C exception
    except KeyboardInterrupt:
        stop()
        GPIO.cleanup()
        signalThread._Thread__stop()
        sensorThread._Thread__stop()
    # operate here when this program ends
    finally:
        stop()
        GPIO.cleanup()
        signalThread._Thread__stop()
        sensorThread._Thread__stop()

# end of program
