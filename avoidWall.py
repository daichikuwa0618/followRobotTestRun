# -*- coding: utf-8 -*-
#!/usr/bin/env python

# Created Daichi Hayashi 2017/09/21
# From Yonago Institute of Technology

# escape from wall whenever the robot moveServo

"""モジュールインポート"""
import RPi.GPIO as GPIO
import wiringpi2
import time
import smbus
import sys
import threading
import numpy as np

"""ピン定義"""
#各モータのピン定義
IN1 = 3
IN2 = 4
IN3 = 17
IN4 = 27
#ADコンバータのピン定義
spi_clk = 11
spi_mosi = 10
spi_miso = 9
spi_ss = 8
#赤外線センサーのピン定義
sensor_switch = 2

"""変数定義"""
#走行時間:1min
jikan = 60
#待機時間
wait_time = 5
#角度
deg = 0
#距離
length = 0
error = 0
signal = 0
# センサーの値
sensorList = [0, 0, 0]
# 近いと判断する閾値
NEAR = 2300

def setup():
    GPIO.setmode(GPIO.BCM)
    wiringpi2.wiringPiSetupGpio()
    wiringpi2.pinMode(IN1, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN2, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN3, wiringpi2.GPIO.PWM_OUTPUT)
    wiringpi2.pinMode(IN4, wiringpi2.GPIO.PWM_OUTPUT)
    # 初期値から停止させておく
    wiringpi2.softPwmCreate(IN1, 100, 100)
    wiringpi2.softPwmCreate(IN2, 100, 100)
    wiringpi2.softPwmCreate(IN3, 100, 100)
    wiringpi2.softPwmCreate(IN4, 100, 100)
    GPIO.setup(spi_mosi, GPIO.OUT)
    GPIO.setup(spi_miso, GPIO.IN)
    GPIO.setup(spi_clk, GPIO.OUT)
    GPIO.setup(spi_ss, GPIO.OUT)
    GPIO.setup(sensor_switch,GPIO.OUT)

"""モータ関数"""
# 前進
def forward(pwm):
    wiringpi2.softPwmWrite(IN1, pwm)
    wiringpi2.softPwmWrite(IN2, 0)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

# 後退
def back(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, 0)
    wiringpi2.softPwmWrite(IN4, pwm)

# clockwise
def clockwise(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

# cclockwise
def cclockwise(pwm):
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


# ADコンバータを用いた赤外線センサーの関数
# センサの値に応じて逃げるべきかも判断する
def readSensor():
    i = 0
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


# 実際に逃げる関数
def avoidWall(value0, value1, value2):
    # センサの配置
    # ch0:右 ch1:正面 ch2:左
    # 角度から秒数に変換する定数
    turnTime = 90
    resetNum = 80 #補正用変数
    stop()
    # 少し後退する
    #back(25)
    time.sleep(1)
    stop()

    # 逃げる角度
    degree = 180
    # ZeroDevisionError回避
    if value1 <= resetNum:
        value1 = resetNum + 1
    if value0 <= resetNum:
        value0 = resetNum
    if value2 <= resetNum:
        value2 = resetNum
    # 180degからどれだけ回転するのか
    cwDeg = np.rad2deg(np.arctan2(value2-resetNum, value1-resetNum))
    ccwDeg = np.rad2deg(np.arctan2(value0-resetNum, value1-resetNum))
    # 角度の補正
    degree = degree + cwDeg - ccwDeg

    #nowTime = time.time()

    # cw回転
    if degree <= 180:
        clockwise(80)
        print ("cw:" + str(degree))

    # ccw回転
    else:
        degree = 360 - degree
        cclockwise(80)
        print ("ccw:" + str(degree))

    time.sleep(degree / turnTime)
    stop()

    print ("Avoiding complete. Here, go back to normal operation...")
    time.sleep(1)

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
        t = threading.Thread(target=sensorLoop)
        t.start()
        time.sleep(wait_time)
        print ("Automatic running...")
        times = time.time() # start time
        while True:
            print ("[" + str(sensorList[0]) + "," + str(sensorList[1]) + "," + str(sensorList[2]) + "]")
            if error == 0:
                forward(80)
            # error true
            else:
                print("avoiding")
                avoidWall(sensorList[0], sensorList[1], sensorList[2])
            # Timer
            if jikan < (time.time() - times):
                print ("End of running")
                break
            time.sleep(0.1)
    # operate here when this program ends
    finally:
        stop()
        GPIO.cleanup()
        t._Thread__stop()

# end of program
