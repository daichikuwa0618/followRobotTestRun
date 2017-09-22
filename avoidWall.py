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
from gpiozero import MCP3208 # ADCを使用するパッケージ
import numpy as np

adc0 = MCP3208(channel = 0)
adc1 = MCP3208(channel = 1)
adc2 = MCP3208(channel = 2)
adc3 = MCP3208(channel = 3)

"""ピン定義"""
#各モータのピン定義
IN1 = 5
IN2 = 6
IN3 = 18
IN4 = 23
#ADコンバータのピン定義
spi_clk = 11
spi_mosi = 10
spi_miso = 9
spi_ss = 8
#赤外線センサーのピン定義
sensor_switch = 25
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
    #GPIO.setup(sensor_switch,GPIO.OUT)

    """モータ関数"""
    #前進
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
    global error, signal
    #使用しているchの数だけrangeの中の値を変更する
    for ch in range(3):
        #ReadSensor()
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
        #ここまでch指定

        #センサーの値を格納する変数
        value = 0
        for i in range(12):
            value <<= 1
            GPIO.output(spi_clk, 1)
            if (GPIO.input(spi_miso)):
                value |= 0x1
            GPIO.output(spi_clk, 0)

        GPIO.output(spi_ss, 1)

        if (value > 2300):
            print ("The channel of responsing:" + str(ch))

        sensorList[ch] = value

    # 逃げるかの判断
    if np.sqrt((sensorList[1] ** 2) + ((sensorList[0] + sensorList[2]) ** 2)) > NEAR:
        error = 1
    else:
        error = 0

# 実際に逃げる関数
def avoidWall(value):
    # センサの配置
    # ch0:右 ch1:正面 ch2:左
    # 角度から秒数に変換する定数
    turnTime = 90
    stop()
    # 少し後退する
    back(25)
    time.sleep(1)
    stop()

    # 逃げる角度
    degree = 180
    # ZeroDevisionError回避
    if sensorList[1] == 0:
        sensorList = 1
    # 180degからどれだけ回転するのか
    cwDeg = np.arctan2(sensorList[2]/sensorList[1])
    ccwDeg = np.arctan2(sensorList[0]/sensorList[1])
    # 角度の補正
    degree = degree + cwDeg - ccwDeg

    # cw回転
    if degree <= 180:
        clockwise(50)
        time.sleep(degree / turnTime)
        print ("cw:" + str(degree))

    # ccw回転
    else:
        cclockwise(50)
        time.sleep((360 - degree) / turnTime)
        print ("ccw:" + str(degree))


def sensorLoop():
    while True:
        readSensor()
        #gpioZeroReadSensor()
        time.sleep(0.1)

if __name__ == '__main__':
    setup()
    try:
        print ("Automatic running...")
        times = time.time()
        t = threading.Thread(target=sensorLoop)
        t.start()
        time.sleep(3)      #情報取得までのインターバル
        while True:
            print (sensorList)
            if error == 0:
                forward(50)
            # error = 1の時
            else:
                print("avoiding")
                avoidWall(sensorList)
            if jikan < (time.time() - times):
                print ("End of running")
                break
    finally:
        stop()
        GPIO.cleanup()
        t._Thread__stop() #マルチスレッドの強制終了

# end of program
