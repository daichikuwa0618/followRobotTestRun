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
import DETECT3  #井上研のプログラム修正版
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
    wiringpi2.softPwmCreate(IN1, 0, 100)
    wiringpi2.softPwmCreate(IN2, 0, 100)
    wiringpi2.softPwmCreate(IN3, 0, 100)
    wiringpi2.softPwmCreate(IN4, 0, 100)
    GPIO.setup(spi_mosi, GPIO.OUT)
    GPIO.setup(spi_miso, GPIO.IN)
    GPIO.setup(spi_clk, GPIO.OUT)
    GPIO.setup(spi_ss, GPIO.OUT)
    GPIO.setup(sensor_switch,GPIO.OUT)

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
    global error, sensorList

    # chの数だけ読み込み、リストに格納する
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
        #ここまでch指定

        #センサーの値を格納する変数
        sensorList[ch] = 0
        for i in range(12):
            sensorList <<= 1
            GPIO.output(spi_clk, 1)
            if (GPIO.input(spi_miso)):
                sensorList[ch] |= 0x1
            GPIO.output(spi_clk, 0)

        GPIO.output(spi_ss, 1)

    # 逃げるかの判断
    if np.sqrt(sensorList[1] ** 2 + (sensorList[0] + sensorList[2]) ** 2) > NEAR:
        error = 1
    else:
        error = 0

# 実際に逃げる関数
def avoidWall(value):
    stop()
    time.sleep(0.5)
    back(25)

    degree = 180



def sensorLoop():
    while True:
        readSensor()
        #gpioZeroReadSensor()
        time.sleep(0.1)

if __name__ == '__main__':
    setup()
    stop()
    try:
        print ("Automatic running...")
        times = time.time()
        t = threading.Thread(target=sensorLoop)
        t.start()
        time.sleep(3)      #情報取得までのインターバル
        while True:
            if error == 0:
                forward()

            # error = 1の時
            else:
                avoidWall(sensorList)
            if jikan < (time.time() - times):
                print ("End of running")
                break
    finally:
        stop()
        GPIO.clean()
        t._Thread__stop() #マルチスレッドの強制終了

# end of program
