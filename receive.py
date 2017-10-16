#!/usr/bin/env python
# -*- coding: utf-8 -*-

#動いていないというビットがたったらもう一度送信してもらう

import RPi.GPIO as GPIO
import time
import sys
import numpy as np

# ピン定義(S0がMSB, S9がLSB)
S0 = 7      # 右下8 合図
S1 = 5      # 左下6 識別(High:距離, Low:角度)
S2 = 6      # 左下5
S3 = 13     # 左下4
S4 = 19     # 左下3
S5 = 26     # 左下2
S6 = 12     # 右下5
S7 = 16     # 右下3
S8 = 20     # 右下2
S9 = 21     # 右下1
S10 = 25    # 動作していないという信号(Highで動作していない)
chan_list = [S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10]

BIT_TO_RAD = 360/256    # bit情報から角度に変換
RANGE = 2               # bit情報から距離に変換(2bit左シフト)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(S0, GPIO.OUT)
    GPIO.setup(chan_list[1:11], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def signalInput():
    signal = ""
    cnt = 0
    for cnt in range(8):
        if GPIO.input(chan_list[cnt+2]):
            signal += "1"
        else:
            signal += "0"
    return signal

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
        #result = [int(distance,2) * 4, float(int(angle,2)) * 360 / 256]
        result = [int(distance,2), int(angle,2)]

        print (str(result))


# ========== main ==========
if __name__ == '__main__':
    setup()
    try:
        while True:
            signalGet()
            time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()

    finally:
        GPIO.cleanup()
