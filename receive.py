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
    GPIO.setup(chan_list[1:11], GPIO.IN)

def signalInput():
    for cnt in range(8):
        if GPIO.input(chan_list[cnt+2]):
            signal += "1"
        else:
            signal += "0"
    return signal

def signalGet():
    distance = ""
    angle = ""
    GPIO.output(S0, 1)
    time.sleep(0.05)
    print ("initial signal 1")
    # not moving
    if GPIO.input(S10):
        GPIO.output(S0, 0)
        time.sleep(0.05)
        print ("No motion. Try again...")
    # moving
    else:
        print ("wait rising")
        GPIO.wait_for_edge(S1, GPIO.RISING)
        distance = signalInput()
        print ("wait falling")
        GPIO.wait_for_edge(S1, GPIO.FALLING)
        angle = signalInput()
        GPIO.output(S0, 0)
        time.sleep(0.05)
        print (type(distance), type(angle))
        result = [int(distance,2) * 4, int(angle,2)　*　BIT_TO_RAD]
        print (str(result))


# ========== main ==========
if __name__ == '__main__':
    setup()
    try:
        while True:
            signalGet()
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.remove_event_detect(S1)
        GPIO.cleanup()
