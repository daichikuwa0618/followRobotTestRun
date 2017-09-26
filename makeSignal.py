# -*- coding: utf-8 -*-
#/usr/bin/env python

#

import RPi.GPIO as GPIO
import time
import sys

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
S10 = 25    # 動作していないという信号(Highで動作してい
chan_list = [S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10]

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(S0, GPIO.IN)
    GPIO.setup(chan_list[1:11], GPIO.OUT)

def decToBit(value):
    valueList = np.zeros(8, dtype=np.int)
    for i in range(8):
        if (value >> i) & 1:
            valueList[i] = "1"
        else:
            valueList[i] = "0"


# ========== main ==========
if __name__ == '__main__':
    inputMode = input("working? 0:yes, 1:no")
    if inputMode == 1:
        GPIO.output(chan_list[10], 1)
    else:
        GPIO.output(chan_list[10], 0)
        distance = int(input("distance...:")) * 4
        angle = int(input("angle(0 to 255)...:"))

        distanceList = angleList =np.zeros(8, dtype=np.int)
        GPIO.wait_for_edge(S0, GPIO.RISING)
        distanceList = decToBit(distance)
        angleList = decToBit(angle)
        # send distance
        for i in range(8):
            if distanceList[i] == 1:
                GPIO.output(chan_list[i + 2], 1)
            else:
                GPIO.output(chan_list[i + 2], 0)
        GPIO.output(S1, 1) # send

        # send angle
        for i in range(8):
            if angleList[i] == 1:
                GPIO.output(chan_list[i + 2], 1)
            else:
                GPIO.output(chan_list[i + 2], 0)
        GPIO.output(S1, 0) # send




# end of program
