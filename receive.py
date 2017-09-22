#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
S10 = 25    #動作していないという信号(Highで動作していない)
chan_list = [S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10]

BIT_TO_RAD = 360/256    # bit情報から角度に変換
RANGE = 2               # bit情報から距離に変換(2bit左シフト)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(S0, GPIO.OUT)
    GPIO.setup(chan_list[1:11], GPIO.IN)

def signalInput(signal):
    for cnt in range(8):
        if GPIO.input(chan_list[cnt+2]):
            signal[cnt] = 1
        else:
            signal[cnt] = 0

def signalGet():
    try:
        # return values when the device is working fine
        if not GPIO.input(chan_list[10]):
            distance = angle = np.zeros(8, dtype=np.int)
            GPIO.output(S0, 1)
            GPIO.wait_for_edge(S1, GPIO.RISING)
            signalInput(distance)
            GPIO.wait_for_edge(S1, GPIO.FALLING)
            signalInput(angle)
            GPIO.output(S0, 0)
            distance = "".join(map(str, distance))
            angle = "".join(map(str, angle))
            result = [int(distance,2)<<RANGE, int(angle,2)*BIT_TO_RAD]
            return result

    except Exception as e:
        pass
    else:
        pass
    finally:
        GPIO.remove_event_detect(S1)
        GPIO.cleanup()
