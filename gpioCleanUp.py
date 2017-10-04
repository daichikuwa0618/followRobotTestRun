# !/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

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

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(S0, GPIO.OUT)
    GPIO.setup(chan_list[1:11], GPIO.IN)

setup()
GPIO.cleanup()