# coding=utf-8
#!/usr/bin/env python

# project1/test_run.py のコピーをし、ADC部分を改良したプログラム
# とりあえずレーザレンジファインダーは使わない方向

"""モジュールインポート"""
import RPi.GPIO as GPIO
import wiringpi2
import time
import smbus
import sys
import threading
import DETECT3  #井上研のプログラム修正版
from gpiozero import MCP3208 # ADCを使用するパッケージ

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
"""i2c設定"""
#i2cの初期化設定
i2c = smbus.SMBus(1)
#赤外線センサーのアドレス及び値を格納しているアドレス
SENSOR_ADRS = 0x40
DISTANCE_ADRS = 0x5E

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

"""
# センサ取得関数
#i2cを用いた赤外線センサーの関数
def ReadSensor():
    global error
    ans1 = i2c.read_byte_data(SENSOR_ADRS, DISTANCE_ADRS)
    ans2 = i2c.read_byte_data(SENSOR_ADRS, 0x5F)
    t_data = ((ans1*16+ans2)/16)/4
    if (t_data > 20):
        error = 2
        stop()
        return error

#ADコンバータを用いた赤外線センサーの関数
def READSensor():
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
        #2000=閾値、この値によって壁の検知を行う
        #閾値を大きくすると、距離が短くなる
        if (value > 2300):
            signal |= (0x1 << ch)
        else:
            signal &= ~(0x1 << ch)

    #Bit Jadge---要修正
    if (signal & 0xF) != 0:
        #Bit Jadge True
        print ("現在反応しているセンサは" + str(signal) + "の番号です")
        error = 1
        stop()
        #return error
    else:
        error = 0
"""

# GPIOzeroを用いた赤外線センサの使用(追加部分)
def gpioZeroReadSensor():
    global error, signal
    adcList = [adc0.value, adc1.value, adc2.value, adc3.value]
    for i in range(4):
        if adcList[i] > 0.5:
            signal |= (0x1 << i)
        else:
            signal &= ~(0x1 << i)
        print("sensor" + str(i) + "=" + str(adcList[i]))

    # Bit Judge
    if (signal & 0xF) != 0:
        #Bit Jadge True
        print ("現在反応しているセンサは" + str(signal) + "の番号です")
        error = 1
        stop()
        #return error
    else:
        error = 0

def sensorLoop():
    while True:
        #READSensor()
        gpioZeroReadSensor()
        time.sleep(0.1)

if __name__ == '__main__':
    setup()
    try:
        #井上研との共同実験モード
        GPIO.output(sensor_switch, 1)   #赤外線センサを有効化
        print ("井上研との共同実験を行います")
        time.sleep(wait_time)
        print ("勝手に動きますので, ご注意ください")
        print ("----------------------")
        dst = [0]*360
        count = 0
        used_dst = 0
        used_degs = 0
        times = time.time()
        threadFlg = 0        #threadFlg 0:OFF 1:ON
        forwardflg = 0
        errorflg = 0
        #a = threading.Thread(target=DETECT3.setup)
        #a.start()          #周囲の情報取得
        t = threading.Thread(target=sensorLoop)
        t.start()
        time.sleep(3)      #情報取得までのインターバル
        while True:
            #変数更新
            #dst = DETECT3.dst2
            #deg = DETECT3.degs2 #角度は時計回り
            time.sleep(0.1)    #インターバル(不要かもしれない)
            length = dst[deg]
            if length < 10:
                #補足対象までの距離が30cm未満の場合
                length = 0
            if deg > 180:       #角度補正
                deg = deg - 360
            if signal & 0xF :
                stop()
                print ("緊急停止！")
                errorflg = 1
            if error == 1:
                forwardflg = 0
                errorflg = 1
            """
            if length == 0 or (used_dst == dst and used_deg == deg):
                #検出できなかった場合
                count = count + 1
                if count == 10:
                    stop()
                    forwardflg = 0
            """
            elif (-30 <= deg ) and (deg <= 30):
                #±30degなら誤差範囲内
                used_dst = dst
                used_deg = deg
                count = 0
                if forwardflg == 0 and errorflg == 0:
                    print ("前進")
                    forward(100)
                    forwardflg = 1
                if errorflg == 1:
                    errorflg -= 1
            else:
                #誤差以上の角度の場合
                used_dst = dst
                used_deg = deg
                if -30 > deg:
                    #CCW方向に検出
                    count = 0
                    if errorflg == 0:
                        print ("角度補正:CCW")
                        cclockwise(100)
                        forwardflg = 0
                    if errorflg == 1:
                        errorflg -= 1
                else:
                    #CW方向に検出
                    count = 0
                    if errorflg == 0:
                        print ("角度補正:CW")
                        clockwise(100)
                        forwardflg = 0
                    if errorflg == 1:
                        errorflg -= 1
                    #ここまで
            if(jikan < time.time()-times):
                print ("追従終わり")
                break
    finally:
        stop()
        GPIO.cleanup()
        #a._Thread__stop()   #マルチスレッドを強制終了
        t._Thread__stop()   #マルチスレッドを強制終了
