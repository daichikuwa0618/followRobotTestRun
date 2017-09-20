# -*- coding: utf-8 -*-
#! /usr/bin/env python

#追記コメント欄
#2/9(追記)
#threadingでは問題がありそう
#モジュール間でもっと強固な引数のやりとりを行うべきか？
#python3に対応するためにprintの書き方を変更
#
#2/14(追記)
#障害情報取得の関数を導入
#回避動作を今後導入する必要がある
#stop関数の変数が1だったので100に変更
#停止時に音がなっていたのはこれの所為？
#
#2/15(追記)
#singleSensorを追加
#まだ未完成なので修正する必要あり
#
#2/22(追記)
#インターフェイスの部分を若干変更
#Threadを2つ使うのは厳しそうなので、
#障害物認識に重きを置いて妥協するしかない
#共同研究モードのときに赤外線センサをHighにしていなかった！
#この所為で多分動かなかったんだと思う……。
#焦電センサ, 自動走行の記述部分を削除
#

"""モジュールインポート"""
import RPi.GPIO as GPIO
import wiringpi2
import time
import smbus
import sys
import threading
import DETECT3  #井上研のプログラム修正版

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
#加速度センサの精度切り替えピン
selected = 12
#赤外線センサーのピン定義
sensor_switch = 25
"""変数定義"""
#global変数定義
flg2 = 0
#走行時間:1min
jikan = 60
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
    GPIO.setup(selected, GPIO.OUT)
    GPIO.setup(sensor_switch,GPIO.OUT)

def destroy():
    # モーターを停止してから、各GPIOを入力モードへ設定
    stop()
    GPIO.cleanup()

"""モータ関数"""
#前進
def forward(pwm):
    wiringpi2.softPwmWrite(IN1, pwm)
    wiringpi2.softPwmWrite(IN2, 0)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

#時間指定前進
def MoveFor(clock, pwm):
    start = time.time()
    global error
    forward(pwm)
    while True:
        RestTime = time.time() - start
        if RestTime < clock:
            error = 0
            READSensor()
            if error == 1:
                print ("aroundError")
                if(flg2 == 1):
                    stop()
                break
            elif error == 2:
                print ("underError")
                if(flg2 == 1):
                    stop()
                break
            else:
                pass
        else:
            if(flg2 == 0):
                stop()
                print ("正常に動作完了")
            break
    if(flg2 == 0):
        stop()

# 後退
def back(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, 0)
    wiringpi2.softPwmWrite(IN4, pwm)

#時間指定後進
def MoveBack(clock, pwm):
    start = time.time()
    global error
    back(pwm)
    while True:
        RestTime = time.time() - start
        if RestTime < clock:
            error = 0
            READSensor()
            if error == 1:
                print ("aroundError")
                break
            elif error == 2:
                print ("underError")
                break
            else:
                pass
        else:
            stop()
            print ("正常に動作完了")
            break
    stop()

# clockwise
def clockwise(pwm):
    wiringpi2.softPwmWrite(IN1, 0)
    wiringpi2.softPwmWrite(IN2, pwm)
    wiringpi2.softPwmWrite(IN3, pwm)
    wiringpi2.softPwmWrite(IN4, 0)

#時間指定旋回
def MoveCW(clock, pwm):
    start = time.time()
    global error
    clockwise(pwm)
    while True:
        RestTime = time.time() - start
        if RestTime < clock:
            error = 0
            READSensor()
            if error == 1:
                print ("aroundError")
                if(flg2 == 1):
                    stop()
                break
            elif error == 2:
                print ("underError")
                if(flg2 == 1):
                    stop()
                break
            else:
                pass
        else:
            if(flg2 == 0):
                stop()
                print ("正常に動作完了")
            break
    if(flg2 == 0):
        stop()

# cclockwise
def cclockwise(pwm):
    wiringpi2.softPwmWrite(IN1, pwm)
    wiringpi2.softPwmWrite(IN2, 0)
    wiringpi2.softPwmWrite(IN3, 0)
    wiringpi2.softPwmWrite(IN4, pwm)

#時間指定旋回
def MoveCCW(clock, pwm):
    start = time.time()
    global error
    cclockwise(pwm)
    while True:
        RestTime = time.time() - start
        if RestTime < clock:
            error = 0
            READSensor()
            if error == 1:
                print ("aroundError")
                break
            elif error == 2:
                print ("underError")
                break
            else:
                pass
        else:
            stop()
            print ("正常に動作完了")
            break
    stop()

# stop
def stop():
    wiringpi2.softPwmWrite(IN1, 100)
    wiringpi2.softPwmWrite(IN2, 100)
    wiringpi2.softPwmWrite(IN3, 100)
    wiringpi2.softPwmWrite(IN4, 100)
    time.sleep(0.1)

"""モータ関数Fin"""

"""センサ取得関数"""
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
    global error
    global signal
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
        if value > 2300:
            signal |= (0x1 << ch)
        else:
            #print ("初期化:" + str(ch))
            signal &= ~(0x1 << ch)

        #print (str(signal))
    #Bit Jadge---要修正
    if (signal & 0xF) != 0:
        #Bit Jadge True
        print ("現在反応しているセンサは" + str(signal) + "の番号です")
        error = 1
        stop()
        #return error
    else:
        error = 0

def singleSensor(outch):
    signal = 0
    i = 0
    global error
    #使用しているchの数だけrangeの中の値を変更する
    for ch in range(3):
        ReadSensor()
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
        if outch == ch:
            difvalue = 0
            for i in range(12):
                difvalue <<= 1
                GPIO.output(spi_clk, 1)
                if (GPIO.input(spi_miso)):
                    difvalue |= 0x1
                GPIO.output(spi_clk, 0)

            GPIO.output(spi_ss, 1)
        else:
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
        if (difvalue < 2000):
            # IDEA: 反応しなくなったらreturnする
            pass
        if (value > 2000):
            signal |= (0x1 << ch)
            # IDEA: 反応したらまたsingleSensorに入る

    #Bit Jadge---要修正
    if (signal & 0xF):
        #Bit Jadge True
        # IDEA: ここは変更するか、再考察するべき
        print ("現在反応しているセンサは" + str(signal) + "の番号です")
        error = 1
        stop()
        return error


#加速度センサの計測用関数
def AccelerationSensor():
    signal = 0
    i = 0
    #使用しているchの数だけrangeの中の値を変更する
    for ch in range(5,8):
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
        #計測結果を標準出力
        if ch > 0:
            sys.stdout.write("　")
        GPIO.output(spi_ss, 1)
        sys.stdout.write(str(value))

def sensorLoop():
    while True:
        READSensor()
        time.sleep(0.1)
#        if error == 1:
#            break

"""センサ取得関数Fin"""

"""入力関数"""
def INPUT():
    global inputmode	    #モード
    global length           #距離
    global deg              #角度
    global times            #計測時間
    global interval         #計測間隔
    global pwm              #pwm設定(0~100)
    global flg2             #autoFlg
    print ("モード選択をしてください")
    inputmode = input("1:手動モード　2:計測モード　3:共同実験モード")
    if (inputmode == 1):
        #赤外線センサを有効化。加速度センサとGPIO共用
        GPIO.output(sensor_switch, 1)
        length = input("対象までの距離を入力してください")
        deg = input("対象までの角度を入力してください")
        pwm = input("0~100までのpwmを設定してください")
        #degはロボットから右が+, 左が-とする
        print ("対象までの距離は" + str(length) + "です")
        print ("対象までの角度は" + str(deg) + "です")
        print ("設定されたpwmは" + str(pwm) + "です")
        print ("----------------------")
    elif (inputmode == 2):
        print ("計測モードを選択してください")
        select = input("1:1.5G　2:6G")
        if select == 1:
            GPIO.output(selected, 0)
        elif select == 2:
            GPIO.output(selected, 1)
        else:
            print ("設定されていないモードです")
            print ("再度入力してください")
            INPUT()
        times = input("計測時間[Sec]を入力してください")
        interval = input("計測間隔[Sec]を入力してください")
        pwm = input("0~100までのpwmの設定をしてください")
        print ("計測時間は" + str(times) + "[Sec]です")
        print ("計測間隔は" + str(interval) + "[Sec]です")
        print ("計測回数は" + str((times)/(interval)) + "回です")
        print ("設定されたpwmは" + str(pwm) + "です")
        print ("----------------------")
    elif (inputmode == 3):
        #井上研との共同実験モード
        GPIO.output(sensor_switch, 1)   #赤外線センサを有効化
        print ("井上研との共同実験を行います")
        print ("勝手に動きますので, ご注意ください")
        print ("----------------------")
    else:
        print ("設定されていないモードです")
        INPUT()

def RestINPUT():
    global length
    global deg
    global pwm
    length = input("対象までの残りの距離を入力してください")
    deg = input("対象までの残りの角度を入力してください")
    pwm = input("0~100までのpwmを設定してください")
    #degはロボットから右が-, 左が+とする
    print ("対象までの残りの距離は" + str(length) + "です")
    print ("対象までの残りの角度は" + str(deg) + "です")
    print ("設定されたpwmは" + str(pwm) + "です")
    print ("----------------------")

"""入力関数Fin"""

if __name__ == '__main__':
    setup()
    try:
        INPUT()
        dst = [0]*360
        count = 0
        used_dst = 0
        used_degs = 0
        times = time.time()
        forwardFlg = 0        #threadFlg 0:OFF 1:ON
        if (inputmode == 1) or (inputmode == 3):
            if (inputmode == 3):    #共同実験モードの場合
                a = threading.Thread(target=DETECT3.setup)
                a.start()          #周囲の情報取得
                t = threading.Thread(target=sensorLoop)
                t.start()
                time.sleep(3)      #情報取得までのインターバル
            while True:
                #手動モードor共同実験モード開始
                if inputmode == 3:
                    #変数更新
                    dst = DETECT3.dst2
                    deg = DETECT3.degs2 #角度は時計回り
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
                if error == 1:
                    forwardFlg = 0
                if length == 0 or (used_dst == dst and used_deg == deg):
                    #検出できなかった場合
                    count = count + 1
                    if count == 10:
                        stop()
                        forwardFlg = 0
                elif (-30 <= deg ) and (deg <= 30):
                    #±30degなら誤差範囲内
                    used_dst = dst
                    used_deg = deg
                    #stop()
                    if inputmode == 1:
                        x = length / 10
                        print (str(x) + "秒間前進します")
                        MoveFor(x, pwm)
                    elif signal & 0xF:
                        pass 
                    else:
                        count = 0
                        print ("前進")
                        if forwardFlg == 0:
                            forward(80)
                            forwardFlg = 1
                else:
                    #誤差以上の角度の場合
                    used_dst = dst
                    used_deg = deg
                    if -30 > deg:
                        #CCW方向に検出
                        if inputmode == 1:
                            t = -(deg / 10)
                            print ("角度補正:CCW")
                            print (str(t) + "秒間CCW方向に旋回します")
                            MoveCCW(t, pwm)
                        elif signal & 0xF:
                            pass
                        else:
                            count = 0
                            print ("角度補正:CCW")
                            cclockwise(100)
                            forwardFlg = 0
                    else:
                        #CW方向に検出
                        if inputmode == 1:
                            t = deg / 10
                            print ("角度補正:CW")
                            print (str(t) + "秒間CW方向に旋回します")
                            MoveCW(t, pwm)
                        elif signal & 0xF:
                            pass
                        else:
                            count = 0
                            print ("角度補正:CW")
                            clockwise(100)
                            forwardFlg = 0
                        #ここまで
                if inputmode == 1:
                    RestINPUT()
                #手動モード終了
                else:
                    if(jikan < time.time()-times):
                        print ("追従終わり")
                        break
        elif inputmode == 2:
            #計測モード
            forward(pwm)
            start = time.time()
            Freq = ('%.0f' % (times/interval))    #Freq << float
            Freq = int(Freq)    #Freq << int
            for i in range(0,Freq):
                AccelerationSensor()
                RestTime = time.time() - start
                sys.stdout.write("　")
                print ('%03.3f' % (RestTime))
                time.sleep(interval)
            stop()
        else:
            print ("エラーです")
    finally:
        stop()
        GPIO.cleanup()
        if inputmode == 3:
            a._Thread__stop()   #マルチスレッドを強制終了
            t._Thread__stop()   #マルチスレッドを強制終了
