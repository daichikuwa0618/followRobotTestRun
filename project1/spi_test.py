#!/usr/bin/env python
# -*- coding:utf-8 -*-
import spidev
spi = spidev.SpiDev()
# spi.open(bus,device)
spi.open(0,0)
# 解説参照
def readAdc(channel):
 adc = spi.xfer2([1,(8+channel)<<4,0])
 print(adc)
 data = ((adc[1]&3) << 8) + adc[2] return data def convertVolts(data, vref): volts = (data * vref) / float(1023) volts = round(volts,4) return volts if __name__ == '__main__': # 1ch -> 0, ..., 8ch -> 7
 ch = 0
 data = readAdc(ch)
 print("adc : {:8} ".format(data))
# MCP3008 の Vref に入れた電圧. ここでは 5V
 v = 5.0
 volts = convertVolts(data, v)
 print("volts: {:8.2f}".format(volts))