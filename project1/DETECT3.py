# -*- coding: utf-8 -*-
import serial
import time
import binascii
length = 800
bytecount = 0
address = 0
startcount = 0
counter = 0
degs = 0
degs2 = 0
dst = [0]*500
dst2 = [0]*500
stra = [0]*4
a = 360
length = 0

#con = serial.Serial('/dev/ttyAMA0',115200,timeout=10)

#con.write(chr(165))
#con.write(chr(32))
#print con.portstr

#print "Address  | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F"
#print "----------------------------------------------------------"
#print "00000000 :",

#f = open("data.log", "w")
def setup():
    bytecount = 0
    address = 0
    startcount = 0
    counter = 0
    global dst, dst2
    #dst = [0]*361
    stra = [0]*4
    global degs, degs2
    #degs = 0


    con = serial.Serial('/dev/ttyAMA0',115200,timeout=10)

    con.write(chr(165))
    con.write(chr(32))
    #print con.portstr
    #print "ここまで完了"
    while 1 :
        counter = counter + 1
        str = con.read(1)
        str = binascii.b2a_hex(str)
        #print str
        if str == "a5" and startcount == 0:
            startcount += 1
            #print counter
        elif str == "5a" and startcount == 1 :
            startcount += 1
            #print counter
        elif str == "05" and startcount == 2 :
            startcount += 1
            #print counter
        elif str == "00" and startcount == 3 :
            startcount += 1
            #print counter
        elif str == "00" and startcount == 4 :
            startcount += 1
            #print counter
        elif str == "40" and startcount == 5 :
            startcount += 1
            #print counter
        elif str == "81" and startcount == 6 :
            startcount += 1
            #print counter

        if startcount == 7 :
            break

    print ("ok")

    while 1 :
        str = con.read(1)
        str = binascii.b2a_hex(str)
        #print str,

        if bytecount == 1 :
            stra[0] = int(str, 16)
        elif bytecount == 2 :
            stra[1] = int(str, 16)
        elif bytecount == 3 :
            stra[2] = int(str, 16)
        elif bytecount == 4 :
            stra[3] = int(str, 16)

        if bytecount == 4 :
            degs = ((stra[1]<<7) | (stra[0]>>1)) / 64
            dst[degs] =int(((stra[3]<<8) | (stra[2])) / 4)
            if dst[degs] > 800:
                dst[degs] = 0
            if dst[degs] != 0:
                dst2 = dst
                degs2 = degs
            #print degs
            #print dst[degs]
            #if dst[degs] != 0 :
            #a = degs
                #length = dst[a]
                #print "ifnoNAKA"


        bytecount = bytecount + 1
        address = address + 1

        if bytecount > 4 :
            #print("")
            #f.write('\n')
            bytecount = 0
            #print "%08X :" % address,

        #if address > 0x00010000 :
        #    break

    #f.close()
