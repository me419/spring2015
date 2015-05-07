# -*- coding: utf-8 -*-
"""
USB incoming data display

Fernando Aragon
History:
3.17.15 - Created

"""

import serial
import time 

print "Starting Serial Connection..."
ser = serial.Serial ('COM18', 9600, timeout= 0 )

while 1:
    try:
        data =ser.readline()
        if data:
            print data
        else:
            print "No data"
        time.sleep(1)
    except ser.SerialTimeoutException:
        print "Data could not be read"
        time.sleep(1)