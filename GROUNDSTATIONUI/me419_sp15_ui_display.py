# -*- coding: utf-8 -*-
"""
USB incoming data display

Fernando Aragon
History:
3.17.15 - Created

"""

##import numpy as np        
##import matplotlib as mpl
##from matplotlib import pyplot as plt
##import cv2
import serial
import time 

def init_conn(COMPORT , BAUDRATE = 9600,TIMEOUT = 0 ):
    print "Starting Serial Connection..."
    return serial.Serial ('COM%i'%COMPORT, BAUDRATE, timeout= TIMEOUT )

def data_proc(data):
    return None, None, None

def data_display(*args):
    for arg in args:
        pass


if __name__ == "__main__":
    ser = init_conn(18,9600,0)
    while True:
        try:
            data =ser.readline()
            if data:
                data_display(*data_proc)
                print data
            else:
                print "No data"
            time.sleep(1)
        except ser.SerialTimeoutException:
            print "Data could not be read"
            time.sleep(1)


