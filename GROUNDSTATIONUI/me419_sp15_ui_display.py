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
    """
    Input data format:
    (Internal temperature[C], External temperature[C], Air Pressure[Pa],
    Longitude[dd], Latitude[dd], Altitude[dd], Velocity[m/s]
    Roll[dd], Pitch[dd], Yaw[dd], Roll_rate[dd/s], Pitch_rate[dd/s], yaw_rate[dd/s]
    Arduino Voltage)
    """
    def dd2dms(dd):
        mnt,sec = divmod(dd*3600,60)
        deg,mnt = divmod(mnt,60)
        return deg,mnt,sec

    datalist = data.split(',')
    datalist = [float(i) for i in datalist]
    datalist[3:6] = [dd2dms(i) for i in datalist[3:6]]
    return *datalist

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


