import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import serial
import time

from me419_sp15_display_fakedata import datastream
d = datastream()

comport = 1

class Ground_UI():
    def __init__(self):
        print "Starting Serial Connection..."
        self.serial_conn = self.init_conn(comport)
        self.datalist = [[] for _ in range(14)]
        self.init_display()

    def init_conn(self,COMPORT , BAUDRATE = 9600,TIMEOUT = 0 ):
        print "Starting Serial Connection..."
        return serial.Serial ('COM%i'%COMPORT, BAUDRATE, timeout= TIMEOUT )
    
    def init_display(self):
        plt.ion()
        self.figure = plt.figure(figsize=(15,10),dpi = 64)
        self.text1 = self.figure.text(0.7,0.8,"")
        self.text2 = self.figure.text(0.7,0.8,"")
        self.text3 = self.figure.text(0.7,0.8,"")
        self.text4 = self.figure.text(0.7,0.8,"")
        self.text5 = self.figure.text(0.7,0.8,"")
        
        #self.figure.patch.set_facecolor('blue')
        self.figure.suptitle("BoatSat2015 teleView Version0.6",fontsize = 20)
        self.gs = gridspec.GridSpec(3, 3)
        self.gs.update(left=0.06, right=0.98, wspace=0.3)
        self.dirplot = self.figure.add_subplot(self.gs[0,0:2])
        self.dirplot.set_title("Pointing Directions")
        self.dirplot.set_ylabel("Degrees")
        self.posplot = self.figure.add_subplot(self.gs[1:,0:2])
        self.posplot.set_title("Location")
        self.posplot.set_ylabel("Latitude")
        self.posplot.set_xlabel("Longitude")
        self.preplot = self.figure.add_subplot(self.gs[1,-1])
        self.preplot.set_title("Pressure")
        self.preplot.set_ylabel("Pressure [atm]")
        self.templot = self.figure.add_subplot(self.gs[2,-1])
        self.templot.set_title("Temperature")
        self.templot.set_ylabel("Temperature [$\degree$C]")
        self.plotlist = [self.dirplot,self.dirplot,self.dirplot,
                         self.posplot,
                         self.preplot,
                         self.templot,self.templot,]
        self.linelist = [i.plot([],[], '.-')[0] for i in self.plotlist]
        for i,j in zip(self.linelist,['Roll [deg]','Pitch [deg]','Yaw [deg]',
                                      'Position','Pressure [atm]','Int. Temp. [$\degree$C]','Ext. Temp. [$\degree$C]']):
            i.set_label(j)
            
        self.plotlist = [self.dirplot,self.posplot,self.preplot,self.templot]
        self.reading = [0]
        #Autoscale on unknown axis and known lims on the other
        for i in self.plotlist:
            i.set_autoscaley_on(True)
            i.set_autoscalex_on(True)
            if i in [self.dirplot,self.templot]: i.legend(fancybox=True, framealpha=0.5)
        

    def display_loop(self,data):
        for i in range(3):
            self.linelist[i].set_xdata(self.reading)
            self.linelist[i].set_ydata(self.datalist[7+i])
        self.linelist[3].set_xdata(self.datalist[3])
        self.linelist[3].set_ydata(self.datalist[4])
        self.linelist[4].set_xdata(self.reading)
        self.linelist[4].set_ydata(self.datalist[2])
        
        self.linelist[5].set_xdata(self.reading)
        self.linelist[5].set_ydata(self.datalist[0])
        self.linelist[6].set_xdata(self.reading)
        self.linelist[6].set_ydata(self.datalist[1])

        self.text1.set_visible(False)
        self.text1 = self.figure.text(0.67,0.85,"Time Elapsed: %is"%self.reading[-1],fontsize = 16)
        self.text2.set_visible(False)
        self.text2 = self.figure.text(0.67,0.78,"Arduino Voltage: %is"%self.datalist[-1][-1],fontsize = 16)
        self.text3.set_visible(False)
        self.text3 = self.figure.text(0.85,0.85,"Roll Rate: %is"%self.datalist[10][-1],fontsize = 16)
        self.text4.set_visible(False)
        self.text4 = self.figure.text(0.85,0.78,"Pitch Rate: %is"%self.datalist[11][-1],fontsize = 16)
        self.text5.set_visible(False)
        self.text5 = self.figure.text(0.85,0.71,"Yaw Rate: %is"%self.datalist[12][-1],fontsize = 16)
        
        

        for plot in self.plotlist:
            plot.relim()
            plot.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def data_proc(self,data):
        """
        Input data format:
        (Internal temperature[C], External temperature[C], Air Pressure[Pa],
        Longitude[dd], Latitude[dd], Altitude[dd], Velocity[m/s]
        Roll[dd], Pitch[dd], Yaw[dd], Roll_rate[dd/s], Pitch_rate[dd/s], yaw_rate[dd/s]
        Arduino Voltage)
        """
        datalist = data.split(',')
        for ind,val in enumerate(datalist):
            try:
                datalist[ind] = float(val)
                if ind == 2: datalist[ind] = datalist[ind]/101325.
            except:
                datalist[ind] = None
            self.datalist[ind].append(datalist[ind])
        return datalist


    def __call__(self):
        while True:
            try:
                self.data = self.serial_conn.readline()

                #test case
                #self.data  = next(d)

                procdata = self.data_proc(self.data)
                if self.reading == [0]:
                    self.initialpos = tuple(procdata[3:6])
                
                if self.data:
                    self.display_loop(procdata)
                    print self.data
                else:
                    print "No data"
                
                time.sleep(1)
                self.reading.append(self.reading[-1]+1)
            except Exception as e:
                print e
                print "Data could not be read"
                time.sleep(1)

def dd2dms(dd):
    mnt,sec = divmod(dd*3600,60)
    deg,mnt = divmod(mnt,60)
    return deg,mnt,sec

if __name__ == "__main__":
    grst = Ground_UI()
    grst()

