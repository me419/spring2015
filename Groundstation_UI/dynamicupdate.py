import matplotlib.pyplot as plt
plt.ion()
class DynamicUpdate():

    def on_launch(self):
        #Set up plot
        self.figure = plt.figure(figsize=(3,3),dpi = 100)
        self.ax1 = self.figure.add_subplot(2,1,1)
        self.ax2 = self.figure.add_subplot(2,1,2)
        self.lines, = self.ax1.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
        self.ax1.set_autoscaley_on(True)
        self.ax1.set_autoscalex_on(True)
        #Other stuff
        self.ax1.grid()

    def on_running(self, xdata, ydata):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax1.relim()
        self.ax1.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    #Example
    def __call__(self):
        import numpy as np
        import time
        self.on_launch()
        xdata = []
        ydata = []
        for x in np.arange(0,10,0.5):
            xdata.append(x)
            ydata.append(np.exp(-x**2)+10*np.exp(-(x-7)**2))
            self.on_running(xdata, ydata)
            time.sleep(1)
        return xdata, ydata

d = DynamicUpdate()
d()
