import cv2
import numpy as np
import time
import string

window = cv2.namedWindow('main')
alphabetdict = dict(zip(range(97,123),string.ascii_lowercase))
cont = True

def plotgraph():
    import matplotlib as mpl
    import matplotlib.pyplot as plt

    #plt.ion()
    xx = np.linspace(1,200,200)
    yy = xx*3 + 5
    plt.plot(xx,yy)
    plt.show()
plotgraph()

while cont:


    
    key = cv2.waitKey(10)
    if key > 96 and key < 123:
        print alphabetdict[key],
    if key == 2424832:#Key press Left Arrow
        print 'Left'
    if key == 2490368:#Key press Up Arrow
        print 'Up'
    if key == 2555904:#Key press Right Arrow
        print 'Right'
    if key == 2621440:#Key press Down Arrow
        print 'Down'
    if key == 13:#Key press Enter
        plotgraph()
        print '\n'
    
    
    
    if key == 8 or key == 27:#Key press Backspace or ESC
        cont = False
        cv2.destroyAllWindows()
