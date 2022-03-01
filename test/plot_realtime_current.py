from rtplot import client
from sys import path
from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
from math import sin, pi, sqrt, log, exp
from SysID import Chirp
import time



def current_demo(dev, amp=1.0):
    """ Tests 1000Hz performance"""
    dev.set_current_gains(kp=40, ki=400, ff=128)
    for t in SoftRealtimeLoop(dt=0.001):
        dev.i=amp*sin(t*55*2*pi) # a musical note three octaves below 440 Hz A


def current_demo(dev, amp=1.0, dt=0.001):
    #Initialize the plotter
    plot_config1 = {'names':['position (rad)'],
                   'yrange':[-3.14,3.14],
                   'title':"Motor Position",
                   'ylabel':"Motor Position (radians)"}

    plot_config2 = {'names':['current (amps)'],
                   'yrange':[-3,3],
                   'title':"Motor Current",
                   'ylabel':"Motor Current (amps)"}
    #client.initialize_plots(2)
    client.initialize_plots([plot_config1, plot_config2])

    print("Testing real-time performance. Press CTRL-C to finish.")
    ttarg = None 
    sum_err = 0.0
    sum_var = 0.0
    n = 0
    chirp = Chirp(250, 50, .25)
    dev.set_current_gains(kp=40, ki=400, ff=128)
    for t in SoftRealtimeLoop(dt = dt):
        if ttarg is None: 
            # inits ttarg on first call
            ttarg = time.time()+dt
            # then skips the first loop
            continue
        # print(t, chirp._phase)
        dev.i=amp*chirp.next(t) # a barely audible note
        dev.update()

        #Get angle
        client.send_array([dev.θ, dev.i])

        error = time.time()-ttarg # seconds
        sum_err += error
        sum_var += error**2
        n+=1
        ttarg+=dt
      
    print('In %d cycles at %.2f Hz:'%(n, 1./dt))
    print('\tavg error: %.3f milliseconds'% (1e3*sum_err/n))
    print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((sum_var-sum_err**2/n)/(n-1))))

if __name__ == '__main__':
    with ActPackMan('/dev/ttyACM0', updateFreq=1000, gear_ratio=9) as dev:
        current_demo(dev, amp=3.0)
    print("done with current_demo()")
