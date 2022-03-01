from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
from SysID import Chirp
from math import sin, pi, sqrt, log, exp
import time


def current_demo(dev, amp=1.0):
    """ Tests 1000Hz performance"""
    dev.set_current_gains(kp=40, ki=400, ff=128)
    for t in SoftRealtimeLoop(dt=0.001):
        dev.i=amp*sin(t*55*2*pi) # a musical note three octaves below 440 Hz A


def current_demo(dev, amp=1.0, dt=0.001):
    print("Testing real-time performance. Press CTRL-C to finish.")
    ttarg = None 
    sum_err = 0.0
    sum_var = 0.0
    n = 0
    chirp = Chirp(250, 50, .25)
    dev.set_current_gains()
    # time.sleep(0.5)
    # dev.update()
    for t in SoftRealtimeLoop(dt = dt, report=True):
        if ttarg is None: 
            # inits ttarg on first call
            ttarg = time.time()+dt
            # then skips the first loop
            continue
        # print(t, chirp._phase)
        dev.update()
        print(dev.θ*180/3.1415)
        dev.τ=amp*chirp.next(t)*3/3.7 # a barely audible note
        error = time.time()-ttarg # seconds
        sum_err += error
        sum_var += error**2
        n+=1
        ttarg+=dt
      
    print('In %d cycles at %.2f Hz:'%(n, 1./dt))
    print('\tavg error: %.3f milliseconds'% (1e3*sum_err/n))
    print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((sum_var-sum_err**2/n)/(n-1))))


def main():
    with ActPackMan('/dev/ttyActPackB', gear_ratio=9, updateFreq=1000) as dev:
        current_demo(dev, amp=3.0)
    print("done with current_demo()")

if __name__ == '__main__':
    main()
