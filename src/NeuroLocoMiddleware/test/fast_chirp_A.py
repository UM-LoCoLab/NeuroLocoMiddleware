from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
from SysID import Chirp
from math import sin, pi, sqrt, log, exp
import time

def chirp_demo(dev, amp=1.0, dt=0.001):
    print("Chirping ActPackA. Press CTRL-C to finish.")
    chirp = Chirp(250, 50, .25)
    dev.set_current_gains()
    loop = SoftRealtimeLoop(dt = dt, report=True, fade=0.01)
    for t in loop:
        dev.update()
        dev.τ=loop.fade*amp*chirp.next(t)*3/3.7 # a barely audible note

def main():
    with ActPackMan('/dev/ttyActPackA', gear_ratio=9, updateFreq=1000) as dev:
        chirp_demo(dev, amp=3.0)
    print("done with chirp_demo()")

if __name__ == '__main__':
    main()
