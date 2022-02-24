from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
from AhrsManager import AhrsManager
from SysID import Chirp
from math import sin, pi, sqrt, log, exp
import time


def fast_chirp_demo(devA, devB, ahrsA, ahrsB, amp=1.0, dt=0.001):
    print("Testing real-time performance. Two Actuators. Press CTRL-C to finish.")
    ttarg = None 
    sum_err = 0.0
    sum_var = 0.0
    n = 0
    chirp = Chirp(250, 50, .25)
    devA.set_current_gains()
    devB.set_current_gains()

    loop = SoftRealtimeLoop(dt = dt, report=True, fade=0.01)
    for t in loop:
        devA.update(), devB.update(), ahrsA.update(), ahrsB.update()
        tmp = str((devA.θ*180/3.1415,devB.θ*180/3.1415))
        τ = amp*chirp.next(t)*3/3.7 # a barely audible note
        devA.τ=τ * loop.fade
        devB.τ=τ * loop.fade

def main():
    with ActPackMan('/dev/ttyActPackA', gear_ratio=9, updateFreq=1000) as devA:
        with ActPackMan('/dev/ttyActPackB', gear_ratio=9, updateFreq=1000) as devB:
            with AhrsManager(csv_file_name="test_AhrsA.csv", port="/dev/ttyAhrsA") as ahrsA: 
                with AhrsManager(csv_file_name="test_AhrsB.csv", port="/dev/ttyAhrsB") as ahrsB:
                    fast_chirp_demo(devA, devB, ahrsA, ahrsB, amp=3.0)
    print("done with current_demo()")

if __name__ == '__main__':
    main()
