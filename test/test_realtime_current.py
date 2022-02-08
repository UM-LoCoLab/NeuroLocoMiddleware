from sys import path
try:
  from SoftRealtimeLoop import SoftRealtimeLoop
except ModuleNotFoundError:
  print("module SoftRealtimeLoop not found in path: %s"%path)
  print("to add SoftRealtimeLoop to the path, edit the .bashrc file like so:")
  print("""
export PYTHONPATH={$PYTHONPATH}:/home/pi/NeuroLocoMiddleware.
    """)
  path.append("/home/pi/NeuroLocoMiddleware")
  from SoftRealtimeLoop import SoftRealtimeLoop
from ActPackMan import ActPackMan
from math import sin, pi, sqrt, log, exp
import time


def current_demo(dev, amp=1.0):
    """ Tests 1000Hz performance"""
    dev.set_current_gains(kp=40, ki=400, ff=128)
    for t in SoftRealtimeLoop(dt=0.001):
        dev.i=amp*sin(t*55*2*pi) # a musical note three octaves below 440 Hz A

class Chirp():
    def __init__(self, start_freq_Hz, end_freq_Hz, time, repeat=True):
        self._start_freq = start_freq_Hz*2*pi
        self._end_freq = end_freq_Hz*2*pi
        self._maxtime = time
        self._repeat = repeat
        self._end_log_ω = log(self._end_freq)
        self._phase_growth_rate = log(self._end_freq/self._start_freq)/self._maxtime
        self._sign_growth = self._phase_growth_rate/abs(self._phase_growth_rate)
        self._phase = 0.0
        self._last_t = 0.0
        self._log_ω = log(self._start_freq)
        assert(self._log_ω*self._sign_growth < self._end_log_ω*self._sign_growth)

    def next(self, t):
        dt = (t-self._last_t)
        self._last_t = t
        self._log_ω+=self._phase_growth_rate*dt
        if self._log_ω*self._sign_growth >= self._end_log_ω*self._sign_growth:
            if self._repeat:
                self._log_ω-=self._phase_growth_rate*self._maxtime
            else:
                raise StopIteration()
        self._phase+=exp(self._log_ω)*dt
        return sin(self._phase)



def current_demo(dev, amp=1.0, dt=0.001):
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
        error = time.time()-ttarg # seconds
        sum_err += error
        sum_var += error**2
        n+=1
        ttarg+=dt
      
    print('In %d cycles at %.2f Hz:'%(n, 1./dt))
    print('\tavg error: %.3f milliseconds'% (1e3*sum_err/n))
    print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((sum_var-sum_err**2/n)/(n-1))))

if __name__ == '__main__':
    with ActPackMan('/dev/ttyACM0', updateFreq=1000) as dev:
        current_demo(dev, amp=3.0)
    print("done with current_demo()")