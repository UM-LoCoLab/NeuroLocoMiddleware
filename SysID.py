import numpy as np
from math import pi, exp, log, sin, cos

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
