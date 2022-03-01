"""
Soft Realtime Loop---a class designed to allow clean exits from infinite loops
with the potential for post-loop cleanup operations executing.

The Loop Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Soft Realtime Loop to stop iterating.
Typically, it detects the CTRL-C from your keyboard, which sends a SIGTERM signal.

the function_in_loop argument to the Soft Realtime Loop's blocking_loop method is the function to be run every loop.
A typical usage would set function_in_loop to be a method of an object, so that the object could store program state.
See the 'ifmain' for two examples.
"""

import signal
import time
import asyncio
from math import sqrt
# print(dir(asyncio))
# print(asyncio.__name__)
# exit()
PRECISION_OF_SLEEP = 0.0001

# Version of the SoftRealtimeLoop library
__version__="1.0.0"

class LoopKiller:
  def __init__(self, fade_time=0.0):
    signal.signal(signal.SIGTERM, self.handle_signal)
    signal.signal(signal.SIGINT, self.handle_signal)
    signal.signal(signal.SIGHUP, self.handle_signal)
    self._fade_time = fade_time
    self._soft_kill_time = None

  def handle_signal(self,signum, frame):
    self.kill_now=True

  def get_fade(self):
    # interpolates from 1 to zero with soft fade out
    if self._kill_soon:
      t = time.time()-self._soft_kill_time
      if t>=self._fade_time:
        return 0.0
      return 1.0-(t/self._fade_time)
    return 1.0

  _kill_now = False
  _kill_soon = False
  @property
  def kill_now(self):
    if self._kill_now:
      return True
    if self._kill_soon:
      t = time.time()-self._soft_kill_time
      if t>self._fade_time:
        self._kill_now=True
    return self._kill_now

  @kill_now.setter
  def kill_now(self, val):
    if val:
      if self._kill_soon: # if you kill twice, then it becomes immediate
        self._kill_now = True
      else:
        if self._fade_time > 0.0:
          self._kill_soon = True
          self._soft_kill_time = time.time()
        else:
          self._kill_now = True
    else:
      self._kill_now = False
      self._kill_soon = False
      self._soft_kill_time = None

class SoftRealtimeLoop(object):
  def __init__(self, dt=0.001, report=False, fade=0.0):
    self.t0 = self.t1 = time.time()
    self.killer = LoopKiller(fade_time=fade)
    self.dt = dt
    self.ttarg = None 
    self.sum_err = 0.0
    self.sum_var = 0.0
    self.sleep_t_agg = 0.0
    self.n = 0
    self.report=report

  def __del__(self):
    if self.report:
      print('In %d cycles at %.2f Hz:'%(self.n, 1./self.dt))
      print('\tavg error: %.3f milliseconds'% (1e3*self.sum_err/self.n))
      print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((self.sum_var-self.sum_err**2/self.n)/(self.n-1))))
      print('\tpercent of time sleeping: %.1f %%' % (self.sleep_t_agg/self.time()*100.))

  @property
  def fade(self):
    return self.killer.get_fade()

  def run(self, function_in_loop, dt=None):
    if dt is None:
      dt = self.dt
    self.t0 = self.t1 = time.time()+dt
    while not self.killer.kill_now:
      ret = function_in_loop()
      if ret==0:
        self.stop()
      while time.time()<self.t1 and not self.killer.kill_now:
        if signal.sigtimedwait([signal.SIGTERM,signal.SIGINT,signal.SIGHUP], 0):
          self.stop()
      self.t1+=dt
    print("Soft realtime loop has ended successfully.")

  def run(self, function_in_loop, dt=None):
    if dt is None:
      dt = self.dt
    for t in self:
      ret = function_in_loop()
      if ret==0:
        self.stop()
    print("Soft realtime loop has ended successfully.")

  def stop(self):
    self.killer.kill_now=True

  def time(self):
    return time.time()-self.t0

  def time_since(self):
    return time.time()-self.t1

  def __iter__(self):
    self.t0 = self.t1 = time.time()+self.dt
    return self

  def __next__(self):
    if self.killer.kill_now:
      raise StopIteration

    while time.time()<self.t1-2*PRECISION_OF_SLEEP and not self.killer.kill_now:
      t_pre_sleep = time.time()
      time.sleep(max(PRECISION_OF_SLEEP,self.t1-time.time()-PRECISION_OF_SLEEP))
      self.sleep_t_agg+=time.time()-t_pre_sleep

    while time.time()<self.t1 and not self.killer.kill_now:
      if signal.sigtimedwait([signal.SIGTERM,signal.SIGINT,signal.SIGHUP], 0):
        self.stop()
    if self.killer.kill_now:
      raise StopIteration
    self.t1+=self.dt
    if self.ttarg is None: 
      # inits ttarg on first call
      self.ttarg = time.time()+self.dt
      # then skips the first loop
      return self.t1-self.t0
    error = time.time()-self.ttarg # seconds
    self.sum_err += error
    self.sum_var += error**2
    self.n+=1
    self.ttarg+=self.dt
    return self.t1-self.t0


