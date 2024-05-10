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
import heapq
# print(dir(asyncio))
# print(asyncio.__name__)
# exit()
PRECISION_OF_SLEEP = 0.0001 # microseconds

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
      t = time.monotonic()-self._soft_kill_time
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
      t = time.monotonic()-self._soft_kill_time
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
          self._soft_kill_time = time.monotonic()
        else:
          self._kill_now = True
    else:
      self._kill_now = False
      self._kill_soon = False
      self._soft_kill_time = None

class SoftRealtimeLoop(object):
  def __init__(self, dt=0.001, report=False, fade=0.0, 
               max_error_trigger_value: float = float('inf'), 
               max_error_trigger_kill: bool = False,):
    """
    The SoftRealtimeLoop object is a class designed to allow perform smart
    loops that can approximate a real time operating system. It also allows
    clean exits from infinite loops with the potential for post-loop cleanup 
    operations executing. You can also kill the loop if it exceeds a certain
    error threshold.

    Parameters
    ----------
    dt : float
        The time step of the loop in seconds.
    report : bool
        If True, the loop will print a report at the end of the loop.
    fade : float
        The time in seconds to fade out the loop when it is killed.
    max_error_trigger_value : float
        The maximum error value in seconds that the loop can have before it is 
        killed. The default value is infinity (i.e. it is never triggered).
    max_error_trigger_kill : bool
        If True, the loop will be killed if a loop error exceeds the max error. 
        The default value is False. 
    """
    self.t0 = self.t1 = time.monotonic()
    self.killer = LoopKiller(fade_time=fade)
    self.dt = dt
    self.ttarg = None 
    self.sum_err = 0.0
    self.sum_var = 0.0
    self.sleep_t_agg = 0.0
    self.n = 0
    self.report=report
    self.max_errors = [0.0, 0.0, 0.0, 0.0, 0.0]
    # Max error trigger
    self.max_error_trigger_value = max_error_trigger_value
    self.max_error_trigger_kill = max_error_trigger_kill

  def __del__(self):
    if self.report:
      print('In %d cycles at %.2f Hz:'%(self.n, 1./self.dt))
      print('\tavg error: %.3f milliseconds'% (1e3*self.sum_err/self.n))
      print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((self.sum_var-self.sum_err**2/self.n)/(self.n-1))))
      print('\tpercent of time sleeping: %.1f %%' % (self.sleep_t_agg/self.time()*100.))
      print('\tfive max cycle errors: %.3f, %.3f, %.3f, %.3f, %.3f milliseconds'% (1e3*self.max_errors[0], 1e3*self.max_errors[1], 1e3*self.max_errors[2], 1e3*self.max_errors[3], 1e3*self.max_errors[4]))

  @property
  def fade(self):
    return self.killer.get_fade()

  def run(self, function_in_loop, dt=None):
    if dt is None:
      dt = self.dt
    self.t0 = self.t1 = time.monotonic()+dt
    while not self.killer.kill_now:
      ret = function_in_loop()
      if ret==0:
        self.stop()
      while time.monotonic()<self.t1 and not self.killer.kill_now:
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
    return time.monotonic()-self.t0

  def time_since(self):
    return time.monotonic()-self.t1

  def __iter__(self):
    self.t0 = self.t1 = time.monotonic()+self.dt
    return self

  def __next__(self):

    # If the loop is killed, raise a StopIteration
    if self.killer.kill_now:
      raise StopIteration

    # Sleep the amount we need to satisfy the dt.
    while time.monotonic()<self.t1-2*PRECISION_OF_SLEEP and not self.killer.kill_now:
      t_pre_sleep = time.monotonic()
      time.sleep(max(PRECISION_OF_SLEEP,self.t1-time.monotonic()-PRECISION_OF_SLEEP))
      # Update the time spent sleeping to calculate the sleep percentage
      self.sleep_t_agg+=time.monotonic()-t_pre_sleep

    # If we got any signals while we were sleeping, indicate that we 
    # should kill the loop
    while time.monotonic()<self.t1 and not self.killer.kill_now:
      if signal.sigtimedwait([signal.SIGTERM,signal.SIGINT,signal.SIGHUP], 0):
        self.stop()

    # If the loop is killed while we were waiting, raise a StopIteration
    if self.killer.kill_now:
      raise StopIteration
    
    # Increase the dt naively based on the time that we should have slept
    self.t1+=self.dt
    
    # Initialize ttarg on first call
    if self.ttarg is None: 
      self.ttarg = time.monotonic()+self.dt
      # then skips the first loop
      return self.t1-self.t0

    # Calculate the error for the loop and update the max errors
    error = time.monotonic()-self.ttarg # seconds
    self.sum_err += error
    self.sum_var += error**2
    self.n+=1
    self.ttarg+=self.dt

    # Update the max errors
    if error > self.max_errors[0]:
      heap = self.max_errors + [error]
      heapq.heapify(heap)
      heapified_heap = [heapq.heappop(heap) for _ in range(len(heap))]
      self.max_errors = heapified_heap[1:]

    # If the error exceeds the max error trigger value, either inform the 
    # user or kill the loop
    if error > self.max_error_trigger_value:
      if self.max_error_trigger_kill:
        self.stop()
      else:
        print(f"SoftRealTimeLoop: Error in loop exceeded the"
              f"max_error_trigger_value: {1e3*error:.3f} milliseconds")

    return self.t1-self.t0


