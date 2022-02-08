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

# Version of the SoftRealtimeLoop library
__version__="1.0.0"

class LoopKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGTERM, self.exit_gracefully)
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGHUP, self.exit_gracefully)

  def exit_gracefully(self,signum, frame):
    self.kill_now = True

class SoftRealtimeLoop(object):
  def __init__(self, dt=0.001):
    self.t0 = self.t1 = time.time()
    self.killer = LoopKiller()
    self.dt = dt

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

    while time.time()<self.t1 and not self.killer.kill_now:
      if signal.sigtimedwait([signal.SIGTERM,signal.SIGINT,signal.SIGHUP], 0):
        self.stop()
    if self.killer.kill_now:
      raise StopIteration
    self.t1+=self.dt
    return self.t1





def example_usage_2():
  """ Use a soft realtime loop to run a bound method of an object.
  
  Conveniently, the object is a timer that reports statistics about the timing
  error after the loop finishes. Note that, since we expect the loop to be
  terminated by a keyboard interrupt (SIGINT), we can expect the execution to
  continue beyond the line where we initiate the loop. This allows us to make
  the report. Some environments (e.g. sublime-text in-editor run) will kill
  python without sending SIGINT, and will not see the report. Connecting via
  ssh, and running python3 from the ssh terminal, you will send a SIGINT if
  you press CTRL-C, and should see the performance metrics.

  """

  print("Testing real-time performance. Press CTRL-C to finish.")
  class Timer():
    def __init__(self, dt):
      self.ttarg = time.time()+dt
      self.sum_err = 0.0
      self.sum_var = 0.0
      self.dt = dt
      self.n = 0
    def func(self):
      error = time.time()-self.ttarg # seconds
      self.sum_err += error
      self.sum_var += error**2
      self.n+=1
      self.ttarg+=self.dt
    def report(self):
      print('In %d cycles:'%self.n)
      print('\tavg error: %.3f milliseconds'% (1e3*self.sum_err/self.n))
      print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((self.sum_var-self.sum_err**2/self.n)/(self.n-1))))
  dt = 0.0001
  myTimer=Timer(dt)
  SoftRealtimeLoop(dt = dt).run(myTimer.func, dt=myTimer.dt)
  myTimer.report()

def example_usage_3(dt = 0.0001):
  """ Use a soft realtime loop with the cool new for-loop syntax!

  After CTRL-C, the example will report on timing accuracy.
  """
  print("Testing real-time performance. Press CTRL-C to finish.")
  ttarg = None 
  sum_err = 0.0
  sum_var = 0.0
  n = 0
  for t in SoftRealtimeLoop(dt = dt):
    if ttarg is None: 
      # inits ttarg on first call
      ttarg = time.time()+dt
      # then skips the first loop
      continue
    error = time.time()-ttarg # seconds
    sum_err += error
    sum_var += error**2
    n+=1
    ttarg+=dt
  
  print('In %d cycles at %.2f Hz:'%(n, 1./dt))
  print('\tavg error: %.3f milliseconds'% (1e3*sum_err/n))
  print('\tstddev error: %.3f milliseconds'% (1e3*sqrt((sum_var-sum_err**2/n)/(n-1))))

if __name__ == '__main__':
  example_usage_3()

 
