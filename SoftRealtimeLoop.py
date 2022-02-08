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
    return self.t1-self.t0


