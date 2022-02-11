from sys import path
try:
  from SoftRealtimeLoop import SoftRealtimeLoop
except ModuleNotFoundError:
  print("module SoftRealtimeLoop not found in path: %s"%path)
  print("to add SoftRealtimeLoop to the path, edit the .bashrc file like so:")
  print("""
export PYTHONPATH={$PYTHONPATH}:/home/pi/NeuroLocoMiddleware.
    """)
  print("if you are using ssh, you will need to change the .bash_profile to include")
  print("""if [ -f ~/.bashrc ]; then
  . ~/.bashrc""")
  path.append("/home/pi/NeuroLocoMiddleware")
  from SoftRealtimeLoop import SoftRealtimeLoop
import time
from math import sqrt



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
  dt = 0.001
  myTimer=Timer(dt)
  SoftRealtimeLoop(dt = dt, report=True).run(myTimer.func, dt=myTimer.dt)
  myTimer.report()

if __name__ == '__main__':
  example_usage_2()