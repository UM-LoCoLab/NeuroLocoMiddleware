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

 