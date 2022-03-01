from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
import time
from math import sqrt

def example_usage_3(dt = 0.001):
  """ Use a soft realtime loop with the cool new for-loop syntax!

  After CTRL-C, the example will report on timing accuracy.
  """
  print("Testing real-time performance. Press CTRL-C to finish.")
  ttarg = None 
  sum_err = 0.0
  sum_var = 0.0
  n = 0
  for t in SoftRealtimeLoop(dt = dt, report=True):
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

if __name__ == '__main__':
  example_usage_3()

 