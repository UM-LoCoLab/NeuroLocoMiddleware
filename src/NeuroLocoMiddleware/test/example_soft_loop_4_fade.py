from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
import time
from math import sqrt

def example_usage_4(dt = 0.01):
  """ Use a soft realtime loop with the cool new for-loop syntax!

  After CTRL-C, the example will report on timing accuracy.
  """
  print("Testing real-time performance. Press CTRL-C once to fade out, or twice to finish now.")
  loop = SoftRealtimeLoop(dt = dt, report=True, fade=1.0)
  for t in loop:
    print(loop.fade) # normally one, but fades to zero after first CTRL-C

if __name__ == '__main__':
  example_usage_4()

 