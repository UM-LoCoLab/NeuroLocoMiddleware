from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
import time

def example_usage_1():
  """ Use a soft realtime loop to print the elapsed time almost exactly every 0.01 seconds. """
  print("Printing time since the start. Press CTRL-C to finish.")
  t0 = time.time()
  SoftRealtimeLoop(report=True).run(lambda: print("in the loop", time.time()-t0), dt=0.01)

if __name__ == '__main__':
  example_usage_1()