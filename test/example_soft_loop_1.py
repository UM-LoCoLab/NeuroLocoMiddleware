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

def example_usage_1():
  """ Use a soft realtime loop to print the elapsed time almost exactly every 0.01 seconds. """
  print("Printing time since the start. Press CTRL-C to finish.")
  t0 = time.time()
  SoftRealtimeLoop().run(lambda: print("in the loop", time.time()-t0), dt=0.01)

if __name__ == '__main__':
  example_usage_1()