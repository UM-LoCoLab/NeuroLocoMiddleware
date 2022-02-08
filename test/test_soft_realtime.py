from sys import path
try:
  from SoftRealtimeLoop import example_usage_3
except ModuleNotFoundError:
  print("module SoftRealtimeLoop not found in path: %s"%path)
  print("to add SoftRealtimeLoop to the path, edit the .bashrc file like so:")
  print("""
export PYTHONPATH={$PYTHONPATH}:/home/pi/NeuroLocoMiddleware.
    """)
  path.append("/home/pi/NeuroLocoMiddleware")
  from SoftRealtimeLoop import example_usage_3

if __name__ == '__main__':
  example_usage_3()
