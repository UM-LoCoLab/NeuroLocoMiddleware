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
  try:
    path.append("/home/pi/NeuroLocoMiddleware")
    from SoftRealtimeLoop import SoftRealtimeLoop
  except ModuleNotFoundError:
    print("NeuroLocoMiddleware not found in path: /home/pi/NeuroLocoMiddleware")
    print("Are you not on the pi?")
    import os
    dir_path = os.path.dirname(os.path.realpath(__file__))
    path.append(dir_path+"/..") # relative path hack for tests
    from SoftRealtimeLoop import SoftRealtimeLoop