import sys
import os

# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory
parent_dir = os.path.dirname(current_dir)

# Add the parent directory to the system path
sys.path.append(parent_dir)

from SoftRealtimeLoop import SoftRealtimeLoop
from time import monotonic

def test_loop(loop):
    start_time = monotonic()
    n = 0
    for t in loop:
        n += 1
        if t > 5:
            loop.stop()
    end_time = monotonic()
    print("True loop time {:.4f}, num cycles {:.0f}, avg dt {:.4f}".format(end_time - start_time, n, (end_time - start_time)/n))
    
loop1 = SoftRealtimeLoop(0.01, report=True, track_naive_time=False)
print('Loop (not maintain original phase) Started for 5 seconds:')
test_loop(loop1)

loop2 = SoftRealtimeLoop(0.01, report=True, track_naive_time=True)
print('Loop (maintain original phase) Started for 5 seconds:')
test_loop(loop2)