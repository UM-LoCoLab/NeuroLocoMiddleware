import sys
import os
import time

# Add the parent folder to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from VariableUpdater import VariableUpdater

# Create a VariableUpdater object with an initial value of 0.0
updater = VariableUpdater(initial_value=0.0)

# Simulate control loop
while True:
    print(f"Current variable value: {updater.variable}")
    # Simulate work
    time.sleep(0.3)
