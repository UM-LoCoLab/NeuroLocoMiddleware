import time

from udpate_vars import VariableUpdater

# Create a VariableUpdater object with an initial value of 0.0
updater = VariableUpdater(initial_value=0.0)

# Simulate control loop
while True:
    print(f"Current variable value: {updater.variable}")
    # Simulate work
    time.sleep(0.3)  
