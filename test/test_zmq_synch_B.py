from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ZmqBinarySynch import ZmqBinarySynchB
import numpy as np

synch = ZmqBinarySynchB(
    bindport="tcp://*:5558",
    connectport="tcp://localhost:5557")

for t in SoftRealtimeLoop(0.001, report=True):
    print(synch.update(np.array([t*1000, 1337.])))