from FindLibrariesWarning import *
from SoftRealtimeLoop import SoftRealtimeLoop
from ZmqBinarySynch import ZmqBinarySynchA
import numpy as np

synch = ZmqBinarySynchA(
    bindport="tcp://*:5557",
    connectport="tcp://localhost:5558")

for t in SoftRealtimeLoop(0.001, report=True):
    print(synch.update(np.array([42.1, t*1000])))